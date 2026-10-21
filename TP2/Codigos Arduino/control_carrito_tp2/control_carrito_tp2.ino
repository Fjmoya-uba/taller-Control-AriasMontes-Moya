/* Control SISO: error [cm] -> C(z) -> variacion de pulso [us] -> servo.
 * G(s) servo-barra ya forma parte de H(s): NO cerrar otro lazo de angulo.
 * HC-SR04: TRIG 6, ECHO 7; servo 9. IMU omitida para reducir jitter.
 * MATLAB: "abcd" + [referencia_cm, distancia_cm, pulso_us], float32 LE.
 * LED latcheado: periodo excedido o trama omitida. Sin texto por Serial.
 * Control y disparos del sonar a 50 Hz (20 ms), con captura no bloqueante.


 */
#include <Arduino.h>
#include <Servo.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

// TP2: mismo protocolo y adquisicion que control_carrito_matlab_v3.

// TP2: p = d - d0. REFERENCIA_CM es d0, no una posicion relativa.
const float REFERENCIA_CM = 15.612f;
constexpr float POSICION_MIN_CM = -15.0f, POSICION_MAX_CM = 21.0f;
// Consigna relativa al cero fisico. La telemetria conserva distancias absolutas.
// true: escalon automatico; false: referencia cero para aplicar un golpe.
constexpr bool ENSAYO_ESCALON = false;
constexpr uint32_t INSTANTE_ESCALON_MS = 5000UL;
constexpr float CONSIGNA_POSICION_CM = 10.0f; // 0.1 m = 10 cm.
float referenciaPosicion_cm = 0.0f;
uint32_t inicioEnsayo_ms;
// C(z) en forma directa: salida incremental [us], error [cm].
const float COEF_A1 = 0.886792453f;
const float COEF_A2 = 0.0f;
const float COEF_B0 = 15.4003772f;
const float COEF_B1 = -14.5032679f;
const float COEF_B2 = 0.0f;
constexpr float SIGNO_CONTROL = 1.0f;
// Discretizacion bilineal (Tustin) en MATLAB, con Ts = 0.02 s:
// Cz = c2d(C_total, 0.02, 'tustin'); [b,a] = tfdata(tf(Cz),'v');
// COEF_A1 = -a(2)/a(1); COEF_A2 = -a(3)/a(1);
// COEF_B0 = b(1)/a(1); COEF_B1 = b(2)/a(1); COEF_B2 = b(3)/a(1);
// Completar con cero los coeficientes ausentes si el orden es menor que dos.
// P inicial. Para PI o PD reemplazar estos cinco coeficientes por los de Tustin.
// Misma calibracion mecanica que v2: Servo.write(7 + 90), attach por defecto.
const int ANGULO_SERVO_CERO = 7;
const int PULSO_CENTRO = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_CERO + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
// Con MIN_PULSE_WIDTH=544 y MAX_PULSE_WIDTH=2400, el centro es 1544 us.
const int PULSO_MIN = 1100; // TP2: debajo de este valor la barra toca el piso.
const int PULSO_MAX = 2000;
const uint32_t TS_US = 20000UL;
const uint32_t SONAR_US = TS_US; // Un disparo cada 20 ms: 50 Hz.
// HC-SR04 recomienda >60 ms entre disparos; validar ecos a 20 ms en el montaje.
const uint32_t RESERVA_TX_US = 2500UL; // 16 bytes a 115200: ~1389 us.
const uint8_t PIN_TRIG = 6, PIN_ECHO = 7, PIN_SERVO = 9;
const float US_POR_CM = 58.574f; // Misma conversion que en v2.
const uint32_t TIMEOUT_ECO_US = 4000UL;
static_assert(TIMEOUT_ECO_US < SONAR_US,
              "El eco debe finalizar antes del siguiente disparo");
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(PULSO_MIN <= PULSO_CENTRO && PULSO_CENTRO <= PULSO_MAX,
              "El centro debe estar dentro de los limites");
static_assert(POSICION_MIN_CM < POSICION_MAX_CM &&
              CONSIGNA_POSICION_CM >= POSICION_MIN_CM &&
              CONSIGNA_POSICION_CM <= POSICION_MAX_CM, "Rango de posicion invalido");

static_assert(SIGNO_CONTROL == 1.0f || SIGNO_CONTROL == -1.0f,
              "El signo debe ser +1 o -1");

Servo servo;
uint32_t siguienteControl, siguienteSonar;
uint32_t periodosExcedidos = 0, tramasOmitidas = 0;
float distancia_cm = -1.0f;
bool distanciaValida = false;
float errorAnteriorControl = 0.0f, salidaAnteriorControl = 0.0f;
float errorDosAtrasControl = 0.0f, salidaDosAtrasControl = 0.0f;
int pulso = PULSO_CENTRO;

enum EstadoSonar { REPOSO, TRIGGER_ALTO, ESPERA_SUBIDA, ESPERA_BAJADA };
EstadoSonar estadoSonar = REPOSO;
uint32_t inicioPing, inicioEco;

bool vencido(uint32_t ahora, uint32_t instante) {
  // Comparacion modular: funciona tambien al desbordarse micros().
  return (int32_t)(ahora - instante) >= 0;
}

void reiniciarControlador() {
  errorAnteriorControl = 0.0f;
  errorDosAtrasControl = 0.0f;
  salidaAnteriorControl = 0.0f;
  salidaDosAtrasControl = 0.0f;
}

void finalizarEco(float medida) {
  const float posicion = medida - REFERENCIA_CM;
  // Los limites son relativos. No recortar lecturas invalidas a los extremos.
  // Este rango amplio no detecta ecos falsos dentro de el (ver ensayo TP2).
  distanciaValida = isfinite(medida) && medida > 0.0f &&
                   posicion >= POSICION_MIN_CM && posicion <= POSICION_MAX_CM;
  distancia_cm = distanciaValida ? medida : -1.0f;
  if (!distanciaValida) reiniciarControlador();
  estadoSonar = REPOSO;
}

void atenderSonar(uint32_t ahora) {
  // Se llama continuamente. Sin ping(), pulseIn(), delay() ni timers
  // adicionales que puedan entrar en conflicto con la libreria Servo.
  switch (estadoSonar) {
    case REPOSO:
      if (vencido(ahora, siguienteSonar)) {
        // Mantener la grilla temporal; omitir disparos atrasados sin rafagas.
        const uint32_t periodos = (ahora - siguienteSonar) / SONAR_US + 1;
        siguienteSonar += periodos * SONAR_US;
        inicioPing = ahora;
        digitalWrite(PIN_TRIG, HIGH);
        estadoSonar = TRIGGER_ALTO;
      }
      break;
    case TRIGGER_ALTO:
      if (ahora - inicioPing >= 10UL) {
        digitalWrite(PIN_TRIG, LOW);
        estadoSonar = ESPERA_SUBIDA;
      }
      break;
    case ESPERA_SUBIDA:
      if (ahora - inicioPing >= TIMEOUT_ECO_US) finalizarEco(-1.0f);
      else if (digitalRead(PIN_ECHO) == HIGH) {
        inicioEco = ahora;
        estadoSonar = ESPERA_BAJADA;
      }
      break;
    case ESPERA_BAJADA:
      if (ahora - inicioPing >= TIMEOUT_ECO_US) finalizarEco(-1.0f);
      else if (digitalRead(PIN_ECHO) == LOW)
        finalizarEco((ahora - inicioEco) / US_POR_CM);
      break;
  }
}

void actualizarControl() {
  // El escalon ocurre una sola vez, contado desde setup().
  // Mantener la consigna despues del escalon, incluso al desbordarse millis().
  if (ENSAYO_ESCALON && referenciaPosicion_cm == 0.0f &&
      (uint32_t)(millis() - inicioEnsayo_ms) >= INSTANTE_ESCALON_MS)
    referenciaPosicion_cm = CONSIGNA_POSICION_CM;
  if (!distanciaValida) {
    reiniciarControlador();
    pulso = PULSO_CENTRO;
  } else {
    // ZOH: distancia_cm cambia al terminar el eco de cada disparo a 50 Hz.
    const float posicion = distancia_cm - REFERENCIA_CM;
    const float error = referenciaPosicion_cm - posicion;
    const float salida = COEF_A1 * salidaAnteriorControl
                       + COEF_A2 * salidaDosAtrasControl
                       + COEF_B0 * error
                       + COEF_B1 * errorAnteriorControl
                       + COEF_B2 * errorDosAtrasControl;
    const float mando = PULSO_CENTRO + SIGNO_CONTROL * salida;
    const float mandoLimitado = constrain(mando, (float)PULSO_MIN,
                                                 (float)PULSO_MAX);
    pulso = (int)lroundf(mandoLimitado);
    errorDosAtrasControl = errorAnteriorControl;
    errorAnteriorControl = error;
    salidaDosAtrasControl = salidaAnteriorControl;
    // Realimentar la salida limitada, en coordenadas del controlador.
    // Sin saturacion conserva C(z); al saturar limita la memoria recursiva.
    // Es una proteccion simple, no un anti-windup universal para cualquier C(z).
    // No incluir el redondeo a us para evitar cuantizacion en la recurrencia.
    salidaAnteriorControl = SIGNO_CONTROL * (mandoLimitado - PULSO_CENTRO);
  }
  servo.writeMicroseconds(pulso);
}

void enviarRegistro(uint32_t limite) {
  if ((int32_t)(limite - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 16) {
    ++tramasOmitidas;
    digitalWrite(LED_BUILTIN, HIGH);
    return; // No bloquear esperando espacio ni mandar una trama parcial.
  }
  uint8_t trama[16] = {'a', 'b', 'c', 'd'};
  const float valores[3] = {REFERENCIA_CM + referenciaPosicion_cm,
                           distanciaValida ? distancia_cm : -1.0f,
                           (float)pulso};
  for (uint8_t i = 0; i < 3; ++i) {
    uint32_t bits;
    memcpy(&bits, &valores[i], sizeof(bits));
    for (uint8_t j = 0; j < 4; ++j)
      trama[4 + 4 * i + j] = (uint8_t)(bits >> (8 * j));
  }
  Serial.write(trama, sizeof(trama)); // UART en segundo plano, sin flush().
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  Serial.begin(115200);
  servo.writeMicroseconds(PULSO_CENTRO);
  servo.attach(PIN_SERVO);
  const uint32_t ahora = micros();
  siguienteControl = ahora + TS_US;
  siguienteSonar = ahora; // Primer eco antes del primer control.
  inicioEnsayo_ms = millis();
}

void loop() {
  uint32_t ahora = micros();
  if (vencido(ahora, siguienteControl)) {
    siguienteControl += TS_US;
    if (vencido(ahora, siguienteControl)) {
      // Contar plazos perdidos y descartar ejecuciones atrasadas.
      const uint32_t perdidos = (ahora - siguienteControl) / TS_US + 1;
      periodosExcedidos += perdidos;
      siguienteControl += perdidos * TS_US;
      digitalWrite(LED_BUILTIN, HIGH);
      reiniciarControlador();
    }
    actualizarControl();
    enviarRegistro(siguienteControl);
    ahora = micros();
    if (vencido(ahora, siguienteControl)) {
      const uint32_t perdidos = (ahora - siguienteControl) / TS_US + 1;
      periodosExcedidos += perdidos;
      siguienteControl += perdidos * TS_US;
      digitalWrite(LED_BUILTIN, HIGH);
      reiniciarControlador();
    }
  }
  // El control/TX se atiende antes de disparar el sonar. El eco se captura
  // en las vueltas libres del loop; la UART no detiene esta captura.
  atenderSonar(micros());
}
