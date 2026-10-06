/* Control SISO: error [cm] -> C(z) -> variacion de pulso [us] -> servo.
 * G(s) servo-barra ya forma parte de H(s): NO cerrar otro lazo de angulo.
 * HC-SR04: TRIG 6, ECHO 7; servo 9. IMU omitida para reducir jitter.
 * MATLAB: "abcd" + [referencia_cm, distancia_cm, pulso_us], float32 LE.
 * LED latcheado: periodo excedido o trama omitida. Sin texto por Serial.
 */
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// Parametros de sintonia. C(z) entrega microsegundos, NO grados.
const float REFERENCIA_CM = 17.0f;
const float COEF_A = 0.600000f;
const float COEF_B0 = 52.563947f;
const float COEF_B1 = -51.936942f;
const float SIGNO_CONTROL = 1.0f; // +1 o -1 segun el sentido fisico.
// MATLAB, incluyendo la ganancia del controlador completo:
// Cz = c2d(C_total, 0.02, 'tustin'); [b,a] = tfdata(tf(Cz),'v');
// COEF_A = -a(2)/a(1); COEF_B0 = b(1)/a(1); COEF_B1 = b(2)/a(1).
// Misma calibracion mecanica que v2: Servo.write(7 + 90), attach por defecto.
const int ANGULO_SERVO_CERO = 7;
const int PULSO_CENTRO = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_CERO + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
// Con MIN_PULSE_WIDTH=544 y MAX_PULSE_WIDTH=2400, el centro es 1544 us.
const int PULSO_MIN = 1000;
const int PULSO_MAX = 2000;
const uint32_t TS_US = 20000UL;
const uint32_t SONAR_US = 60000UL;
const uint32_t RESERVA_TX_US = 2500UL; // 16 bytes a 115200: ~1389 us.
const uint8_t PIN_TRIG = 6, PIN_ECHO = 7, PIN_SERVO = 9;
const float MAX_DISTANCIA_CM = 35.0f;
const float US_POR_CM = 58.574f; // Misma conversion que en v2.
const uint32_t TIMEOUT_ECO_US = 4000UL;
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(PULSO_MIN <= PULSO_CENTRO && PULSO_CENTRO <= PULSO_MAX,
              "El centro debe estar dentro de los limites");

Servo servo;
uint32_t siguienteControl, siguienteSonar;
uint32_t periodosExcedidos = 0, tramasOmitidas = 0;
float distancia_cm = -1.0f;
bool distanciaValida = false;
float errorAnteriorControl = 0.0f, salidaAnteriorControl = 0.0f;
bool controladorInicializado = false;
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
  salidaAnteriorControl = 0.0f;
  controladorInicializado = false;
}

void finalizarEco(float medida) {
  distanciaValida = medida > 0.0f && medida <= MAX_DISTANCIA_CM;
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
        siguienteSonar = ahora + SONAR_US;
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
  if (!distanciaValida) {
    reiniciarControlador();
    pulso = PULSO_CENTRO;
  } else {
    // ZOH: distancia_cm solo cambia al terminar un eco, cada ~60 ms.
    const float error = REFERENCIA_CM - distancia_cm;
    if (!controladorInicializado) {
      // Inicializacion en equilibrio del filtro: evita golpe derivativo
      // al arrancar o recuperar el eco. No se arrastra historia invalida.
      errorAnteriorControl = error;
      salidaAnteriorControl = (COEF_B0 + COEF_B1) / (1.0f - COEF_A) * error;
      controladorInicializado = true;
    }
    const float salida = COEF_A * salidaAnteriorControl
                       + COEF_B0 * error + COEF_B1 * errorAnteriorControl;
    errorAnteriorControl = error;
    salidaAnteriorControl = salida; // Estado sin saturar: conserva C(z).
    const float mando = PULSO_CENTRO + SIGNO_CONTROL * salida;
    pulso = (int)constrain(lroundf(mando), (long)PULSO_MIN, (long)PULSO_MAX);
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
  const float valores[3] = {REFERENCIA_CM,
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
