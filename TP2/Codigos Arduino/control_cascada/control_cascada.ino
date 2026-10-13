/* Control en cascada a 50 Hz: periodo 0.02 s = 20 ms.
 * EXTERNA: error posicion[cm] -> Cexterno(z) -> referencia angulo[deg].
 * INTERNA: error angulo[deg] -> Cinterno(z) -> delta PWM[us].
 * Servo 9; sonar TRIG 6, ECHO 7; MPU6050 por I2C.
 * TX: abcd + cuatro float32 LE (20 bytes): servo_us, posicion_cm,
 * referencia_angulo_deg, angulo_filtrado_deg.
 * Mantener quieta la barra al calibrar el cero en 1544 us.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>

const uint8_t PIN_SERVO = 9;
const int ANGULO_CENTRO_SERVO_DEG = 90 + 7;
// Equivalente calibrado de ese centro. Usar microsegundos evita depender
// del mapeo de Servo.write(97); el angulo de referencia es el de la BARRA.
const int PULSO_CENTRO_US = 1544;
const int PULSO_MIN_US = 1000;
const int PULSO_MAX_US = 2000;
const uint32_t TS_US = 20000UL;
const float ALPHA = 0.94f;
// ================= EXTERNA: ANGULO -> POSICION =================
const float REFERENCIA_CM = 17.0f;
const float MAX_ANGULO_DEG = 14.0f;
// Pegar coeficientes del controlador externo, disenado con entrada en cm
// y salida en GRADOS. No reutilizar el controlador posicion -> PWM de v3.
// MATLAB: [b,a] = tfdata(tf(Cz_externo), 'v'), con Ts=0.02 s.
// Copiar signos originales; si b es mas corto, anteponer ceros hasta
// igualar la longitud de a. A_EXTERNO[0] debe ser distinto de cero.
// Pendientes: B=0 mantiene la barra horizontal hasta cargar TU controlador.
const float B_EXTERNO[] = {2.278345f, -2.246670f};
const float A_EXTERNO[] = {1.0f, -0.6f};
const size_t N_EXTERNO = sizeof(A_EXTERNO) / sizeof(A_EXTERNO[0]);
static_assert(sizeof(B_EXTERNO) == sizeof(A_EXTERNO), "Longitudes externas distintas");
const float SIGNO_POSICION = 1.0f; // +1 si angulo positivo aumenta distancia.
const uint8_t PIN_TRIG = 6, PIN_ECHO = 7;
const float MAX_DISTANCIA_CM = 35.0f;
const float US_POR_CM = 58.574f;
const uint32_t TIMEOUT_ECO_US = 4000UL;
static_assert(TIMEOUT_ECO_US < TS_US, "Timeout mayor que periodo");
float erroresExternos[N_EXTERNO] = {};
float mandosExternos[N_EXTERNO] = {};
float posicionCm = -1.0f;
float referenciaAnguloDeg = 0.0f;
bool posicionValida = false;

// ================= INTERNA: SERVO -> ANGULO =================

// Coeficientes DISCRETOS para Ts=0.02 s. Salida: delta PWM en us.
// C(z) = (b0 + b1*z^-1 + ...)/(a0 + a1*z^-1 + ...).
// En MATLAB: [b,a] = tfdata(tf(Cz), 'v'); copiar b y a aqui.
// Si el numerador tiene menor grado, anteponer ceros a b hasta que
// tenga la misma longitud que a (conserva los retardos de C(z)).
// Ejemplo actual: C(z)=3.162278*(z+1)/(z-1).
// u[k] = u[k-1] + 3.162278*e[k] + 3.162278*e[k-1].
const float B_INTERNO[] = {3.162278f, 3.162278f};
const float A_INTERNO[] = {1.0f, -1.0f}; // Copiar signos de a; no invertirlos.
const size_t N_COEF = sizeof(A_INTERNO) / sizeof(A_INTERNO[0]);
static_assert(sizeof(B_INTERNO) == sizeof(A_INTERNO), "B_INTERNO y A_INTERNO deben tener igual longitud");
static_assert(N_COEF >= 1, "Se requiere al menos un coeficiente");
// A_INTERNO[0] debe ser distinto de cero. Validar la sintonia en la planta.
// +1 si aumentar PWM aumenta el angulo medido; -1 si el montaje lo invierte.
const float SIGNO_ACTUADOR = 1.0f;
const uint32_t RESERVA_TX_US = 3000UL; // 20 bytes a 115200: ~1.74 ms.
const uint16_t MUESTRAS_ASENTAMIENTO = 50;
const uint16_t MUESTRAS_CERO = 150;
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(PULSO_MIN_US < PULSO_CENTRO_US &&
              PULSO_CENTRO_US < PULSO_MAX_US, "Centro fuera de limites");

Adafruit_MPU6050 mpu;
Servo servo;
float biasGyroRadS = 0.0f;
float anguloAccDeg = 0.0f;
float anguloFiltradoDeg = 0.0f;
float offsetAnguloDeg = 0.0f;
float anguloDeg = 0.0f;
float errores[N_COEF] = {}; // e[k], e[k-1], ...
float mandosUs[N_COEF] = {}; // u[k-1], u[k-2], ... (sin el centro)
float pulsoUs = PULSO_CENTRO_US;
uint32_t anteriorUs = 0;
uint32_t siguienteUs = 0;

void actualizarAngulo(float dt) {
  sensors_event_t a, g, temperatura;
  mpu.getEvent(&a, &g, &temperatura);
  anguloAccDeg = atan2(a.acceleration.y,
      sqrt(sq(a.acceleration.x) + sq(a.acceleration.z))) * 180.0f / PI;
  const float velocidadDegS = (g.gyro.x - biasGyroRadS) * 180.0f / PI;
  anguloFiltradoDeg = ALPHA * (anguloFiltradoDeg + velocidadDegS * dt)
                   + (1.0f - ALPHA) * anguloAccDeg;
  anguloDeg = anguloFiltradoDeg - offsetAnguloDeg;
}

void calibrar() {
  delay(2000); // Asentar el mecanismo en el cero fisico.
  float sumaGyro = 0.0f;
  for (uint16_t i = 0; i < 500; ++i) {
    sensors_event_t a, g, temperatura;
    mpu.getEvent(&a, &g, &temperatura);
    sumaGyro += g.gyro.x;
    delay(3);
  }
  biasGyroRadS = sumaGyro / 500.0f;
  actualizarAngulo(0.0f);
  anguloFiltradoDeg = anguloAccDeg;
  uint32_t previo = micros();
  uint32_t proximo = previo + TS_US;
  float sumaAngulo = 0.0f;
  for (uint16_t i = 0; i < MUESTRAS_ASENTAMIENTO + MUESTRAS_CERO; ++i) {
    while ((int32_t)(micros() - proximo) < 0) {}
    const uint32_t ahora = micros();
    actualizarAngulo((ahora - previo) * 1.0e-6f);
    previo = ahora;
    proximo = ahora + TS_US;
    if (i >= MUESTRAS_ASENTAMIENTO) sumaAngulo += anguloFiltradoDeg;
  }
  offsetAnguloDeg = sumaAngulo / MUESTRAS_CERO;
  anguloDeg = anguloFiltradoDeg - offsetAnguloDeg;
}

float controlarInterna(float error) {
  for (size_t i = N_COEF - 1; i > 0; --i) errores[i] = errores[i - 1];
  errores[0] = SIGNO_ACTUADOR * error;
  // a0*u[k] = sum(bi*e[k-i]) - sum(ai*u[k-i]), i>=1 en la segunda suma.
  float mando = B_INTERNO[0] * errores[0];
  for (size_t i = 1; i < N_COEF; ++i)
    mando += B_INTERNO[i] * errores[i] - A_INTERNO[i] * mandosUs[i - 1];
  mando /= A_INTERNO[0];
  const float pedido = PULSO_CENTRO_US + mando;
  const float limitado = constrain(pedido, (float)PULSO_MIN_US,
                                   (float)PULSO_MAX_US);
  // Realimentar el mando limitado a la recurrencia evita windup en el
  // integrador actual. Para otro C(z), revisar su comportamiento al saturar.
  for (size_t i = N_COEF - 1; i > 0; --i) mandosUs[i] = mandosUs[i - 1];
  mandosUs[0] = limitado - PULSO_CENTRO_US;
  return (float)lroundf(limitado);
}

// Un disparo por ciclo; espera del eco acotada a 4 ms.
void medirPosicion() {
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  const unsigned long ecoUs = pulseIn(PIN_ECHO, HIGH, TIMEOUT_ECO_US);
  posicionCm = ecoUs ? ecoUs / US_POR_CM : -1.0f;
  posicionValida = posicionCm > 0.0f && posicionCm <= MAX_DISTANCIA_CM;
  if (!posicionValida) posicionCm = -1.0f;
}

void controlarExterna() {
  if (!posicionValida) {
    memset(erroresExternos, 0, sizeof(erroresExternos));
    memset(mandosExternos, 0, sizeof(mandosExternos));
    referenciaAnguloDeg = 0.0f; // Sin eco, pedir barra horizontal.
    return;
  }
  for (size_t i = N_EXTERNO - 1; i > 0; --i)
    erroresExternos[i] = erroresExternos[i - 1];
  erroresExternos[0] = SIGNO_POSICION * (REFERENCIA_CM - posicionCm);
  float mando = B_EXTERNO[0] * erroresExternos[0];
  for (size_t i = 1; i < N_EXTERNO; ++i)
    mando += B_EXTERNO[i] * erroresExternos[i] - A_EXTERNO[i] * mandosExternos[i - 1];
  mando /= A_EXTERNO[0];
  referenciaAnguloDeg = constrain(mando, -MAX_ANGULO_DEG, MAX_ANGULO_DEG);
  for (size_t i = N_EXTERNO - 1; i > 0; --i)
    mandosExternos[i] = mandosExternos[i - 1];
  // Tracking limitado: revisar anti-windup para el controlador que se cargue.
  mandosExternos[0] = referenciaAnguloDeg;
}

void enviarRegistro() {
  if ((int32_t)(siguienteUs - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 20) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  uint8_t trama[20] = {'a', 'b', 'c', 'd'};
  const float valores[] = {pulsoUs, posicionCm, anguloDeg};
  for (uint8_t i = 0; i < 4; ++i) {
    uint32_t bits;
    memcpy(&bits, &valores[i], sizeof(bits));
    for (uint8_t j = 0; j < 4; ++j)
      trama[4 + 4 * i + j] = (uint8_t)(bits >> (8 * j));
  }
  Serial.write(trama, sizeof(trama)); // TX en segundo plano, sin flush().
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (A_INTERNO[0] == 0.0f || A_EXTERNO[0] == 0.0f) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100); // Denominador invalido; no activar el servo.
  }
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  // Precargar el cero antes de habilitar los pulsos.
  servo.writeMicroseconds(PULSO_CENTRO_US);
  servo.attach(PIN_SERVO, PULSO_MIN_US, PULSO_MAX_US);
  Wire.begin();
  Wire.setClock(400000);
  if (!mpu.begin(0x68)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  calibrar(); // Sin control ni tramas durante los ~7.5 s de calibracion.
  anteriorUs = micros();
  siguienteUs = anteriorUs + TS_US;
}

void loop() {
  const uint32_t ahora = micros();
  if ((int32_t)(ahora - siguienteUs) < 0) return;
  siguienteUs += TS_US;
  if ((int32_t)(ahora - siguienteUs) >= 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    siguienteUs = ahora + TS_US; // No recuperar pasos con la misma medicion.
  }
  const float dt = (ahora - anteriorUs) * 1.0e-6f;
  anteriorUs = ahora;
  actualizarAngulo(dt); // Medir y[k] antes de aplicar u[k].
  medirPosicion();
  controlarExterna();
  pulsoUs = controlarInterna(referenciaAnguloDeg - anguloDeg);
  servo.writeMicroseconds((int)pulsoUs);
  enviarRegistro();
  // LED persistente: plazo perdido o trama omitida durante el ensayo.
  if ((int32_t)(micros() - siguienteUs) >= 0)
    digitalWrite(LED_BUILTIN, HIGH);
}
