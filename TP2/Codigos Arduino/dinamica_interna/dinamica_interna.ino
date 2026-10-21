/* Control de la dinamica interna servo-barra, a 50 Hz.
 * r[deg] -> C(z) -> delta PWM[us] -> servo/barra -> IMU -> filtro -> y[deg].
 * Mantener quieta la barra durante el arranque: el cero se mide a 1544 us.
 * TX a MATLAB: "abcd" + referencia_deg + angulo_deg + servo_us,
 * tres float32 little-endian (16 bytes). Sin texto ni salto de linea.
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
const float REFERENCIA_DEG = 5.0f; // Cambiar aqui el angulo deseado de la barra.

// Coeficientes DISCRETOS para Ts=0.02 s. Salida: delta PWM en us.
// C(z) = (b0 + b1*z^-1 + ...)/(a0 + a1*z^-1 + ...).
// En MATLAB: [b,a] = tfdata(tf(Cz), 'v'); copiar b y a aqui.
// Si el numerador tiene menor grado, anteponer ceros a b hasta que
// tenga la misma longitud que a (conserva los retardos de C(z)).
// Ejemplo actual: C(z)=3.162278*(z+1)/(z-1).
// u[k] = u[k-1] + 3.162278*e[k] + 3.162278*e[k-1].
const float B[] = {3.162278f, 3.162278f};
const float A[] = {1.0f, -1.0f}; // Copiar signos de a; no invertirlos.
const size_t N_COEF = sizeof(A) / sizeof(A[0]);
static_assert(sizeof(B) == sizeof(A), "B y A deben tener igual longitud");
static_assert(N_COEF >= 1, "Se requiere al menos un coeficiente");
// A[0] debe ser distinto de cero. Validar la sintonia en la planta.
// +1 si aumentar PWM aumenta el angulo medido; -1 si el montaje lo invierte.
const float SIGNO_ACTUADOR = 1.0f;
const uint32_t RESERVA_TX_US = 2500UL; // 16 bytes a 115200: ~1.39 ms.
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

float controlar(float error) {
  for (size_t i = N_COEF - 1; i > 0; --i) errores[i] = errores[i - 1];
  errores[0] = SIGNO_ACTUADOR * error;
  // a0*u[k] = sum(bi*e[k-i]) - sum(ai*u[k-i]), i>=1 en la segunda suma.
  float mando = B[0] * errores[0];
  for (size_t i = 1; i < N_COEF; ++i)
    mando += B[i] * errores[i] - A[i] * mandosUs[i - 1];
  mando /= A[0];
  const float pedido = PULSO_CENTRO_US + mando;
  const float limitado = constrain(pedido, (float)PULSO_MIN_US,
                                   (float)PULSO_MAX_US);
  // Realimentar el mando limitado a la recurrencia evita windup en el
  // integrador actual. Para otro C(z), revisar su comportamiento al saturar.
  for (size_t i = N_COEF - 1; i > 0; --i) mandosUs[i] = mandosUs[i - 1];
  mandosUs[0] = limitado - PULSO_CENTRO_US;
  return (float)lroundf(limitado);
}

void enviarRegistro() {
  if ((int32_t)(siguienteUs - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 16) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  uint8_t trama[16] = {'a', 'b', 'c', 'd'};
  const float valores[] = {REFERENCIA_DEG, anguloDeg, pulsoUs};
  for (uint8_t i = 0; i < 3; ++i) {
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
  if (A[0] == 0.0f) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100); // Denominador invalido; no activar el servo.
  }
  // Precargar el cero antes de habilitar los pulsos.
  servo.writeMicroseconds(PULSO_CENTRO_US);
  servo.attach(PIN_SERVO, PULSO_MIN_US, PULSO_MAX_US);
  Wire.begin();
  Wire.setClock(400000);
  if (!mpu.begin(0x68)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
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
  pulsoUs = controlar(REFERENCIA_DEG - anguloDeg);
  servo.writeMicroseconds((int)pulsoUs);
  enviarRegistro();
  // LED persistente: plazo perdido o trama omitida durante el ensayo.
  if ((int32_t)(micros() - siguienteUs) >= 0)
    digitalWrite(LED_BUILTIN, HIGH);
}
