/* Filtro complementario con MPU6050 y excitacion periodica del servo.
 *
 * Secuencia de entrada (en microsegundos):
 *   1544 -> 1750 -> 1544 -> 1350 -> 1544 -> ... (1.5 s por nivel)
 * Lectura, filtro, comando y TX en un unico ciclo de 20 ms (50 Hz).
 * LED encendido: plazo perdido o trama omitida; descartar ese ensayo.
 *
 * Trama binaria para MATLAB (12 bytes):
 *   "abcd" + angulo centrado (salida, grados) + comandoServoUs (entrada ordenada, us)
 * Antes de transmitir: estabilizar en 1544 us y estimar el cero angular.
 *
 * Los dos datos son float de 32 bits. No se imprime texto por Serial.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>

Adafruit_MPU6050 mpu;
Servo miServo;

const uint8_t PIN_SERVO = 9;
// Esta orden define la posicion de referencia de 0 grados.
const int PULSO_CENTRO_US = 1544;
const uint32_t ESTABILIZACION_SERVO_MS = 2000;
const uint16_t MUESTRAS_ASENTAMIENTO_FILTRO = 50; // 1 s a 50 Hz
const uint16_t MUESTRAS_OFFSET_ANGULO = 150; // Promediar durante 3 s
const uint32_t INTERVALO_EXCITACION_US = 1500000UL;

const uint16_t SECUENCIA_SERVO_US[] = {PULSO_CENTRO_US, 1750, PULSO_CENTRO_US, 1350};
const uint8_t CANTIDAD_PULSOS =
    sizeof(SECUENCIA_SERVO_US) / sizeof(SECUENCIA_SERVO_US[0]);

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 20000UL; // Ciclo completo a 50 Hz
const uint32_t RESERVA_TX_US = 2500UL; // 12 bytes a 115200: ~1042 us
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(INTERVALO_EXCITACION_US % SAMPLE_PERIOD_US == 0,
              "Cada escalon debe durar un numero entero de periodos");
const float ALPHA = 0.94;

float gyroX_offsetRadS = 0.0;
float anguloAccX = 0.0;
float anguloGyroX = 0.0;
float anguloFiltradoX = 0.0;
float offsetAnguloX = 0.0;
// Entrada u: orden enviada al servo; no es una medicion de pulsos ni de posicion.
float comandoServoUs = PULSO_CENTRO_US;

uint32_t tiempoAnteriorUs = 0;
uint32_t proximaMuestraUs = 0;
uint32_t proximoCambioServoUs = 0;
uint8_t indicePulso = 0;

void sendBinaryFrame() {
  if ((int32_t)(proximaMuestraUs - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 12) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  uint8_t trama[12] = {'a', 'b', 'c', 'd'};
  const float valores[] = {anguloFiltradoX - offsetAnguloX, comandoServoUs};
  for (uint8_t i = 0; i < 2; ++i) {
    uint32_t bits;
    memcpy(&bits, &valores[i], sizeof(bits));
    for (uint8_t j = 0; j < 4; ++j)
      trama[4 + 4 * i + j] = (uint8_t)(bits >> (8 * j));
  }
  Serial.write(trama, sizeof(trama)); // UART en segundo plano, sin flush()
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  // Mantener el actuador en el punto medio durante la calibracion.
  comandoServoUs = PULSO_CENTRO_US;
  miServo.writeMicroseconds((int)comandoServoUs);
  miServo.attach(PIN_SERVO);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (!mpu.begin(MPU_ADDR)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Mantener el sistema quieto durante esta calibracion inicial.
  delay(ESTABILIZACION_SERVO_MS);
  calibrarGyro();
  calibrarCeroAngular();

  // La adquisicion comienza en el centro mecanico.
  comandoServoUs = (float)SECUENCIA_SERVO_US[indicePulso];
  miServo.writeMicroseconds((int)comandoServoUs);

  const uint32_t ahoraUs = micros();
  tiempoAnteriorUs = ahoraUs;
  proximaMuestraUs = ahoraUs + SAMPLE_PERIOD_US;
  proximoCambioServoUs = proximaMuestraUs + INTERVALO_EXCITACION_US;
}

void loop() {
  const uint32_t ahoraUs = micros();
  if ((int32_t)(ahoraUs - proximaMuestraUs) < 0) return;
  proximaMuestraUs += SAMPLE_PERIOD_US;
  while ((int32_t)(ahoraUs - proximaMuestraUs) >= 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    proximaMuestraUs += SAMPLE_PERIOD_US;
  }

  const float dt = (ahoraUs - tiempoAnteriorUs) * 1.0e-6;
  tiempoAnteriorUs = ahoraUs;
  actualizarAngulo(dt);

  // y[k] se mide antes de aplicar u[k], retenida hasta el siguiente ciclo.
  // Los cambios de nivel tambien ocurren sobre esta grilla de 20 ms.
  if ((int32_t)(ahoraUs - proximoCambioServoUs) >= 0) {
    do {
      proximoCambioServoUs += INTERVALO_EXCITACION_US;
      indicePulso = (indicePulso + 1) % CANTIDAD_PULSOS;
    } while ((int32_t)(ahoraUs - proximoCambioServoUs) >= 0);
  }
  comandoServoUs = (float)SECUENCIA_SERVO_US[indicePulso];
  miServo.writeMicroseconds((int)comandoServoUs);
  sendBinaryFrame();
  if ((int32_t)(micros() - proximaMuestraUs) >= 0)
    digitalWrite(LED_BUILTIN, HIGH);
}

// El mismo filtro se usa durante la calibracion y durante la adquisicion.
void actualizarAngulo(float dt) {
  sensors_event_t accel, gyro, temperatura;
  mpu.getEvent(&accel, &gyro, &temperatura);
  anguloAccX =
      atan2(accel.acceleration.y,
            sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) *
      180.0 / PI;
  const float gyroX_dps =
      (gyro.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
  anguloGyroX += gyroX_dps * dt;
  anguloFiltradoX =
      ALPHA * (anguloFiltradoX + gyroX_dps * dt) +
      (1.0 - ALPHA) * anguloAccX;
}

void calibrarCeroAngular() {
  // No hay excitacion ni tramas durante esta etapa; el servo sigue en 1544 us.
  actualizarAngulo(0.0f);
  anguloFiltradoX = anguloAccX;
  uint32_t anteriorUs = micros();
  uint32_t siguienteUs = anteriorUs + SAMPLE_PERIOD_US;
  float sumaAngulo = 0.0f;
  const uint16_t total = MUESTRAS_ASENTAMIENTO_FILTRO + MUESTRAS_OFFSET_ANGULO;
  for (uint16_t i = 0; i < total; ++i) {
    while ((int32_t)(micros() - siguienteUs) < 0) {}
    const uint32_t ahoraUs = micros();
    actualizarAngulo((ahoraUs - anteriorUs) * 1.0e-6f);
    anteriorUs = ahoraUs;
    siguienteUs += SAMPLE_PERIOD_US;
    if (i >= MUESTRAS_ASENTAMIENTO_FILTRO)
      sumaAngulo += anguloFiltradoX;
  }
  offsetAnguloX = sumaAngulo / MUESTRAS_OFFSET_ANGULO;
}

void calibrarGyro() {
  float sumaXRadS = 0.0;
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    sensors_event_t accel, gyro, temperatura;
    mpu.getEvent(&accel, &gyro, &temperatura);
    sumaXRadS += gyro.gyro.x;
    delay(3);
  }

  gyroX_offsetRadS = sumaXRadS / muestras;
}
