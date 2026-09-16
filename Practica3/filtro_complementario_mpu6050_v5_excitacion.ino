/* Filtro complementario con MPU6050 y excitacion periodica del servo.
 *
 * Secuencia de entrada (en microsegundos):
 *   1100 -> 900 -> 700 -> 900 -> 1100 -> 900 -> ...
 *
 * Trama binaria para MATLAB (20 bytes):
 *   "abcd" + anguloFiltradoX + anguloGyroX + anguloAccX + pulsoServoUs
 *
 * Los cuatro datos son float de 32 bits. No se imprime texto por Serial.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Servo miServo;

const uint8_t PIN_SERVO = 9;
const int PULSO_MIN_ATTACH_US = 500;
const int PULSO_MAX_ATTACH_US = 2500;
const uint32_t INTERVALO_EXCITACION_MS = 1500;

// Al repetirse, genera: 1100, 900, 700, 900, 1100, 900, ...
const uint16_t SECUENCIA_SERVO_US[] = {1100, 900, 700, 900};
const uint8_t CANTIDAD_PULSOS =
    sizeof(SECUENCIA_SERVO_US) / sizeof(SECUENCIA_SERVO_US[0]);

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 10000; // Filtro a 100 Hz
const uint8_t TX_DECIMATION = 2;         // Envio a MATLAB a 50 Hz
const float ALPHA = 0.96;

float gyroX_offsetRadS = 0.0;
float anguloAccX = 0.0;
float anguloGyroX = 0.0;
float anguloFiltradoX = 0.0;
float pulsoServoUs = 1100.0;

uint32_t tiempoAnteriorUs = 0;
uint32_t proximaMuestraUs = 0;
uint32_t ultimoCambioServoMs = 0;
uint8_t indicePulso = 0;
uint8_t contadorTx = 0;

void sendFloat(float value) {
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  Serial.write(bytes, sizeof(value));
}

void sendBinaryFrame() {
  Serial.write("abcd", 4);
  sendFloat(anguloFiltradoX);
  sendFloat(anguloGyroX);
  sendFloat(anguloAccX);
  sendFloat(pulsoServoUs); // Entrada aplicada a la planta, en us
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  // Limites explicitos para evitar el recorte interno de Servo.
  miServo.attach(PIN_SERVO, PULSO_MIN_ATTACH_US, PULSO_MAX_ATTACH_US);
  // Mantener el actuador en el punto medio durante la calibracion.
  pulsoServoUs = 900.0;
  miServo.writeMicroseconds((int)pulsoServoUs);

  pinMode(LED_BUILTIN, OUTPUT);
  if (!mpu.begin(MPU_ADDR)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Mantener el sistema quieto durante esta calibracion inicial.
  calibrarGyro();

  // La adquisicion comienza junto con la primera entrada de 1100 us.
  pulsoServoUs = (float)SECUENCIA_SERVO_US[indicePulso];
  miServo.writeMicroseconds((int)pulsoServoUs);

  const uint32_t ahoraUs = micros();
  tiempoAnteriorUs = ahoraUs;
  proximaMuestraUs = ahoraUs + SAMPLE_PERIOD_US;
  ultimoCambioServoMs = millis();
}

void loop() {
  // Cambiar la entrada sin bloquear el muestreo de la IMU.
  const uint32_t ahoraMs = millis();
  if ((uint32_t)(ahoraMs - ultimoCambioServoMs) >= INTERVALO_EXCITACION_MS) {
    ultimoCambioServoMs += INTERVALO_EXCITACION_MS;
    indicePulso = (indicePulso + 1) % CANTIDAD_PULSOS;
    pulsoServoUs = (float)SECUENCIA_SERVO_US[indicePulso];
    miServo.writeMicroseconds((int)pulsoServoUs);
  }

  const uint32_t ahoraUs = micros();
  if ((int32_t)(ahoraUs - proximaMuestraUs) < 0) return;
  proximaMuestraUs += SAMPLE_PERIOD_US;
  while ((int32_t)(ahoraUs - proximaMuestraUs) >= 0) {
    proximaMuestraUs += SAMPLE_PERIOD_US;
  }

  sensors_event_t accel, gyro, temperatura;
  mpu.getEvent(&accel, &gyro, &temperatura);

  const float dt = (ahoraUs - tiempoAnteriorUs) * 1.0e-6;
  tiempoAnteriorUs = ahoraUs;

  anguloAccX =
      atan2(accel.acceleration.y,
            sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) *
      180.0 / PI;

  // Adafruit entrega rad/s; el filtro integra grados/s.
  const float gyroX_dps =
      (gyro.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
  anguloGyroX += gyroX_dps * dt;
  anguloFiltradoX =
      ALPHA * (anguloFiltradoX + gyroX_dps * dt) +
      (1.0 - ALPHA) * anguloAccX;

  if (++contadorTx >= TX_DECIMATION) {
    contadorTx = 0;
    sendBinaryFrame();
  }
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
