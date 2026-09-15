/* Filtro complementario con MPU6050 y comunicacion binaria con MATLAB.
 * Trama: "abcd" + anguloFiltradoX + anguloGyroX + anguloAccX.
 * Son tres float en grados: 16 bytes por trama en total.
 */
#include <Wire.h>
#include <Servo.h>

Servo miServo;
const uint8_t PIN_SERVO = 9;
const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 10000; // 100 Hz
const uint8_t TX_DECIMATION = 2;         // 50 tramas/s
const float ALPHA = 0.96;
const float SENS_ACC = 16384.0;          // LSB/g, rango +/-2 g
const float SENS_GYRO = 131.0;           // LSB/(grados/s), rango +/-250

int16_t accX_raw, accY_raw, accZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
float gyroX_offset = 0.0;
float anguloAccX = 0.0;
float anguloGyroX = 0.0;
float anguloFiltradoX = 0.0;
uint32_t tiempoAnteriorUs = 0;
uint32_t proximaMuestraUs = 0;
uint32_t ultimoCambioServoMs = 0;
uint8_t contadorTx = 0;
bool servoArriba = true;

void sendFloat(float value) {
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  Serial.write(bytes, sizeof(value));
}

void sendBinaryFrame() {
  Serial.write("abcd", 4);
  sendFloat(anguloFiltradoX);
  sendFloat(anguloGyroX);
  sendFloat(anguloAccX);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  miServo.attach(PIN_SERVO);
  miServo.write(20);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Despertar sensor
  Wire.endTransmission(true);

  // No imprimir texto: MATLAB espera un flujo exclusivamente binario.
  calibrarGyro();

  const uint32_t ahoraUs = micros();
  tiempoAnteriorUs = ahoraUs;
  proximaMuestraUs = ahoraUs + SAMPLE_PERIOD_US;
  ultimoCambioServoMs = millis();
}

void loop() {
  // Alternar el servo sin bloquear el muestreo de la IMU.
  const uint32_t ahoraMs = millis();
  if ((uint32_t)(ahoraMs - ultimoCambioServoMs) >= 2000) {
    ultimoCambioServoMs += 2000;
    servoArriba = !servoArriba;
    miServo.write(servoArriba ? 20 : 0);
  }

  const uint32_t ahoraUs = micros();
  if ((int32_t)(ahoraUs - proximaMuestraUs) < 0) return;
  proximaMuestraUs += SAMPLE_PERIOD_US;
  while ((int32_t)(ahoraUs - proximaMuestraUs) >= 0) {
    proximaMuestraUs += SAMPLE_PERIOD_US;
  }

  leerSensores();
  const float dt = (ahoraUs - tiempoAnteriorUs) * 1.0e-6;
  tiempoAnteriorUs = ahoraUs;

  anguloAccX = atan2((float)accY_raw, sqrt(sq((float)accX_raw) + sq((float)accZ_raw)))* 180.0 / PI;
  const float gyroX_dps = (gyroX_raw / SENS_GYRO) - gyroX_offset;
  anguloGyroX += gyroX_dps * dt;
  anguloFiltradoX = ALPHA * (anguloFiltradoX + gyroX_dps * dt) + (1.0 - ALPHA) * anguloAccX;

  if (++contadorTx >= TX_DECIMATION) {
    contadorTx = 0;
    sendBinaryFrame();
  }
}

void leerSensores() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return;

  accX_raw = (int16_t)((Wire.read() << 8) | Wire.read());
  accY_raw = (int16_t)((Wire.read() << 8) | Wire.read());
  accZ_raw = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read(); // Temperatura
  gyroX_raw = (int16_t)((Wire.read() << 8) | Wire.read());
  gyroY_raw = (int16_t)((Wire.read() << 8) | Wire.read());
  gyroZ_raw = (int16_t)((Wire.read() << 8) | Wire.read());
}

void calibrarGyro() {
  int32_t sumaX = 0;
  const int muestras = 500;
  for (int i = 0; i < muestras; i++) {
    leerSensores();
    sumaX += gyroX_raw;
    delay(3);
  }
  gyroX_offset = (sumaX / (float)muestras) / SENS_GYRO;
}
