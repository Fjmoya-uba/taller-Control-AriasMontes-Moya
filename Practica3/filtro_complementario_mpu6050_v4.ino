/* Filtro complementario con MPU6050 y comunicacion binaria con MATLAB.
 * Trama: "abcd" + anguloFiltradoX + anguloGyroX + anguloAccX.
 * Son tres float en grados: 16 bytes por trama en total.
 * El servo se controla mediante ancho de pulso en microsegundos.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Servo miServo;
const uint8_t PIN_SERVO = 9;
const int PULSO_INICIO_US = 500;
const int PULSO_FINAL_US = 1600;
const uint32_t INTERVALO_SERVO_MS = 1500;

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 10000; // 100 Hz
const uint8_t TX_DECIMATION = 2;         // 50 tramas/s
const float ALPHA = 0.96;

float gyroX_offsetRadS = 0.0;
float anguloAccX = 0.0;
float anguloGyroX = 0.0;
float anguloFiltradoX = 0.0;
uint32_t tiempoAnteriorUs = 0;
uint32_t proximaMuestraUs = 0;
uint32_t ultimoCambioServoMs = 0;
uint8_t contadorTx = 0;
bool servoEnPulsoFinal = false;

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

  // Limites explicitos para que Servo no recorte los 500 us a 544 us.
  miServo.attach(PIN_SERVO, PULSO_INICIO_US, 2500);
  miServo.writeMicroseconds(PULSO_INICIO_US);

  pinMode(LED_BUILTIN, OUTPUT);
  if (!mpu.begin(MPU_ADDR)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }

  // Misma configuracion que lectura_imu.ino.
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // No imprimir texto: MATLAB espera un flujo exclusivamente binario.
  calibrarGyro();

  const uint32_t ahoraUs = micros();
  tiempoAnteriorUs = ahoraUs;
  proximaMuestraUs = ahoraUs + SAMPLE_PERIOD_US;
  // Fuerza el primer cambio al pulso final apenas comienza loop().
  ultimoCambioServoMs = millis() - INTERVALO_SERVO_MS;
}

void loop() {
  // Alternar los pulsos sin bloquear el muestreo ni la comunicacion serie.
  const uint32_t ahoraMs = millis();
  if ((uint32_t)(ahoraMs - ultimoCambioServoMs) >= INTERVALO_SERVO_MS) {
    ultimoCambioServoMs += INTERVALO_SERVO_MS;
    servoEnPulsoFinal = !servoEnPulsoFinal;
    miServo.writeMicroseconds(servoEnPulsoFinal
                               ? PULSO_FINAL_US
                               : PULSO_INICIO_US);
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

  anguloAccX = atan2(accel.acceleration.y, sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) * 180.0 / PI; 
  // Adafruit entrega rad/s; el filtro integra grados/s.
  const float gyroX_dps = (gyro.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
  anguloGyroX += gyroX_dps * dt;
  anguloFiltradoX = ALPHA * (anguloFiltradoX + gyroX_dps * dt) + (1.0 - ALPHA) * anguloAccX;

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
