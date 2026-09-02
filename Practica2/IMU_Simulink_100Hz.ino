/*
 * Adquisicion de MPU6050 y transmision hacia Simulink/PC
 *
 * Trama binaria enviada (28 bytes en total):
 *   "abcd" + ax + ay + az + gx + gy + gz
 *
 * Los primeros cuatro bytes son una cabecera que permite encontrar el inicio
 * de cada muestra. Cada medicion es un float de 4 bytes, igual que en el
 * ejemplo Serial_simulink.ino:
 *
 *   bytes  0..3  : caracteres 'a', 'b', 'c', 'd'
 *   bytes  4..7  : ax
 *   bytes  8..11 : ay
 *   bytes 12..15 : az
 *   bytes 16..19 : gx
 *   bytes 20..23 : gy
 *   bytes 24..27 : gz
 *
 * Unidades entregadas por la libreria Adafruit:
 *   aceleracion: m/s^2
 *   velocidad angular: rad/s
 *
 * Recepcion sugerida en PC/Simulink:
 *   - Puerto serie: 115200 bit/s, 8-N-1, sin terminador de texto.
 *   - Leer uint8 y buscar la secuencia de cabecera "abcd".
 *   - Tomar los siguientes 24 bytes y reconstruir seis valores single.
 *   - Guardar las muestras con To Workspace/To File, o guardar primero los
 *     bytes recibidos en un archivo para procesarlos luego desde MATLAB.
 *
 * La IMU siempre se adquiere a 100 Hz. TX_DECIMATION solo modifica la tasa
 * enviada: 1 = 100 Hz, 2 = 50 Hz, 5 = 20 Hz, etc. Esto permite reducir la
 * carga del puerto serie sin cambiar la base de tiempo del sensor.
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 400000;  // I2C rapido (Fast Mode)
constexpr uint32_t SAMPLE_PERIOD_US = 10000;  // 10 ms = 100 Hz
constexpr uint8_t TX_DECIMATION = 2;          // Cambiar si aparece lag en PC

uint32_t nextSampleUs = 0;
uint8_t decimationCounter = 0;

void sendFloat(float value) {
  // Envia directamente los cuatro bytes que forman el float.
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  Serial.write(bytes, sizeof(value));
}

void sendBinaryFrame(const sensors_event_t &accel,
                     const sensors_event_t &gyro) {
  // La cabecera permite resincronizar la recepcion si se pierden bytes.
  Serial.write("abcd", 4);

  sendFloat(accel.acceleration.x);
  sendFloat(accel.acceleration.y);
  sendFloat(accel.acceleration.z);
  sendFloat(gyro.gyro.x);
  sendFloat(gyro.gyro.y);
  sendFloat(gyro.gyro.z);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  // No se envian mensajes de error por Serial porque romperian la trama
  // binaria. Si la IMU no responde, el LED incorporado queda encendido.
  pinMode(LED_BUILTIN, OUTPUT);
  if (!mpu.begin()) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) {
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // La primera adquisicion se programa un periodo despues de inicializar.
  nextSampleUs = micros() + SAMPLE_PERIOD_US;
}

void loop() {
  const uint32_t nowUs = micros();

  // La resta con signo hace que la comparacion siga funcionando cuando
  // micros() desborda (aproximadamente cada 71 minutos en micros de 32 bits).
  if ((int32_t)(nowUs - nextSampleUs) < 0) {
    return;
  }

  // Se suma el periodo al instante programado, no al instante actual. Asi se
  // evita acumular deriva temporal por el tiempo que tardan lectura y envio.
  nextSampleUs += SAMPLE_PERIOD_US;

  // Si una operacion excepcionalmente larga hizo perder periodos, se saltean
  // esas ranuras: no se generan rafagas de muestras atrasadas.
  while ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;
  }

  sensors_event_t accel, gyro, temperature;
  mpu.getEvent(&accel, &gyro, &temperature);

  decimationCounter++;
  if (decimationCounter >= TX_DECIMATION) {
    decimationCounter = 0;
    sendBinaryFrame(accel, gyro);
  }
}
