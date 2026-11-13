/* Observador [angulo, velocidad, sesgo] en grados y grados/s.
 * Base: validar_observador_tustin. Conserva calibracion, filtro y retardo 40 ms.
 * Ejecutar exportar_sesgo_tustin.m antes de compilar.
 * Sesgo artificial SOLO en el canal de velocidad del observador.
 * TX: abcd + 6 float32: omega_est, omega_medida_con_sesgo,
 * theta_est, theta_filtro, sesgo_aplicado, sesgo_estimado.
 * Unidades TX: grados y grados/s. Periodo: 20 ms. Baud: 115200.
 * LED encendido: ensayo invalido por plazo perdido o trama omitida.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include "coeficientes_sesgo_tustin.h"

Adafruit_MPU6050 mpu;
Servo miServo;

const uint8_t PIN_SERVO = 9;
// Esta orden define la posicion de referencia de 0 grados.
const int PULSO_CENTRO_US = 1544;
const uint32_t ESTABILIZACION_SERVO_MS = 2000;
const uint16_t MUESTRAS_ASENTAMIENTO_FILTRO = 50; // 1 s a 50 Hz
const uint16_t MUESTRAS_OFFSET_ANGULO = 150; // Promediar durante 3 s
const uint32_t INTERVALO_EXCITACION_US = 2000000UL;

const uint16_t SECUENCIA_SERVO_US[] = {PULSO_CENTRO_US, 1800, PULSO_CENTRO_US, 1300};
const uint8_t CANTIDAD_PULSOS =
    sizeof(SECUENCIA_SERVO_US) / sizeof(SECUENCIA_SERVO_US[0]);

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 20000UL; // Ciclo completo a 50 Hz
const uint32_t RESERVA_TX_US = 3500UL; // 28 bytes a 115200: ~2431 us
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(INTERVALO_EXCITACION_US % SAMPLE_PERIOD_US == 0,
              "Cada escalon debe durar un numero entero de periodos");
// Coeficientes del filtro complementario Tustin incluidos en el encabezado.

const float SESGO_ARTIFICIAL_DPS = 10.0f;
float sesgoEst = 0.0f; // grados/s
float velocidadAnterior = 0.0f; // medicion anterior con sesgo
float velocidadMedida = 0.0f; // grados/s, con sesgo artificial
float thetaEst = 0.0f; // grados
float yAnterior = 0.0f;
float uTresMuestrasAntes = 0.0f;
float accAnterior = 0.0f; // grados
float gyroAnterior = 0.0f; // grados/s
bool filtroInicializado = false;
float omegaEst = 0.0f; // grados/s
float omegaGyro = 0.0f; // rad/s, corregida por bias
// Historial de entrada relativa al centro: u[-1] = u[-2] = 0 us.
float uAnterior = 0.0f; // u[k-1]
float uDosMuestrasAntes = 0.0f; // u[k-2], retardo de 40 ms
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
      Serial.availableForWrite() < 28) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  uint8_t trama[28] = {'a', 'b', 'c', 'd'};
  const float valores[] = {omegaEst, velocidadMedida, thetaEst,
      anguloFiltradoX - offsetAnguloX, SESGO_ARTIFICIAL_DPS, sesgoEst};
  for (uint8_t i = 0; i < 6; ++i) {
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

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
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
  const float y = anguloFiltradoX - offsetAnguloX; // grados
  // El filtro angular conserva el gyro calibrado original, sin sesgo artificial.
  // Esta nueva medicion SI lleva +10; el observador no conoce ese valor.
  velocidadMedida = omegaGyro * 180.0f / PI + SESGO_ARTIFICIAL_DPS;
  const float u = comandoServoUs - PULSO_CENTRO_US;
  // Integrar el intervalo anterior usando los extremos de las senales.
  const float sumaU = uDosMuestrasAntes + uTresMuestrasAntes;
  const float sumaY = y + yAnterior;
  const float sumaVelocidad = velocidadMedida + velocidadAnterior;
  const float estadoAnterior[3] = {thetaEst, omegaEst, sesgoEst};
  float siguiente[3];
  // Todas las filas usan el mismo estado anterior (actualizacion simultanea).
  for (uint8_t i = 0; i < 3; ++i) {
    siguiente[i] = OBS_BU[i]*sumaU + OBS_BY[i][0]*sumaY
                 + OBS_BY[i][1]*sumaVelocidad;
    for (uint8_t j = 0; j < 3; ++j)
      siguiente[i] += OBS_AD[i][j]*estadoAnterior[j];
  }
  thetaEst = siguiente[0];
  omegaEst = siguiente[1];
  sesgoEst = siguiente[2];
  velocidadAnterior = velocidadMedida;
  sendBinaryFrame(); // estados fisicos y sensores del instante actual
  yAnterior = y;
  uTresMuestrasAntes = uDosMuestrasAntes;
  uDosMuestrasAntes = uAnterior;
  uAnterior = u;
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
  omegaGyro = gyro.gyro.x - gyroX_offsetRadS;
  const float gyroX_dps =
      (gyro.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
  if (!filtroInicializado) {
    accAnterior = anguloAccX;
    gyroAnterior = gyroX_dps;
    anguloFiltradoX = anguloAccX;
    filtroInicializado = true;
    return;
  }
  // Tustin con Ts fijo de 20 ms; dt solo se conserva para diagnostico.
  (void)dt;
  anguloGyroX += 0.01f * (gyroX_dps + gyroAnterior);
  anguloFiltradoX = FILTRO_A*anguloFiltradoX +
      FILTRO_ACC*(anguloAccX + accAnterior) +
      FILTRO_GYRO*(gyroX_dps + gyroAnterior);
  accAnterior = anguloAccX;
  gyroAnterior = gyroX_dps;
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


