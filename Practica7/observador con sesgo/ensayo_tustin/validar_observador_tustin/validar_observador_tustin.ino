/* Validacion real del observador de estimar_theta_punto.m.
 * Ejecutar exportar_observador_tustin en MATLAB antes de compilar este sketch.
 * Servo pin 9: 1544 -> 1800 -> 1544 -> 1300, 2 s por nivel.
 * MPU6050: mismo eje X, calibracion y filtro complementario de Practica3.
 * u[k] = pulso - 1544 us; y[k] = angulo relativo al cero calibrado.
 * Se rebasa el equilibrio del modelo al centro del servo (modelo lineal).
 * Estados iniciales del observador: cero; no usa el giro como correccion.
 * Trama: 'abcd' + 6 float32 little endian (28 bytes):
 * ángulos (grados), velocidades angulares (grados/s) y sesgos (grados/s).
 * Sin texto por Serial. LED encendido: plazo perdido o trama omitida;
 * descartar el ensayo, porque el observador supone Ts fijo.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <BasicLinearAlgebra.h>
#include "coeficientes_observador_tustin.h"

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
const uint32_t RESERVA_TX_US = 3500UL; // 20 bytes a 115200: ~1736 us
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(INTERVALO_EXCITACION_US % SAMPLE_PERIOD_US == 0,
              "Cada escalon debe durar un numero entero de periodos");
// Coeficientes del filtro complementario Tustin incluidos en el encabezado.

float yAnterior = 0.0f;
float accAnterior = 0.0f; // grados
float gyroAnterior = 0.0f; // grados/s
bool filtroInicializado = false;
float gyroX_dps = 0.0f;
// Historial de entrada relativa al centro: u[-1] = u[-2] = 0 us.
float uAnterior = 0.0f; // u[k-1]
float uDosMuestrasAntes = 0.0f; // u[k-2], retardo de 40 ms
float uTresMuestrasAntes = 0.0f;
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

using namespace BLA;

Matrix<3, 3> Ad = {OBS_A11, OBS_A12, OBS_A13,
                  OBS_A21, OBS_A22, OBS_A23,
                   OBS_A31, OBS_A32, OBS_A33};

Matrix<3, 1> Bd = {OBS_BU1,
                   OBS_BU2,
                   OBS_BU3};

Matrix<2, 3> Cd = {1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 1.0f};

Matrix<3, 2> L = {OBS_BYACC1, OBS_BYGYRO1,
                  OBS_BYACC2, OBS_BYGYRO2,
                  OBS_BYACC3, OBS_BYGYRO3};

Matrix<3, 1> xk = {0, 0, 0};
Matrix<3, 1> xk_ant = {0, 0, 0};

Matrix<2, 1> yk = {0, 0};
Matrix<2, 1> yk_ant = {0, 0};
Matrix<2, 1> suma_yk = {0, 0};

void sendBinaryFrame() {
  if ((int32_t)(proximaMuestraUs - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 28) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  uint8_t trama[28] = {'a', 'b', 'c', 'd'};
  const float valores[] = {yk(0)*180/PI, xk(0)*180/PI, yk(1)*180/PI, xk(1)*180/PI, 10, xk(2)*180/PI};
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
  yk(0) = (anguloFiltradoX - offsetAnguloX) * PI / 180.0f;
  yk(1) = (gyroX_dps + 10) * PI / 180.0f;
  const float u = comandoServoUs - PULSO_CENTRO_US;

  // Integrar el intervalo anterior usando los extremos de las señales.
  const float sumaU = uDosMuestrasAntes + uTresMuestrasAntes;
  suma_yk = yk + yk_ant;

  // Tustin fisico: xhat[k] = Ad*xhat[k-1] + Bu*(u[k-2]+u[k-3]) + By*(y[k]+y[k-1]).  
  // xhat = [theta; omega; b]; y_acc [rad], y_gyro [rad/s] crudo, u [us]
  xk = Ad * xk_ant + Bd * sumaU + L*(suma_yk);
  xk_ant = xk;
  sendBinaryFrame(); // estados fisicos y sensores del instante actual
  yk_ant = yk;
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
  gyroX_dps = (gyro.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
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


