/* Practica8: control de angulo con observador Tustin e integrador a 50 Hz.
 * MPU6050 y filtro/calibracion conservados de Practica7. Servo en pin 9.
 * Configuracion: K_THETA, K_OMEGA, H_INTEGRAL, REFERENCIA_GRADOS.
 * Retardo asumido de 40 ms conservado del observador previo; validar en planta.
 * Control y telemetria se programan cada 20000 us. Si no se cumple el plazo,
 * se enclava el fallo y vuelve al centro; reiniciar para rearmar.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <math.h>
#include "coeficientes_observador_tustin.h"

Adafruit_MPU6050 mpu;
Servo miServo;

const uint8_t PIN_SERVO = 9;
// Esta orden define la posicion de referencia de 0 grados.
const int PULSO_CENTRO_US = 1544;
const uint32_t ESTABILIZACION_SERVO_MS = 2000;
const uint16_t MUESTRAS_ASENTAMIENTO_FILTRO = 50; // 1 s a 50 Hz
const uint16_t MUESTRAS_OFFSET_ANGULO = 150; // Promediar durante 3 s

// Editar estas ganancias con los resultados de controlador_variable_estados.m.
// u[us] = -K_THETA*thetaEst[rad] - K_OMEGA*omegaEst[rad/s] + H_INTEGRAL*q[rad*s].
const float K_THETA = -260.6476f;
const float K_OMEGA = 6.0377f;
const float H_INTEGRAL = 3.7347e+03f;
const float REFERENCIA_GRADOS = 5.0f; // Relativa al cero calibrado
const uint32_t ESPERA_REFERENCIA_US = 1000000UL;
const int PULSO_MIN_US = 1100; // Confirmar limites mecanicos del equipo
const int PULSO_MAX_US = 2000;
static_assert(PULSO_MIN_US < PULSO_CENTRO_US && PULSO_CENTRO_US < PULSO_MAX_US,
              "Centro fuera de limites");
float integralError = 0.0f;
uint32_t inicioControlUs = 0;
uint32_t numeroMuestra = 0;
uint32_t fallos = 0; // bit 0: plazo incumplido; bit 1: trama omitida; bit 2: no finito
bool detenido = false;
float referenciaRad = 0.0f;

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 20000UL; // Ciclo completo a 50 Hz
const uint32_t RESERVA_TX_US = 5000UL; // Margen conservador; 20 bytes a 115200: ~1736 us
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(SAMPLE_PERIOD_US == 20000UL,
              "Los coeficientes Tustin requieren Ts de 20 ms");
// Coeficientes del filtro complementario Tustin incluidos en el encabezado.

float thetaEst = 0.0f; // estado fisico, rad
float yAnterior = 0.0f;
float uTresMuestrasAntes = 0.0f;
float accAnterior = 0.0f; // grados
float gyroAnterior = 0.0f; // grados/s
bool filtroInicializado = false;
float omegaEst = 0.0f; // rad/s
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

void sendBinaryFrame(float thetaMedida) {
  if ((int32_t)(proximaMuestraUs - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 20) {
    fallos |= 2;
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  // Protocolo Practica7: abcd + cuatro float32 little endian (20 bytes).
  // Orden: omega estimada [rad/s], omega medida [rad/s], theta estimada [grados], theta complementaria [grados].
  uint8_t trama[20] = {'a', 'b', 'c', 'd'};
  const float valores[] = {omegaEst, omegaGyro, thetaEst * 180.0f / PI, anguloFiltradoX - offsetAnguloX};
  for (uint8_t i = 0; i < 4; ++i) {
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

  // Estado inicial y prehistoria relativos al centro calibrado: cero.
  comandoServoUs = PULSO_CENTRO_US;
  miServo.writeMicroseconds(PULSO_CENTRO_US);
  inicioControlUs = micros();
  tiempoAnteriorUs = inicioControlUs;
  proximaMuestraUs = inicioControlUs + SAMPLE_PERIOD_US;
}

void loop() {
  const uint32_t ahoraUs = micros();
  if ((int32_t)(ahoraUs-proximaMuestraUs) < 0) return;
  proximaMuestraUs += SAMPLE_PERIOD_US;
  if ((int32_t)(ahoraUs-proximaMuestraUs) >= 0) {
    // No simular muestras perdidas ejecutando ciclos consecutivos.
    fallos |= 1;
    detenido = true;
    proximaMuestraUs = ahoraUs + SAMPLE_PERIOD_US;
  }
  actualizarAngulo(0.02f);
  const float y = (anguloFiltradoX-offsetAnguloX)*PI/180.0f;
  referenciaRad = (uint32_t)(ahoraUs-inicioControlUs) >= ESPERA_REFERENCIA_US
      ? REFERENCIA_GRADOS*PI/180.0f : 0.0f;

  // Observador original Tustin, con el retardo de dos muestras de Practica7.
  const float sumaU = uDosMuestrasAntes+uTresMuestrasAntes;
  const float sumaY = y+yAnterior;
  const float thetaNueva = OBS_A11*thetaEst+OBS_A12*omegaEst+OBS_BU1*sumaU+OBS_BY1*sumaY;
  const float omegaNueva = OBS_A21*thetaEst+OBS_A22*omegaEst+OBS_BU2*sumaU+OBS_BY2*sumaY;
  thetaEst = thetaNueva;
  omegaEst = omegaNueva;

  const float error = referenciaRad-y;
  const float uPedido = -K_THETA*thetaEst-K_OMEGA*omegaEst+H_INTEGRAL*integralError;
  if (!isfinite(uPedido) || !isfinite(y) || !isfinite(omegaEst)) {
    fallos |= 4;
    detenido = true;
  }
  const float uMin = PULSO_MIN_US-PULSO_CENTRO_US;
  const float uMax = PULSO_MAX_US-PULSO_CENTRO_US;
  float u = 0.0f;
  if (!detenido) {
    u = constrain(uPedido,uMin,uMax);
    // Integracion Euler a Ts fijo; congelar si agrava la saturacion.
    const bool agrava = (uPedido>=uMax && H_INTEGRAL*error>0) ||
                        (uPedido<=uMin && H_INTEGRAL*error<0);
    if (!agrava) integralError += 0.02f*error;
  } else {
    integralError = 0;
  }
  comandoServoUs = (float)lroundf(PULSO_CENTRO_US+u);
  miServo.writeMicroseconds((int)comandoServoUs);
  u = comandoServoUs-PULSO_CENTRO_US;

  yAnterior = y;
  uTresMuestrasAntes = uDosMuestrasAntes;
  uDosMuestrasAntes = uAnterior;
  uAnterior = u;
  if ((int32_t)(micros()-proximaMuestraUs) >= 0) {
    fallos |= 1;
    detenido = true;
    comandoServoUs = PULSO_CENTRO_US;
    miServo.writeMicroseconds(PULSO_CENTRO_US);
    uAnterior = 0;
  }
  if (fallos) digitalWrite(LED_BUILTIN,HIGH);
  sendBinaryFrame(y);
  ++numeroMuestra;
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


