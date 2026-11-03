/* Script para validar observador de estados que incluye el sesgo del giróscopo.
 * Los coeficientes de las matrices Ad, Bd, Cd y L salen del script de matlab
 * 'observador_sesgo_gyro.m'. En caso de hacer alguna modificación en las matrices
 * cambiar los coeficientes de Ad, Bd, Cd y L definidas más abajo.
 * De este script se envían 6 variables a matlab: el ángulo de la barra medido con
 * la IMU y el estimado por el observador (en grados), la velocidad angular medida 
 * por el giróscopo y la estimada (grados/s), y el sesgo del giróscopo, estimado 
 * como el promedio de 500 muestras con la barra quieta, junto con la estimación 
 * del observador (grados/s).
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <BasicLinearAlgebra.h>

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
const float ALPHA = 0.94f;

static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(INTERVALO_EXCITACION_US % SAMPLE_PERIOD_US == 0,
              "Cada escalon debe durar un numero entero de periodos");


float gyroX_offsetRadS = 0.0;
float anguloAccX = 0.0;
float anguloGyroX = 0.0;
float anguloFiltradoX = 0.0;
float offsetAnguloX = 0.0;
// Entrada u: orden enviada al servo; no es una medicion de pulsos ni de posicion.
float comandoServoUs = PULSO_CENTRO_US;
float u = 0; // para el modelo en espacio de estados u debe ser un ángulo

uint32_t tiempoAnteriorUs = 0;
uint32_t proximaMuestraUs = 0;
uint32_t proximoCambioServoUs = 0;
uint8_t indicePulso = 0;

// defino las matrices del modelo en espacio de estados
using namespace BLA;

/*Matrix<3, 3> Ad = {1.0f, 0.02, 0.0f,
                   -19.35, -0.0024, 0.0f,
                   0.0f, 0.0f, 1.0f};*/

Matrix<3, 3> Ad = {0.8619, 0.0118, 0.0f,
                  -11.4588, 0.2683, 0.0f,
                   0.0f, 0.0f, 1.0f};


/*Matrix<3, 1> Bd = {0.0f,
                   0.7364,
                   0.0f};*/

Matrix<3, 1> Bd = {0.0053,
                   0.4361,
                   0.0f};

Matrix<2, 3> Cd = {1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 1.0f};


// polos 3 veces + rápidos
/*Matrix<3, 2> L = {0.0961, -0.0140,
                  -19.3661, -0.0501,
                  -0.0319, 0.8473};

// polos 4 veces + rápidos
Matrix<3, 2> L = {1.0343, -0.2351,
                  -19.2764, -0.0690,
                  -0.0461, 0.1015};*/

/*Matrix<3, 2> L = {0.8734, 0.0781,
                  -11.7452, 0.1869,
                  0.1355, 0.1391};*/

Matrix<3, 2> L = {-0.0424, -0.0102,
                  -11.4664, -0.0833,
                  -0.0246, 0.8076};

Matrix<3, 1> xk = {0, 0, 0};
Matrix<3, 1> xk_ant = {0, 0, 0};

Matrix<2, 1> yk = {0, 0};
Matrix<2, 1> yk_ant = {0, 0};

void sendBinaryFrame() {
  if ((int32_t)(proximaMuestraUs - micros()) < (int32_t)RESERVA_TX_US ||
      Serial.availableForWrite() < 28) {
    return;
  }
  uint8_t trama[28] = {'a', 'b', 'c', 'd'};
  //const float valores[] = {yk(0), xk(0), yk(1), xk(1), gyroX_offsetRadS*(180.0/PI), xk(2)};
  const float valores[] = {yk(0), xk(0), yk(1), xk(1), 10, xk(2)};
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

  if (!mpu.begin(MPU_ADDR)) {
    Serial.println("No se pudo encontrar a la MPU6050");
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
  
  sensors_event_t accel, gyro, temperatura;
  mpu.getEvent(&accel, &gyro, &temperatura);

  /*anguloAccX =
      atan2(accel.acceleration.y,
            sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) *
      180.0 / PI;*/

  const float velAngularGyro = gyro.gyro.x * (180.0/PI) + 10;   // agrego un sesgo artificial de 10°/s
  anguloAccX = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI - offsetAnguloX;
  //anguloGyroX = anguloFiltradoX + (velAngularGyro - gyroX_offsetRadS * (180.0/PI))*dt;
  anguloGyroX = anguloFiltradoX + (velAngularGyro - 10)*dt;

  anguloFiltradoX = ALPHA * anguloGyroX + (1-ALPHA) * anguloAccX;

  yk_ant = yk;
  //yk(0) = anguloFiltradoX - offsetAnguloX;
  yk(0) = anguloFiltradoX;
  yk(1) = velAngularGyro;

  xk_ant = xk;
  xk = Ad * xk_ant + Bd * u + L * (yk_ant - Cd * xk_ant);

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
  
  //u = mapear_us_a_angulo(comandoServoUs);
  u = comandoServoUs - PULSO_CENTRO_US;

  sendBinaryFrame();

  if ((int32_t)(micros() - proximaMuestraUs) >= 0){}
}

// función auxiliar que convierte una acción de control expresada en us a su equivalente angular en grados
float mapear_us_a_angulo(float u) {
  return 180.0 / 1856 * (u - PULSO_CENTRO_US);
}

void calibrarCeroAngular() {
  // No hay excitacion ni tramas durante esta etapa; el servo sigue en 1544 us.
  sensors_event_t a, g, t;
  for(int i = 0; i < MUESTRAS_OFFSET_ANGULO; i++){ //se calcula el sesgo como el promedio de 100 mediciones
    mpu.getEvent(&a, &g, &t);
    offsetAnguloX += (180.0/PI) * atan2(a.acceleration.y, a.acceleration.z);
  }
  offsetAnguloX /= MUESTRAS_OFFSET_ANGULO;
}

void calibrarGyro() {
  float sumaXRadS = 0.0;  
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    sensors_event_t accel, gyro, temperatura;
    mpu.getEvent(&accel, &gyro, &temperatura);
    sumaXRadS += gyro.gyro.x + 10*(PI/180.0); // agrego sesgo artificial de 10°/s
    delay(3);
  }

  gyroX_offsetRadS = sumaXRadS / muestras;
}
