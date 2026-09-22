#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Servo servo;
Adafruit_MPU6050 mpu;
const int PULSO_INICIAL_US = 664;  // Barra aproximadamente a -10 grados.
const int PULSO_FINAL_US = 1528;   // Barra aproximadamente a +20 grados.
const float ANGULO_INICIAL = -10.0f;
const float ANGULO_FINAL = 20.0f;
const float TOLERANCIA_DEG = 1.0f;
const unsigned long PERMANENCIA_US = 100000UL;
const unsigned long LIMITE_US = 3000000UL;
const unsigned long MUESTREO_US = 10000UL;  // 100 Hz.
const float ALFA = 0.92f;
float angulo;
unsigned long ultimaLecturaUs;

float anguloAccel(const sensors_event_t &a) {
  return atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
}

bool actualizarAngulo() {
  const unsigned long ahora = micros();
  const unsigned long dtUs = ahora - ultimaLecturaUs;
  if (dtUs < MUESTREO_US) return false;
  ultimaLecturaUs = ahora;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  const float prediccion = angulo + g.gyro.x * 180.0f / PI * dtUs * 1.0e-6f;
  angulo = ALFA * prediccion + (1.0f - ALFA) * anguloAccel(a);
  return true;
}

void ensayar() {
  Serial.println(F("Preparando posicion inicial durante 2 s..."));
  servo.writeMicroseconds(PULSO_INICIAL_US);
  const unsigned long preparacion = micros();
  unsigned long inicioReposo = preparacion;
  bool enReposo = false;
  while (micros() - preparacion < 2000000UL) {
    if (!actualizarAngulo()) continue;
    if (fabs(angulo - ANGULO_INICIAL) <= TOLERANCIA_DEG) {
      if (!enReposo) inicioReposo = micros();
      enReposo = true;
    } else {
      enReposo = false;
    }
  }
  if (!enReposo || micros() - inicioReposo < PERMANENCIA_US) {
    Serial.println(F("Ensayo cancelado: no se alcanzo la posicion inicial."));
    Serial.println(F("Revisar referencia de la IMU y calibracion de los pulsos."));
    return;
  }

  Serial.println(F("tiempo_ms,angulo_barra_deg"));
  Serial.flush();
  bool dentro = false;
  bool confirmado = false;
  unsigned long entradaUs = 0;
  const unsigned long t0 = micros();
  servo.writeMicroseconds(PULSO_FINAL_US);
  while (micros() - t0 < LIMITE_US) {
    if (!actualizarAngulo()) continue;
    const unsigned long transcurrido = micros() - t0;
    Serial.print(transcurrido / 1000.0f, 1);
    Serial.print(',');
    Serial.println(angulo, 2);

    if (fabs(angulo - ANGULO_FINAL) <= TOLERANCIA_DEG) {
      if (!dentro) entradaUs = transcurrido;
      dentro = true;
      if (transcurrido - entradaUs >= PERMANENCIA_US) {
        confirmado = true;
        break;
      }
    } else {
      dentro = false;
    }
  }

  if (confirmado) {
    Serial.print(F("Tiempo de llegada (ms): "));
    Serial.println(entradaUs / 1000.0f, 1);
    Serial.println(F("Entrada a +/-1 grado, confirmada durante 100 ms."));
  } else {
    Serial.println(F("Sin llegada confirmada en 3000 ms."));
  }
  // Se mantiene la posicion final; el siguiente ensayo vuelve al inicio.
  Serial.println(F("Enviar t para repetir."));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println(F("ERROR: no se encontro la MPU6050."));
    while (true) delay(100);
  }
  Wire.setClock(400000);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  angulo = anguloAccel(a);
  ultimaLecturaUs = micros();
  servo.attach(9, 550, 1600);
  servo.writeMicroseconds(900);
  Serial.println(F("Enviar t para medir el paso de -10 a +20 grados de barra."));
  // El resultado incluye espera del pulso, mecanica y retardo de IMU/filtro.
  // Muestreo de 10 ms: los decimales impresos no implican esa precision.
  // La permanencia confirma llegada, no garantiza establecimiento posterior.
}

void loop() {
  actualizarAngulo();
  if (Serial.available() > 0 && Serial.read() == 't') ensayar();
}
