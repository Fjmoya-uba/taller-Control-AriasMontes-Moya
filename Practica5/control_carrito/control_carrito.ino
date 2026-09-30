/* PID de posicion del carrito + control P del angulo de la barra.
 * HC-SR04: TRIG 6, ECHO 7. MPU6050 por I2C. Servo en 9.
 * Mantener la barra quieta al arrancar para calibrar el giroscopio.
 * Monitor/Plotter Serie a 115200 (texto, no la trama binaria anterior).
 */
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// -------- Parametros para modificar --------
const float REFERENCIA_CM = 17.0f; // Distancia deseada desde el sensor: medir.
const float kp = 8.0f;            // grados / cm
const float ki = 0.0f;            // grados / (cm*s)
const float kd = 0.0f;            // grados*s / cm
const float MAX_ANGULO = 15.0f;    // Limite de inclinacion de la barra.
const float KP_ANGULO = 20.0f;    // us / grado, ajustar primero este lazo.
// Ganancias iniciales de prueba, no sintonizadas para el mecanismo.
// Calibracion de mover_servo.ino: comandos nominales, no angulos de barra.
const int ANGULO_SERVO_CERO = 7;
const int ANGULO_SERVO_MIN = -35; // -42 grados nominales respecto del cero.
const int ANGULO_SERVO_MAX = 81;  // +74 grados nominales respecto del cero.
// Reproduce Servo.write(comando + 90) con el attach() del ensayo.
const int PULSO_CENTRO = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_CERO + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const int PULSO_MIN = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_MIN + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const int PULSO_MAX = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_MAX + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const float CERO_IMU = 0.0f;      // Angulo del acelerometro con barra horizontal.
// +1 si angulo positivo hace AUMENTAR distancia; -1 si la hace disminuir.
const float SIGNO_POSICION = 1.0f;
// +1 si aumentar pulso AUMENTA angulo medido; -1 si lo disminuye.
const float SIGNO_SERVO = 1.0f;
const float ALFA = 0.92f;

Servo servo;
Adafruit_MPU6050 mpu;
NewPing sonar(6, 7, 40);          // Hasta 40 cm.
float angulo = 0, biasGyro = 0;
float distancia = 0, distanciaAnterior = 0;
float referenciaAngulo = 0, integral = 0;
bool distanciaValida = false;
int pulso = PULSO_CENTRO;
unsigned long ultimaIMU, ultimaPosicion;

float anguloAccel(const sensors_event_t &a) {
  // Mismo eje y expresion que en control_angulo.ino.
  return atan2(a.acceleration.y,
               sqrt(sq(a.acceleration.x) + sq(a.acceleration.z))) *
         180.0f / PI - CERO_IMU;
}

void actualizarAngulo() {
  unsigned long ahora = micros();
  if (ahora - ultimaIMU < 10000UL) return; // IMU y lazo de angulo: 100 Hz. Ojo que tengo que controlar a 50Hz
  float dt = (ahora - ultimaIMU) * 1e-6f;
  ultimaIMU = ahora;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float velocidad = (g.gyro.x - biasGyro) * 180.0f / PI;
  angulo = ALFA * (angulo + velocidad * dt) + (1.0f - ALFA) * anguloAccel(a);

  // El PID de posicion pide grados; este lazo entrega microsegundos.
  float mando = PULSO_CENTRO + SIGNO_SERVO * KP_ANGULO * (referenciaAngulo - angulo);
  pulso = (int)lroundf(constrain(mando, (float)PULSO_MIN, (float)PULSO_MAX));
  servo.writeMicroseconds(pulso);
}

void actualizarPosicion() {
  unsigned long ahora = micros();
  if (ahora - ultimaPosicion < 20000UL) return; // Un eco cada 60 ms.
  float dt = (ahora - ultimaPosicion) * 1e-6f;
  ultimaPosicion = ahora;
  unsigned int eco = sonar.ping();

  if (eco == 0) {
    // Sin eco: pedir barra horizontal, no interpretar el fallo como 0 cm.
    distanciaValida = false;
    referenciaAngulo = 0;
    integral = 0;
  } else {
    distancia = eco / 58.574f; // Misma conversion que en Practica1.
    float error = REFERENCIA_CM - distancia;
    // Derivada sobre la medicion: evita un salto al cambiar la referencia.
    float velocidad = distanciaValida ? (distancia - distanciaAnterior) / dt : 0;
    distanciaAnterior = distancia;
    distanciaValida = true;

    // Guardamos el aporte integral en grados. Con ki=0 queda desactivado.
    float nuevaIntegral = ki == 0 ? 0 : integral + ki * error * dt;
    float salida = kp * error + nuevaIntegral - kd * velocidad;
    // No acumular integral si empuja mas alla del limite de inclinacion.
    if (ki == 0 || (salida >= -MAX_ANGULO && salida <= MAX_ANGULO) ||
        (salida > MAX_ANGULO && ki * error < 0) ||
        (salida < -MAX_ANGULO && ki * error > 0)) {
      integral = nuevaIntegral;
    }
    salida = kp * error + integral - kd * velocidad;
    referenciaAngulo = SIGNO_POSICION * constrain(salida, -MAX_ANGULO, MAX_ANGULO);
  }

  Serial.print("ref_cm:"); Serial.print(REFERENCIA_CM);
  Serial.print(" dist_cm:"); Serial.print(distanciaValida ? distancia : -1);
  Serial.print(" ref_deg:"); Serial.print(referenciaAngulo);
  Serial.print(" ang_deg:"); Serial.print(angulo);
  Serial.print(" pulso_us:"); Serial.println(pulso);
  // dist_cm=-1 indica falta de eco, no una posicion real.
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  if (!mpu.begin(0x68)) {
    Serial.println("ERROR: no se encontro la MPU6050");
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  pulso = constrain(PULSO_CENTRO, PULSO_MIN, PULSO_MAX);
  servo.writeMicroseconds(pulso);
  servo.attach(9); // Mismo rango predeterminado que mover_servo.ino.
  delay(1000);

  sensors_event_t a, g, t;
  for (int i = 0; i < 500; i++) {
    mpu.getEvent(&a, &g, &t);
    biasGyro += g.gyro.x;
    delay(3);
  }
  biasGyro /= 500;
  angulo = anguloAccel(a);
  ultimaIMU = ultimaPosicion = micros();
}

void loop() {
  actualizarAngulo();
  actualizarPosicion();
}
