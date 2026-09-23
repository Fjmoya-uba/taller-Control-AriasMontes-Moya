/* Barra con MPU6050 y servo: referencia en GRADOS por Serial a 115200.
 * PI discretizado con Tustin. Ver ../README_control.md.
 * IMU solidaria a la BARRA, mismo eje y montaje que en la identificacion.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <stdlib.h>

// -------- Parametros para modificar --------
const byte PIN_SERVO = 9;
const unsigned long TS_US = 20000UL;
const float TS = TS_US * 1e-6f;
// Copiar KP, KI y U0_US desde disenar_control.m antes del ensayo.
const float KP = 50.0f;          // us/grado
const float KI = 200.0f;          // us/(grado*s)
const float U0_US = 900.0f;     // Pulso de equilibrio
const float UMIN_US = 550.0f;   // Rango usado para identificar
const float UMAX_US = 1600.0f;
const float REFERENCIA_MIN = -10.0f;
const float REFERENCIA_MAX = 10.0f;
const float CERO_IMU = 0.0f;    // Lectura de IMU con barra horizontal
const float ALFA = 0.92f;       // Filtro a 100 Hz, igual que identificacion
const unsigned long IMU_US = 10000UL;

Adafruit_MPU6050 mpu;
Servo servo;
float angulo = 0, referencia = 0, biasGyro = 0;
float integral = 0, errorAnterior = 0, pulso = U0_US;
unsigned long ultimaIMU, siguienteControl, ultimoRegistro;
char entrada[24];
byte cantidad = 0;
bool descartar = false;

float anguloAccel(const sensors_event_t &a) {
  // Misma expresion que en el registro usado para estimar G.
  return atan2(a.acceleration.y,
               sqrt(sq(a.acceleration.x) + sq(a.acceleration.z))) *
         180.0f / PI - CERO_IMU;
}

void actualizarIMU() {
  const unsigned long ahora = micros();
  if (ahora - ultimaIMU < IMU_US) return;
  const float dt = (ahora - ultimaIMU) * 1e-6f;
  ultimaIMU = ahora;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  const float velocidad = (g.gyro.x - biasGyro) * 180.0f / PI;
  angulo = ALFA * (angulo + velocidad * dt) + (1-ALFA) * anguloAccel(a);
}

// -------- Cambiar esta funcion para probar otro controlador --------
float controlar(float error) {
  // C(s) = KP + KI/s; 1/s -> (TS/2)*(1+z^-1)/(1-z^-1).
  // I[k] = I[k-1] + KI*TS/2 * (e[k]+e[k-1]).
  const float incremento = KI * TS * 0.5f * (error + errorAnterior);
  const float candidata = integral + incremento;
  const float pedido = U0_US + KP * error + candidata;
  // Anti-windup: no acumular si el incremento empeora la saturacion.
  if (!((pedido > UMAX_US && incremento > 0) ||
        (pedido < UMIN_US && incremento < 0))) {
    integral = candidata;
  }
  errorAnterior = error;
  return constrain(U0_US + KP * error + integral, UMIN_US, UMAX_US);
}

void leerReferencia() {
  // Leer pocos caracteres por vuelta, sin parseFloat() ni esperas.
  for (byte n = 0; n < 16 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!descartar && cantidad > 0) {
        entrada[cantidad] = '\0';
        char *fin;
        float valor = strtod(entrada, &fin);
        if (fin != entrada && *fin == '\0' && isfinite(valor) &&
            valor >= REFERENCIA_MIN && valor <= REFERENCIA_MAX) {
          referencia = valor;
        } else {
          Serial.println(F("Referencia invalida o fuera de limites"));
        }
      } else if (descartar) {
        Serial.println(F("Linea demasiado larga"));
      }
      cantidad = 0;
      descartar = false;
    } else if (!descartar) {
      if (cantidad < sizeof(entrada)-1) entrada[cantidad++] = c;
      else descartar = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  if (!mpu.begin(0x68)) {
    Serial.println(F("No se encontro la MPU6050"));
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  if (KP == 0 && KI == 0) {
    Serial.println(F("Copiar KP y KI de disenar_control.m antes de iniciar"));
    while (true) delay(100);
  }
  servo.attach(PIN_SERVO, 500, 2500);
  pulso = constrain(U0_US, UMIN_US, UMAX_US);
  servo.writeMicroseconds((int)lroundf(pulso));
  Serial.println(F("Mantener quieto: calibrando giroscopio..."));
  delay(1000); // Dejar asentar el servo antes de calibrar.
  sensors_event_t a, g, t;
  for (int i = 0; i < 500; ++i) {
    mpu.getEvent(&a, &g, &t);
    biasGyro += g.gyro.x;
    delay(3);
  }
  biasGyro /= 500;
  angulo = anguloAccel(a);
  referencia = angulo; // Arrancar manteniendo el angulo actual, sin salto.
  ultimaIMU = micros();
  siguienteControl = ultimaIMU + TS_US;
  ultimoRegistro = ultimaIMU;
  Serial.println(F("Enviar referencia en grados y Enter (ejemplo: 0 o 3.5)"));
  Serial.println(F("ref_deg,angulo_deg,servo_us"));
}

void loop() {
  actualizarIMU();
  const unsigned long ahora = micros();
  if ((long)(ahora - siguienteControl) >= 0) {
    siguienteControl += TS_US;
    // Si hubo demora, no ejecutar varios pasos con una misma medicion.
    if ((long)(ahora - siguienteControl) >= 0) siguienteControl = ahora + TS_US;
    pulso = controlar(referencia - angulo);
    servo.writeMicroseconds((int)lroundf(pulso));
  }
  leerReferencia();
  if (ahora - ultimoRegistro >= 100000UL && Serial.availableForWrite() >= 48) {
    ultimoRegistro = ahora;
    Serial.print(referencia, 2); Serial.print(',');
    Serial.print(angulo, 2); Serial.print(',');
    Serial.println(pulso, 0);
  }
}
