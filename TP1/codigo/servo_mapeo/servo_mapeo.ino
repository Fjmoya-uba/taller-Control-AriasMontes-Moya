#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Servo miServo;
Adafruit_MPU6050 mpu;

const byte PIN_SERVO = 9;
// Usar 1000000UL para actualizar la consigna a 1 Hz.
// Servo mantiene su refresco nominal de 20 ms en ambos casos.
const unsigned long PERIODO_CONTROL_US = 20000UL;
unsigned long ultimoControlUs;
unsigned long ultimoRegistroUs;
int pulsoPendiente = 900;
int pulsoAplicado = 900;
bool descartarEntrada = false;

// Modelo calibrado del angulo de la BARRA, no del eje del servo.
float pulsoAAngulo(int pulsoUs) {
  pulsoUs = constrain(pulsoUs, 550, 1600);
  if (pulsoUs <= 900) return (pulsoUs - 900) * (14.8f / 350.0f);
  return (pulsoUs - 900) * (22.3f / 700.0f);
}

int anguloAPulso(float grados) {
  grados = constrain(grados, -14.8f, 22.3f);
  const float pulso = grados <= 0.0f
      ? 900.0f + grados * (350.0f / 14.8f)
      : 900.0f + grados * (700.0f / 22.3f);
  return (int)lroundf(pulso);
}

const unsigned long SERIAL_BAUD = 115200;
const unsigned long PERIODO_IMU_US = 10000;  // 100 Hz
const float ALFA = 0.92f;                    // Peso del giroscopio

unsigned long ultimaLecturaUs;
float angulo = 0.0f;

// Buffer chico para recibir el ancho de pulso sin bloquear el muestreo IMU.
char entrada[12];
byte cantidadEntrada = 0;

float anguloAcelerometro(const sensors_event_t &accel) {
  // Inclinacion alrededor del eje X, en grados.
  return atan2(accel.acceleration.y, accel.acceleration.z) * 180.0f / PI;
}

void actualizarAngulo() {
  const unsigned long ahoraUs = micros();
  const unsigned long transcurridoUs = ahoraUs - ultimaLecturaUs;
  if (transcurridoUs < PERIODO_IMU_US) return;

  ultimaLecturaUs = ahoraUs;

  sensors_event_t accel, gyro, temperatura;
  mpu.getEvent(&accel, &gyro, &temperatura);

  const float dt = transcurridoUs * 1.0e-6f;
  // La libreria entrega rad/s; primero se convierte a grados/s.
  const float prediccionGyro = angulo + gyro.gyro.x * 180.0f / PI * dt;
  const float medicionAccel = anguloAcelerometro(accel);
  angulo = ALFA * prediccionGyro + (1.0f - ALFA) * medicionAccel;
}

void responderComando(const char *texto) {
  char *fin;
  const long anchoPulso = strtol(texto, &fin, 10);
  if (fin == texto || *fin != '\0' || anchoPulso < 550 || anchoPulso > 1600) {
    Serial.println("ERROR: envie un entero entre 550 y 1600 us");
    return;
  }
  pulsoPendiente = (int)anchoPulso;
}

void leerPuertoSerie() {
  // Presupuesto acotado para no bloquear indefinidamente la lectura IMU.
  for (byte n = 0; n < 16 && Serial.available() > 0; ++n) {
    const char caracter = Serial.read();
    if (caracter == '\n' || caracter == '\r') {
      if (descartarEntrada) {
        Serial.println("ERROR: linea demasiado larga");
      } else if (cantidadEntrada > 0) {
        entrada[cantidadEntrada] = '\0';
        responderComando(entrada);
      }
      cantidadEntrada = 0;
      descartarEntrada = false;
    } else if (!descartarEntrada) {
      if (cantidadEntrada < sizeof(entrada) - 1) {
        entrada[cantidadEntrada++] = caracter;
      } else {
        descartarEntrada = true;
      }
    }
  }
}

void actualizarControl() {
  const unsigned long ahoraUs = micros();
  if (ahoraUs - ultimoControlUs < PERIODO_CONTROL_US) return;
  ultimoControlUs = ahoraUs;
  pulsoAplicado = pulsoPendiente;
  miServo.writeMicroseconds(pulsoAplicado);
}

void registrar() {
  const unsigned long ahoraUs = micros();
  if (ahoraUs - ultimoRegistroUs < 100000UL) return;
  ultimoRegistroUs = ahoraUs;
  Serial.print("Pulso_us: ");
  Serial.print(pulsoAplicado);
  Serial.print(", Modelo_deg: ");
  Serial.print(pulsoAAngulo(pulsoAplicado), 3);
  Serial.print(", IMU_deg: ");
  Serial.println(angulo, 3);
}
void setup() {
  miServo.attach(PIN_SERVO, 550, 1600);
  miServo.writeMicroseconds(900);

  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(LED_BUILTIN, OUTPUT);
  if (!mpu.begin()) {
    Serial.println("ERROR: no se encontro la MPU6050");
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Arranca el filtro desde el angulo observado por el acelerometro.
  sensors_event_t accel, gyro, temperatura;
  mpu.getEvent(&accel, &gyro, &temperatura);
  angulo = anguloAcelerometro(accel);
  ultimaLecturaUs = micros();
  ultimoControlUs = ultimaLecturaUs;
  ultimoRegistroUs = ultimaLecturaUs;

  Serial.println("Envie un ancho de pulso en us y presione Enter (ej: 850)");
}

void loop() {
  actualizarAngulo();
  leerPuertoSerie();
  actualizarControl();
  registrar();
}
