#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Servo miServo;
Adafruit_MPU6050 mpu;

const byte PIN_SERVO = 9;
//NEW SKETCH;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long PERIODO_IMU_US = 10000;  // 100 Hz
const float ALFA = 0.98f;                    // Peso del giroscopio

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

  if (fin == texto || *fin != '\0' || anchoPulso <= 0 || anchoPulso > 3000) {
    Serial.println("ERROR: envie un ancho de pulso entre 1 y 3000 us");
    return;
  }

  miServo.writeMicroseconds((int)anchoPulso);
  Serial.print("Pulso_us: ");
  Serial.print(anchoPulso);
  //Serial.print(", Angulo_deg: ");
  //Serial.println(angulo, 2);
}

void leerPuertoSerie() {
  while (Serial.available() > 0) {
    const char caracter = Serial.read();

    if (caracter == '\n' || caracter == '\r') {
      if (cantidadEntrada > 0) {
        entrada[cantidadEntrada] = '\0';
        responderComando(entrada);
        cantidadEntrada = 0;
      }
    } else if (cantidadEntrada < sizeof(entrada) - 1) {
      entrada[cantidadEntrada++] = caracter;
    } else {
      cantidadEntrada = 0;
    }
  }
}

void setup() {
  miServo.attach(PIN_SERVO);
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

  Serial.println("Envie un ancho de pulso en us y presione Enter (ej: 850)");
}

void loop() {
  actualizarAngulo();
  leerPuertoSerie();
}
