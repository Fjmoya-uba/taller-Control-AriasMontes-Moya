#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Servo miServo;
Adafruit_MPU6050 mpu;
const byte PIN_SERVO = 9;
const int PULSO_CERO_US = 1544;
const float ALPHA = 0.94f;
const float SIGNO_BARRA = 1.0f; // Cambiar a -1 si el sentido del informe es opuesto.
const unsigned long PERIODO_US = 20000UL; // 50 Hz, como el ensayo previo.
int pulsoUs = PULSO_CERO_US;
float filtrado = 0, offset = 0, biasGyro = 0;
unsigned long ultimaImuUs, ultimaImpresionMs;
char entrada[16];
byte longitud = 0;
bool descartar = false;

float inclinacion(const sensors_event_t &acc) {
  // IMU fijada a la barra, giro alrededor del eje X.
  return atan2(acc.acceleration.y, acc.acceleration.z) * RAD_TO_DEG;
}

void aplicarPulso(int valor) {
  pulsoUs = valor;
  miServo.writeMicroseconds(valor);
}

void calibrarCero() {
  aplicarPulso(PULSO_CERO_US);
  Serial.println(F("Calibrando en 1544 us: mantener barra horizontal y quieta durante 5 s."));
  delay(2000);
  float sumaGyro = 0, sumaAngulo = 0;
  for (int i = 0; i < 150; ++i) {
    sensors_event_t acc, gyro, temp;
    mpu.getEvent(&acc, &gyro, &temp);
    sumaGyro += gyro.gyro.x;
    sumaAngulo += inclinacion(acc);
    delay(20);
  }
  biasGyro = sumaGyro / 150;
  offset = sumaAngulo / 150;
  filtrado = offset;
  ultimaImuUs = micros();
  Serial.print(F("Offset IMU [deg]=")); Serial.println(offset, 3);
  Serial.print(F("Bias gyro X [deg/s]=")); Serial.println(biasGyro * RAD_TO_DEG, 3);
  Serial.println(F("Cero relativo guardado en RAM. Verificar horizontalidad con nivel."));
}

void actualizarFiltro() {
  const unsigned long ahora = micros();
  const unsigned long transcurrido = ahora - ultimaImuUs;
  if (transcurrido < PERIODO_US) return;
  ultimaImuUs = ahora;
  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);
  const float dt = transcurrido * 1.0e-6f;
  filtrado = ALPHA * (filtrado + (gyro.gyro.x - biasGyro) * RAD_TO_DEG * dt)
      + (1.0f - ALPHA) * inclinacion(acc);
}

void imprimirDatos() {
  // Mapeo nominal de biblioteca; NO mide el giro fisico del eje.
  const float escala = 180.0f / (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);
  Serial.print(F("u_us=")); Serial.print(pulsoUs);
  Serial.print(F(" | du_us=")); Serial.print(pulsoUs - PULSO_CERO_US);
  Serial.print(F(" | write_equiv_deg="));
  Serial.print((pulsoUs - MIN_PULSE_WIDTH) * escala, 2);
  Serial.print(F(" | theta_s_nom_deg="));
  Serial.print((pulsoUs - PULSO_CERO_US) * escala, 2);
  Serial.print(F(" | theta_b_deg=")); Serial.print(SIGNO_BARRA * (filtrado - offset), 3);
  Serial.print(F(" | filtro_sin_cero_deg=")); Serial.print(filtrado, 3);
  Serial.print(F(" | offset_deg=")); Serial.println(offset, 3);
}

void ayuda() {
  Serial.println(F("115200 baudios; Nueva linea o Ambos NL y CR."));
  Serial.println(F("Enviar pulsos enteros 1000..2000 us (YA NO ANGULOS). Ejemplo: 1544."));
  Serial.println(F("c: volver a 1544 y recalibrar cero/bias; ?: ayuda."));
  Serial.println(F("Arrancar desde 1544, pasos de 25 us; detenerse antes de un tope."));
  Serial.println(F("theta_s_nom: comando nominal relativo a 1544, no giro fisico medido."));
  Serial.println(F("theta_b: IMU relativa al cero. Alpha=0.94, muestreo=50 Hz."));
  Serial.print(F("Mapeo nominal 0..180 deg -> ")); Serial.print(MIN_PULSE_WIDTH);
  Serial.print(F("..")); Serial.print(MAX_PULSE_WIDTH); Serial.println(F(" us."));
}

void procesarEntrada() {
  entrada[longitud] = '\0';
  if (strcmp(entrada, "c") == 0) { calibrarCero(); return; }
  if (strcmp(entrada, "?") == 0) { ayuda(); return; }
  if (longitud != 4) {
    Serial.println(F("ERROR: enviar 1000..2000 us, c o ?.")); return;
  }
  for (byte i = 0; i < longitud; ++i) {
    if (entrada[i] < '0' || entrada[i] > '9') {
      Serial.println(F("ERROR: enviar solo digitos para el pulso.")); return;
    }
  }
  const long valor = strtol(entrada, NULL, 10);
  if (valor < 1000 || valor > 2000) {
    Serial.println(F("ERROR: pulso fuera de 1000..2000 us.")); return;
  }
  aplicarPulso((int)valor);
  Serial.println(F("Comando aplicado. Esperar estabilizacion antes de registrar."));
}

void leerPuertoSerie() {
  for (byte n = 0; n < 16 && Serial.available() > 0; ++n) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (descartar) Serial.println(F("ERROR: linea demasiado larga."));
      else if (longitud > 0) procesarEntrada();
      longitud = 0;
      descartar = false;
    } else if (!descartar) {
      if (longitud < sizeof(entrada) - 1) entrada[longitud++] = c;
      else descartar = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  miServo.writeMicroseconds(PULSO_CERO_US);
  // Explicitar el mismo mapeo usado para los grados nominales impresos.
  miServo.attach(PIN_SERVO, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 no detectada. Revisar conexion."));
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  ayuda();
  calibrarCero();
  ultimaImpresionMs = millis();
}

void loop() {
  actualizarFiltro();
  leerPuertoSerie();
  if (millis() - ultimaImpresionMs >= 250) {
    ultimaImpresionMs = millis();
    imprimirDatos();
  }
}
