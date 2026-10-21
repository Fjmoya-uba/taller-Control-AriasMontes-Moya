/* Ensayo SIN PD a 50 Hz. Calibracion inicial: 2 s quieto y horizontal.
 * Trama a 115200: "abcd" + [angulo_filtro_deg, distancia_cm, comando_us].
 * Tres float32 little endian: 16 bytes con cabecera, sin texto.
 * Configurar receptor para TRES single (12 bytes de datos). Distancia -1: sin eco.
 * d1: angulo; d2: distancia; d3: comando absoluto enviado al servo.
 * Comando actualizado despues de medir: entrada retenida hasta la muestra siguiente.
 * Tiempo del receptor: muestras a 50 Hz, t=0 en la primera trama.
 * p = distancia - 15.612 cm. Limites p: [-15,21] cm.
 * Medir primero y aplicar comando despues; IMU y eco separados unos ms.
 * 3 s horizontal + 12 pulsos de 1 s con descansos de 2 s + 6 s final.
 * Sin PD, rampas ni retorno automatico. Maximo 45 s desde primera trama.
 * Fuera de limites/sin eco: abortar y horizontal; LED queda encendido.
 * Sigue transmitiendo tras terminar/abortar; reset para repetir.
 * Horizontal NO frena instantaneamente. Validar ecos HC-SR04 a 50 Hz.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>

const uint8_t PIN_SERVO = 9, PIN_TRIG = 6, PIN_ECHO = 7;
const int PULSO_CENTRO_US = 1544;
const uint32_t TS_US = 20000UL;
const uint32_t TIMEOUT_ECO_US = 4000UL;
const float ALPHA = 0.94f;
const float US_POR_CM = 58.574f;
const float MAX_DISTANCIA_CM = 40.0f;
const uint32_t DURACION_ENSAYO_MUESTRAS = 2250; // 45 s; luego centro.
// Limites operativos de TP2/Informe/main.tex; sin saturacion angular adicional.
const int PULSO_MIN_US = 1100, PULSO_MAX_US = 2000;
const float REFERENCIA_CM = 15.612f;
const float POSICION_MIN_CM = -15.0f, POSICION_MAX_CM = 21.0f;
// Comandos fijos, dos bloques con orden invertido: 3 + 36 + 6 = 45 s.
struct Escalon { int16_t deltaUs; uint16_t muestras; };
const Escalon SECUENCIA[] = {
  {0, 150},
  {100, 30}, {0, 150}, {-70, 30}, {0, 150},
  {120, 30}, {0, 150}, {-75, 25}, {0, 150},
  {115, 40}, {0, 150}, {-75, 30}, {0, 150},
  {105, 50}, {0, 150}, {-70, 30}, {0, 150},
  {85, 70}, {0, 150}, {-50, 30}, {0, 150},
  {95, 50}, {0, 150}, {-60, 30}, {0, 150},
  {0, 300}
};
const uint8_t N_ESCALONES = sizeof(SECUENCIA) / sizeof(SECUENCIA[0]);
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(TIMEOUT_ECO_US + 3000UL < TS_US, "Reservar tiempo para IMU/TX");

Servo miServo;
Adafruit_MPU6050 mpu;
float biasGyro = 0, anguloFiltrado = 0, offsetAngulo = 0;
int pulsoUs = PULSO_CENTRO_US;
uint32_t siguienteUs = 0, muestra = 0;
uint8_t escalon = 0;
uint16_t cuentaEscalon = 0;
bool detenido = false;

bool vencido(uint32_t ahora, uint32_t plazo) {
  return (int32_t)(ahora - plazo) >= 0;
}

void detener() {
  detenido = true;
  pulsoUs = PULSO_CENTRO_US;
  miServo.writeMicroseconds(pulsoUs);
  digitalWrite(LED_BUILTIN, HIGH);
}

float anguloAcc(const sensors_event_t &a) {
  return atan2(a.acceleration.y,
               sqrt(sq(a.acceleration.x) + sq(a.acceleration.z))) * 180.0f / PI;
}

void actualizarAngulo() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  anguloFiltrado = ALPHA * (anguloFiltrado +
      (g.gyro.x - biasGyro) * 180.0f / PI * 0.02f) +
      (1.0f - ALPHA) * anguloAcc(a);
}

float medirDistancia() {
  // Captura acotada por micros(): no usar pulseIn() con timeout por defecto.
  // Durante este bloque no se accede a I2C ni a Serial para no perder el eco.
  if (digitalRead(PIN_ECHO) == HIGH) return -1.0f;
  digitalWrite(PIN_TRIG, HIGH);
  uint32_t inicio = micros();
  while (micros() - inicio < 10UL) {}
  digitalWrite(PIN_TRIG, LOW);
  inicio = micros();
  while (digitalRead(PIN_ECHO) == LOW)
    if (micros() - inicio >= TIMEOUT_ECO_US) return -1.0f;
  const uint32_t subida = micros();
  while (digitalRead(PIN_ECHO) == HIGH)
    if (micros() - inicio >= TIMEOUT_ECO_US) return -1.0f;
  const float cm = (micros() - subida) / US_POR_CM;
  return cm >= 2.0f && cm <= MAX_DISTANCIA_CM ? cm : -1.0f;
}

void enviarTrama(float distancia) {
  if (Serial.availableForWrite() < 16 ||
      (int32_t)(siguienteUs - micros()) < 2500) {
    detener(); // No esperar a la UART ni alterar la grilla de muestreo.
    return;
  }
  uint8_t trama[16] = {'a', 'b', 'c', 'd'};
  // pulsoUs tambien se actualiza a horizontal en detener().
  const float valores[] = {anguloFiltrado - offsetAngulo, distancia, (float)pulsoUs};
  for (uint8_t i = 0; i < 3; ++i) {
    uint32_t bits;
    memcpy(&bits, &valores[i], 4);
    for (uint8_t j = 0; j < 4; ++j)
      trama[4 + 4 * i + j] = (uint8_t)(bits >> (8 * j));
  }
  Serial.write(trama, sizeof(trama));
}

void aplicarExcitacion() {
  pulsoUs = constrain(PULSO_CENTRO_US + SECUENCIA[escalon].deltaUs,
                      PULSO_MIN_US, PULSO_MAX_US);
  miServo.writeMicroseconds(pulsoUs);
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  Serial.begin(115200);
  miServo.writeMicroseconds(PULSO_CENTRO_US);
  miServo.attach(PIN_SERVO);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  if (!mpu.begin(0x68)) {
    detener();
    while (true) {}
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // 100 muestras a 20 ms: ~2 s en centro, sin transmision ni excitacion.
  // Primer segundo: bias gyro. Segundo: filtro y promedio del cero.
  siguienteUs = micros() + TS_US;
  float sumaGyro = 0, sumaAngulo = 0;
  for (uint8_t i = 0; i < 100; ++i) {
    while (!vencido(micros(), siguienteUs)) {}
    siguienteUs += TS_US;
    if (i < 50) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      sumaGyro += g.gyro.x;
      anguloFiltrado = anguloAcc(a);
      if (i == 49) biasGyro = sumaGyro / 50.0f;
    } else {
      actualizarAngulo();
      if (i >= 75) sumaAngulo += anguloFiltrado;
    }
    if (vencido(micros(), siguienteUs)) {
      detener();
      while (true) {}
    }
  }
  offsetAngulo = sumaAngulo / 25.0f;
}

void loop() {
  const uint32_t ahora = micros();
  if (!vencido(ahora, siguienteUs)) return;
  siguienteUs += TS_US; // Grilla absoluta, sin acumular tiempo de ejecucion.
  if (vencido(ahora, siguienteUs)) {
    detener();
    while (true) {} // No fabricar muestras consecutivas para recuperar tiempo.
  }
  actualizarAngulo();
  const float distancia = medirDistancia();
  const float posicion = distancia - REFERENCIA_CM;
  if (!isfinite(distancia) || distancia < 0 ||
      posicion < POSICION_MIN_CM || posicion > POSICION_MAX_CM ||
      muestra >= DURACION_ENSAYO_MUESTRAS) detener();

  if (!detenido) {
    aplicarExcitacion();
    if (++cuentaEscalon >= SECUENCIA[escalon].muestras) {
      cuentaEscalon = 0;
      ++escalon;
      if (escalon >= N_ESCALONES) detener();
    }
  }
  enviarTrama(distancia);
  if (muestra < DURACION_ENSAYO_MUESTRAS) ++muestra;
  if (vencido(micros(), siguienteUs)) {
    detener();
    while (true) {} // Ensayo invalido por plazo perdido.
  }
}

