/* Ensayos libres: pulsos fijos alrededor de 1544 us; tres repeticiones.
 * TRIG=6, ECHO=7, servo=9, MPU6050 por I2C.
 * TX a 50 Hz: "abcd" + angulo_deg + distancia_cm (float32 LE).
 * Sin eco: -1. No se transmite texto ni referencia ni pulso.
 * Ver ../../README_ensayo_rozamiento.md antes de usar.
 * El angulo solo se mide: no hay control de angulo ni de posicion.
 * Arranque automatico despues de calibrar (~4 s), sin comandos del PC.
 * Al terminar centra y deja de transmitir; RESET para repetir.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>

const uint32_t TS_US = 20000UL;
const float TS = 0.02f, ALPHA = 0.94f;
const int CENTRO_US = 1544; // Horizontal comprobada experimentalmente.
const float CERO_IMU_DEG = 0.0f; // Lectura del acelerometro con barra horizontal REAL.
// Dos amplitudes de comando, NO grados: ajustar segun el movimiento observado.
const int DELTAS_US[] = {100, -100, 200, -200};
const uint8_t REPETICIONES = 3;
const uint16_t PAUSA = 200, ASENTAMIENTO = 100, RECORRIDO = 200;
const uint16_t CICLO = PAUSA + ASENTAMIENTO + RECORRIDO;
const uint8_t N_COMANDOS = sizeof(DELTAS_US) / sizeof(DELTAS_US[0]);
const uint32_t TOTAL = (uint32_t)CICLO * N_COMANDOS * REPETICIONES;
static_assert(sizeof(float) == 4, "Se requiere float32");

Servo servo;
Adafruit_MPU6050 mpu;
float angulo = 0, bias = 0;
uint32_t siguiente, muestra = 0;
bool terminado = false;

bool vencido(uint32_t ahora, uint32_t plazo) {
  return (int32_t)(ahora - plazo) >= 0;
}

void fallo() {
  servo.writeMicroseconds(CENTRO_US);
  digitalWrite(LED_BUILTIN, HIGH);
  while (true) {} // Reiniciar: no continuar una grilla temporal invalida.
}

float anguloAcc(const sensors_event_t &a) {
  return atan2(a.acceleration.y,
      sqrt(sq(a.acceleration.x) + sq(a.acceleration.z))) * 180.0f / PI - CERO_IMU_DEG;
}

void leerIMU(sensors_event_t &a, sensors_event_t &g) {
  sensors_event_t temperatura;
#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif
  bool ok = mpu.getEvent(&a, &g, &temperatura);
#if defined(WIRE_HAS_TIMEOUT)
  ok = ok && !Wire.getWireTimeoutFlag();
#endif
  if (!ok || !isfinite(a.acceleration.x) || !isfinite(a.acceleration.y) ||
      !isfinite(a.acceleration.z) || !isfinite(g.gyro.x)) fallo();
}

float distanciaCm() {
  // Captura acotada; no bloquear con el timeout por defecto de pulseIn.
  if (digitalRead(7) == HIGH) return -1.0f;
  digitalWrite(6, HIGH);
  delayMicroseconds(10);
  digitalWrite(6, LOW);
  uint32_t inicio = micros();
  while (digitalRead(7) == LOW)
    if (micros() - inicio >= 4000UL) return -1.0f;
  uint32_t subida = micros();
  while (digitalRead(7) == HIGH)
    if (micros() - inicio >= 4000UL) return -1.0f;
  float cm = (micros() - subida) / 58.574f;
  return cm >= 2.0f && cm <= 40.0f ? cm : -1.0f;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(7, INPUT);
  Serial.begin(115200);
  servo.writeMicroseconds(CENTRO_US);
  servo.attach(9);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  if (!mpu.begin(0x68)) fallo();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  // Barra inmovil: calibrar gyro, sin redefinir como horizontal su inclinacion.
  delay(1000);
  float suma = 0;
  for (uint16_t i = 0; i < 150; ++i) {
    sensors_event_t a, g;
    leerIMU(a, g);
    suma += g.gyro.x;
    angulo = anguloAcc(a);
    delay(20);
  }
  bias = suma / 150.0f;
  siguiente = micros() + TS_US;
}

void loop() {
  if (terminado) return;
  if (!vencido(micros(), siguiente)) return;
  siguiente += TS_US;
  if (vencido(micros(), siguiente)) fallo();
  sensors_event_t a, g;
  leerIMU(a, g);
  angulo = ALPHA * (angulo + (g.gyro.x - bias) * 180.0f / PI * TS)
         + (1.0f - ALPHA) * anguloAcc(a);

  const uint16_t fase = muestra % CICLO;
  const int deltaUs = DELTAS_US[(muestra / CICLO) % N_COMANDOS];
  servo.writeMicroseconds(fase < PAUSA ? CENTRO_US : CENTRO_US + deltaUs);
  digitalWrite(LED_BUILTIN, fase >= PAUSA + ASENTAMIENTO ? HIGH : LOW);
  float distancia = distanciaCm();
  if (Serial.availableForWrite() < 12 || (int32_t)(siguiente - micros()) < 2000) fallo();
  uint8_t trama[12] = {'a', 'b', 'c', 'd'};
  const float valores[] = {angulo, distancia};
  for (uint8_t i = 0; i < 2; ++i) {
    uint32_t bits;
    memcpy(&bits, &valores[i], 4);
    for (uint8_t j = 0; j < 4; ++j) trama[4 + 4*i + j] = (uint8_t)(bits >> (8*j));
  }
  Serial.write(trama, sizeof(trama));
  if (++muestra >= TOTAL) {
    servo.writeMicroseconds(CENTRO_US);
    terminado = true; // Tanda terminada: no reinicia hasta reset.
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (vencido(micros(), siguiente)) fallo();
}
