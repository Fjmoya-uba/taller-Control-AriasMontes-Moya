/* Barra con MPU6050 y servo: referencia constante en GRADOS en el codigo.
 * Cascada discretizada con Tustin. Ver ../README_control.md.
 * TX: "abcd" + referencia_deg + angulo_deg + servo_us (3 float32).
 * IMU solidaria a la BARRA, mismo eje y montaje que en la identificacion.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// -------- Parametros para modificar --------
const byte PIN_SERVO = 9;
const unsigned long TS_US = 20000UL;
const float TS = TS_US * 1e-6f;
// PI inicial; ganancias pendientes de validacion experimental.
const float KP = 1.0f;          // grado de mando / grado de error
const float KI = 2.0f;          // grado de mando / (grado de error * s)
const float U0_US = 900.0f;     // Pulso de equilibrio
const float UMIN_US = 550.0f;   // Rango usado para identificar
const float UMAX_US = 1600.0f;
const float MANDO_MIN_DEG = -14.8f;
const float MANDO_MAX_DEG = 22.3f;
constexpr float REFERENCIA_MIN = -10.0f;
constexpr float REFERENCIA_MAX = 10.0f;
constexpr float REFERENCIA_DEG = 10.0f; // Editar aqui el angulo deseado en grados.
static_assert(REFERENCIA_DEG >= REFERENCIA_MIN && REFERENCIA_DEG <= REFERENCIA_MAX,
              "Referencia fuera del rango permitido (-10 a 10 grados)");
const float CERO_IMU = 0.0f;    // Lectura de IMU con barra horizontal
const float ALFA = 0.92f;       // Filtro a 100 Hz, igual que identificacion
const unsigned long IMU_US = 10000UL;

Adafruit_MPU6050 mpu;
Servo servo;
float angulo = 0, biasGyro = 0;
float pulso = U0_US;
unsigned long ultimaIMU, siguienteControl;

// =====================================================================
//  Controlador genérico: cascada de etapas (ganancia, cero y polo)
//  Cada etapa: C_i(s) = K_i * (s + cero_i) / (s + polo_i)
//  Se discretiza con Tustin, igual método que ya usabas para la integral.
// =====================================================================

// Cada fila: {K, cero, polo}, C_i(s) = K*(s+cero)/(s+polo).
// Parametros CONTINUOS en rad/s; no copiar los z_c/p_c discretos de Cz.
// Agregar filas para encadenar etapas. Ganancia pura: {K, 0, 0}.
// Colocar el integrador en la ultima etapa para el tracking de saturacion.
const float CONFIG_ETAPAS[][3] = {
  {KP, KI / KP, 0.0f} // Equivale al PI original: KP + KI/s; requiere KP != 0.
};
const size_t N_ETAPAS = sizeof(CONFIG_ETAPAS) / sizeof(CONFIG_ETAPAS[0]);
const float KAW = 1.0f; // Correccion por muestra (0..1).
static_assert(sizeof(float) == 4, "La trama requiere float de 4 bytes");

struct Etapa {
  float b0, b1, a1;  // coeficientes discretos
  float w;           // memoria interna
};

Etapa etapas[N_ETAPAS];

// ganancia, cero y polo en rad/s (continuo). polo=0 -> comportamiento integrador.
// cero=0 -> comportamiento derivativo (con el polo actuando de filtro).
void configurarEtapa(Etapa &e, float ganancia, float cero, float polo, float Ts) {
  const float c = 2.0f / Ts;
  e.b0 = ganancia * (c + cero) / (c + polo);
  e.b1 = ganancia * (cero - c) / (c + polo);
  e.a1 = (polo - c) / (c + polo);
  e.w  = 0;
}

float procesarEtapa(Etapa &e, float x) {
  float y = e.b0 * x + e.w;
  e.w = e.b1 * x - e.a1 * y;
  return y;
}

// Calibracion de TP1/servo_mapeo: angulo de la BARRA, no del eje del servo.
int anguloAPulso(float grados) {
  grados = constrain(grados, MANDO_MIN_DEG, MANDO_MAX_DEG);
  const float pedido = grados <= 0.0f
      ? U0_US + grados * (350.0f / 14.8f)
      : U0_US + grados * (700.0f / 22.3f);
  return (int)lroundf(constrain(pedido, UMIN_US, UMAX_US));
}

float controlar(float error) {
  float senal = error;
  for (int i = 0; i < N_ETAPAS; i++) senal = procesarEtapa(etapas[i], senal);

  // La cascada entrega grados de mando; luego se convierten a microsegundos.
  float mando_sat = constrain(senal, MANDO_MIN_DEG, MANDO_MAX_DEG);

  // Tracking en la salida: las memorias de etapas distintas tienen escalas
  // distintas. Esto no protege integradores en etapas anteriores.
  // Corregir en grados, las mismas unidades que la salida de la cascada.
  float exceso = senal - mando_sat;
  if (exceso != 0) {
    etapas[N_ETAPAS - 1].w -= KAW * exceso;
  }
  return anguloAPulso(mando_sat);
}

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


void enviarRegistro() {
  // 16 bytes, sin texto ni terminador; mismo protocolo que la IMU.
  if (Serial.availableForWrite() < 16) return;
  const float valores[] = {REFERENCIA_DEG, angulo, pulso};
  Serial.write("abcd", 4);
  Serial.write(reinterpret_cast<const byte *>(valores), sizeof(valores));
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  if (!mpu.begin(0x68)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  for (size_t i = 0; i < N_ETAPAS; ++i) {
    configurarEtapa(etapas[i], CONFIG_ETAPAS[i][0], CONFIG_ETAPAS[i][1],
                    CONFIG_ETAPAS[i][2], TS);
  }
  servo.attach(PIN_SERVO, 500, 2500);
  pulso = constrain(U0_US, UMIN_US, UMAX_US);
  servo.writeMicroseconds((int)lroundf(pulso));
  delay(1000); // Dejar asentar el servo antes de calibrar.
  sensors_event_t a, g, t;
  for (int i = 0; i < 500; ++i) {
    mpu.getEvent(&a, &g, &t);
    biasGyro += g.gyro.x;
    delay(3);
  }
  biasGyro /= 500;
  angulo = anguloAccel(a);
  ultimaIMU = micros();
  siguienteControl = ultimaIMU + TS_US;
}

void loop() {
  actualizarIMU();
  const unsigned long ahora = micros();
  if ((long)(ahora - siguienteControl) >= 0) {
    siguienteControl += TS_US;
    // Si hubo demora, no ejecutar varios pasos con una misma medicion.
    if ((long)(ahora - siguienteControl) >= 0) siguienteControl = ahora + TS_US;
    pulso = controlar(REFERENCIA_DEG - angulo);
    pulso = lroundf(pulso); // Registrar el pulso entero aplicado.
    servo.writeMicroseconds((int)pulso);
    enviarRegistro();
  }
}
