/* Observador completo TP3, Tustin fisico, Ts=20 ms.
 * Misma secuencia, pines, cero de posicion y limites de dinamica-carrito TP2.
 * Sin control de posicion. Velocidad medida por diferencia de posicion
 * sobre una ventana movil de 5 intervalos (100 ms).
 * Mantener barra horizontal y sistema quieto durante calibracion inicial.
 * Regenerar coeficientes_observador_completo.h ejecutando el .m del TP3.
 * Retardo externo configurable abajo: 2 muestras heredadas de Practica7.
 * Trama: abcd + 8 float32 LE, 36 bytes, 115200 baud, sin texto:
 * theta_rad, theta_est_rad, omega_med_rad_s, omega_est_rad_s,
 * p_cm, p_est_cm, v_ventana_cm_s, v_est_cm_s.
 * p = distancia - 15.612 cm (mismo cero que TP2).
 * NaN: medicion/estimacion no disponible; velocidad NaN hasta llenar ventana.
 * Configurar receptor MATLAB para OCHO single (32 bytes de datos).
 * LED enclavado al detener.
 * Comando nuevo despues de medir; t indica inicio del ciclo (IMU/sonar
 * se adquieren unos ms despues). Horizontal no frena inmediatamente.
 * Reset para repetir. No se carga automaticamente a la placa.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>
#include "coeficientes_observador_completo.h"

const uint8_t PIN_SERVO = 9, PIN_TRIG = 6, PIN_ECHO = 7;
const int PULSO_CENTRO_US = 1544;
const uint32_t TS_US = 20000UL;
const uint32_t TIMEOUT_ECO_US = 4000UL;
// Mismo filtro complementario Tustin de Practica7, Ts=20 ms.
const float FILTRO_A = 0.9399814328194f;
const float FILTRO_ACC = 0.03000928359032f;
const float FILTRO_GYRO = 0.009699907164097f;
// Retardo externo heredado de P7
const uint8_t RETARDO_MUESTRAS = 2;
float historialU[RETARDO_MUESTRAS + 2] = {};
float xhat[4] = {}, yAnterior[2] = {};
float accAnterior = 0, gyroAnterior = 0, omegaGyro = 0;
bool filtroInicializado = false, observadorInicializado = false;
uint8_t fallo = 0; // bits: 1 sensor/limite, 2 plazo, 4 TX, 8 no finito
const float US_POR_CM = 58.574f;
const float MAX_DISTANCIA_CM = 40.0f;
// Limites operativos de TP2/Informe/main.tex; sin saturacion angular adicional.
const int PULSO_MIN_US = 1100, PULSO_MAX_US = 2000;
const float REFERENCIA_CM = 15.612f;
const uint8_t VENTANA_VELOCIDAD = 5; // Intervalos de 20 ms.
float historialPosicion[VENTANA_VELOCIDAD] = {};
uint8_t indicePosicion = 0, muestrasPosicion = 0;
float velocidadVentana = NAN;
const float POSICION_MIN_CM = -15.0f, POSICION_MAX_CM = 21.0f;
// Secuencia completa: 2695 muestras (53.9 s), incluido descanso final.
struct Escalon { int16_t deltaUs; uint16_t muestras; };
const Escalon SECUENCIA[] = {
  {0, 150},
  {100, 30}, {0, 150}, {-80, 30}, {0, 150},
  {120, 20}, {0, 150}, {-90, 25}, {0, 150},
  {115, 20}, {0, 150}, {-90, 30}, {0, 150},
  {105, 20}, {0, 150}, {-100, 20}, {0, 150},
  {85, 30}, {0, 150}, {-50, 30}, {0, 150},
  {95, 25}, {0, 150}, {-60, 30}, {0, 150},
  {0, 300}
};
const uint8_t N_ESCALONES = sizeof(SECUENCIA) / sizeof(SECUENCIA[0]);
static_assert(sizeof(float) == 4, "La trama requiere float32");
static_assert(TIMEOUT_ECO_US + 3000UL < TS_US, "Reservar tiempo para IMU/TX");

Servo miServo;
Adafruit_MPU6050 mpu;
float biasGyro = 0, anguloFiltrado = 0, offsetAngulo = 0;
int pulsoUs = PULSO_CENTRO_US;
uint32_t siguienteUs = 0;
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
  const float acc = anguloAcc(a);
  omegaGyro = g.gyro.x - biasGyro;
  const float gyroDps = omegaGyro * 180.0f / PI;
  if (!filtroInicializado) {
    anguloFiltrado = acc;
    accAnterior = acc;
    gyroAnterior = gyroDps;
    filtroInicializado = true;
    return;
  }
  anguloFiltrado = FILTRO_A*anguloFiltrado +
      FILTRO_ACC*(acc + accAnterior) + FILTRO_GYRO*(gyroDps + gyroAnterior);
  accAnterior = acc;
  gyroAnterior = gyroDps;
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

// Inicializar con las salidas actuales, sin derivar el sonar.
void actualizarObservador(float theta, float posicion, bool valida) {
  if (!valida) {
    observadorInicializado = false;
    return; // Nunca introducir el sentinel -1 del sonar como posicion.
  }
  if (!observadorInicializado) {
    xhat[0] = theta; xhat[1] = 0;
    xhat[2] = posicion; xhat[3] = 0;
    yAnterior[0] = theta; yAnterior[1] = posicion;
    observadorInicializado = true;
    return;
  }
  const float sumaU = historialU[RETARDO_MUESTRAS] +
                      historialU[RETARDO_MUESTRAS + 1];
  const float sumaY[2] = {theta + yAnterior[0], posicion + yAnterior[1]};
  float nuevo[4];
  for (uint8_t i = 0; i < 4; ++i) {
    nuevo[i] = OBS_BU[i]*sumaU;
    for (uint8_t j = 0; j < 4; ++j) nuevo[i] += OBS_AD[i][j]*xhat[j];
    for (uint8_t j = 0; j < 2; ++j) nuevo[i] += OBS_BY[i][j]*sumaY[j];
    if (!isfinite(nuevo[i])) {
      fallo |= 8; detener(); observadorInicializado = false; return;
    }
  }
  memcpy(xhat, nuevo, sizeof(xhat));
  yAnterior[0] = theta; yAnterior[1] = posicion;
}

// Diferencia entre extremos de una ventana movil; no deriva muestra a muestra.
void actualizarVelocidad(float posicion, bool valida) {
  velocidadVentana = NAN;
  if (!valida) {
    indicePosicion = 0;
    muestrasPosicion = 0;
    return;
  }
  if (muestrasPosicion == VENTANA_VELOCIDAD) {
    velocidadVentana = (posicion - historialPosicion[indicePosicion]) /
        (VENTANA_VELOCIDAD * TS_US * 1.0e-6f);
  } else {
    ++muestrasPosicion;
  }
  historialPosicion[indicePosicion] = posicion;
  indicePosicion = (indicePosicion + 1) % VENTANA_VELOCIDAD;
}

void enviarTrama(float theta, float posicion, bool valida) {
  if (Serial.availableForWrite() < 36 ||
      (int32_t)(siguienteUs - micros()) < 4000) {
    fallo |= 4;
    detener();
    return;
  }
  uint8_t trama[36] = {'a', 'b', 'c', 'd'};
  const float noValido = NAN;
  const float valores[8] = {
    isfinite(theta) ? theta : noValido,
    observadorInicializado ? xhat[0] : noValido,
    isfinite(omegaGyro) ? omegaGyro : noValido,
    observadorInicializado ? xhat[1] : noValido,
    valida ? posicion : noValido,
    observadorInicializado ? xhat[2] : noValido,
    velocidadVentana,
    observadorInicializado ? xhat[3] : noValido
  };
  for (uint8_t i = 0; i < 8; ++i) {
    uint32_t bits;
    memcpy(&bits, &valores[i], 4);
    for (uint8_t j = 0; j < 4; ++j)
      trama[4 + 4*i + j] = (uint8_t)(bits >> (8*j));
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
  if (fabs(OBS_TS - 0.02f) > 1e-6f) {
    fallo |= 2; detener(); while (true) {}
  }
}

void loop() {
  const uint32_t ahora = micros();
  if (!vencido(ahora, siguienteUs)) return;
  siguienteUs += TS_US; // Grilla absoluta, sin acumular tiempo de ejecucion.
  if (vencido(ahora, siguienteUs)) {
    fallo |= 2; detener();
    while (true) {} // No fabricar muestras consecutivas para recuperar tiempo.
  }
  actualizarAngulo();
  const float distancia = medirDistancia();
  const float posicion = distancia - REFERENCIA_CM;
  const float theta = (anguloFiltrado - offsetAngulo)*PI/180.0f;
  const bool valida = isfinite(distancia) && distancia >= 0 && isfinite(theta);
  if (!valida || posicion < POSICION_MIN_CM || posicion > POSICION_MAX_CM) {
    fallo |= 1; detener();
  }

  if (!detenido) {
    aplicarExcitacion();
    if (++cuentaEscalon >= SECUENCIA[escalon].muestras) {
      cuentaEscalon = 0;
      ++escalon;
      if (escalon >= N_ESCALONES) { detener(); }
    }
  }
  // Tras desplazar: historialU[j] = u[k-j]. El modelo usa u[k-2]+u[k-3].
  for (int i = RETARDO_MUESTRAS + 1; i > 0; --i)
    historialU[i] = historialU[i-1];
  historialU[0] = pulsoUs - PULSO_CENTRO_US;
  actualizarObservador(theta, posicion, valida);
  actualizarVelocidad(posicion, valida);
  enviarTrama(theta, posicion, valida);
  if (vencido(micros(), siguienteUs)) {
    fallo |= 2; detener();
    while (true) {} // Ensayo invalido por plazo perdido.
  }
}

