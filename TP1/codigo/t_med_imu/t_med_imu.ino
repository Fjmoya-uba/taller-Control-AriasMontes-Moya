/* Ensayo estatico MPU6050: resolucion angular y tiempo de medicion.
 * Abrir este sketch solo (no junto a t_med_pote.ino).
 * Monitor serie: 115200 baud. Enviar 'm' para medir; mantener la barra fija.
 * Resolucion practica estimada: 3*sigma del angulo en reposo.
 * Es un criterio basado en ruido, no una verificacion de exactitud.
 * Tiempo de medicion: lectura I2C + calculo del filtro complementario;
 * excluye esperas, calibracion e impresion y no mide la latencia del sensor.
 */
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;
const uint8_t MPU_ADDR = 0x68;
const uint32_t I2C_CLOCK_HZ = 400000;
const mpu6050_accel_range_t RANGO = MPU6050_RANGE_2_G;
const float ALPHA = 0.92; // Igual que v5, a 100 Hz.
const uint16_t CALENTAMIENTO = 300; // 3 s de filtro antes de las estadisticas.
const uint16_t N = 2000;
const uint32_t PERIODO_US = 10000; // Lectura por software a 100 Hz.

// Welford: no guarda las 2000 muestras en la RAM del Nano.
struct Estadistica {
  uint16_t n = 0;
  float media = 0, m2 = 0, minimo = 0, maximo = 0;
  void agregar(float x) {
    if (n == 0) minimo = maximo = x;
    if (x < minimo) minimo = x;
    if (x > maximo) maximo = x;
    ++n;
    const float delta = x - media;
    media += delta / n;
    m2 += delta * (x - media);
  }
  float sigma() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0; }
};

// volatile hace observable el resultado antes de tomar el tiempo final.
volatile float anguloMedido = 0;

bool lecturaValida(const sensors_event_t &a, const sensors_event_t &g) {
  const float norma2 = a.acceleration.x * a.acceleration.x +
                       a.acceleration.y * a.acceleration.y +
                       a.acceleration.z * a.acceleration.z;
  bool valida = isfinite(norma2) && norma2 >= 64.0 && norma2 <= 144.0 &&
                isfinite(g.gyro.x);
#if defined(WIRE_HAS_TIMEOUT)
  valida = valida && !Wire.getWireTimeoutFlag();
#endif
  return valida;
}

bool calibrarGyro(float &offset) {
  float suma = 0;
  // Mismo promedio de 500 lecturas y pausa de 3 ms que v5.
  for (uint16_t i = 0; i < 500; ++i) {
    sensors_event_t a, g, temperatura;
#if defined(WIRE_HAS_TIMEOUT)
    Wire.clearWireTimeoutFlag();
#endif
    if (!mpu.getEvent(&a, &g, &temperatura) || !lecturaValida(a, g))
      return false;
    suma += g.gyro.x;
    delay(3);
  }
  offset = suma / 500;
  return true;
}

void medir() {
  Estadistica angulos, total;
  uint32_t omitidos = 0;
  Serial.println(F("Mantener quieta la IMU: espera, calibracion y ensayo (~28 s)."));
  Serial.flush();
  delay(3000);
  float gyroX_offsetRadS = 0;
  if (!calibrarGyro(gyroX_offsetRadS)) {
    Serial.println(F("Calibracion invalida. Revisar conexion y reposo."));
    return;
  }
  anguloMedido = 0; // Reiniciar en cada ensayo, como al arrancar v5.
  // No se imprime ni transmite durante calentamiento y ensayo.
  const uint32_t inicio = micros();
  uint32_t tiempoAnteriorUs = inicio;
  uint32_t proxima = inicio + PERIODO_US;
  for (uint16_t i = 0; i < N + CALENTAMIENTO; ++i) {
    while ((int32_t)(micros() - proxima) < 0) {}
    proxima += PERIODO_US;
    sensors_event_t a, g, temperatura;
#if defined(WIRE_HAS_TIMEOUT)
    Wire.clearWireTimeoutFlag();
#endif
    const uint32_t t0 = micros();
    const bool ok = mpu.getEvent(&a, &g, &temperatura);
    const float dt = (t0 - tiempoAnteriorUs) * 1.0e-6;
    tiempoAnteriorUs = t0;
    const float anguloAccX = atan2(a.acceleration.y,
                        sqrt(a.acceleration.x * a.acceleration.x +
                             a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    const float gyroX_dps = (g.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
    anguloMedido = ALPHA * (anguloMedido + gyroX_dps * dt) +
                   (1.0 - ALPHA) * anguloAccX;
    const uint32_t t2 = micros();
    if (!ok || !lecturaValida(a, g) || !isfinite(anguloMedido)) {
      // Un dato malo afecta el estado recursivo: no seguir con estadisticas.
      Serial.println(F("Ensayo invalidado por lectura erronea. Enviar m para recalibrar."));
      return;
    }
    if (i >= CALENTAMIENTO) {
      angulos.agregar(anguloMedido);
      total.agregar(t2 - t0);
    }
    const uint32_t ahora = micros();
    while ((int32_t)(ahora - proxima) >= 0) {
      proxima += PERIODO_US;
      ++omitidos;
    }
  }
  Serial.println(F("\n--- RESULTADOS ---"));
  Serial.println(F("Acelerometro: +/-2 g | I2C: 400 kHz | Muestreo: 100 Hz"));
  Serial.print(F("Angulo medio [grados]: "));
  Serial.println(angulos.media, 6);
  Serial.print(F("Desviacion estandar [grados]: "));
  Serial.println(angulos.sigma(), 6);
  Serial.print(F("Ruido pico a pico [grados]: "));
  Serial.println(angulos.maximo - angulos.minimo, 6);
  Serial.print(F("Resolucion practica estimada (3*sigma) [grados]: "));
  Serial.println(3 * angulos.sigma(), 6);
  Serial.print(F("Tiempo medio de medicion (lectura + filtro) [us]: "));
  Serial.println(total.media, 2);
  Serial.print(F("Tiempo maximo de medicion [us]: "));
  Serial.println(total.maximo, 2);
  Serial.print(F("Tiempo minimo de medicion [us]: ")); 
  Serial.println(total.minimo, 2);                     
  if (omitidos)
    Serial.println(F("Aviso: no se sostuvo el muestreo de 100 Hz. Repetir ensayo."));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  if (!mpu.begin(MPU_ADDR)) {
    Serial.println(F("No se encuentra MPU6050. Revisar direccion y cableado."));
    while (true) delay(100);
  }
  // Despues de begin(): la inicializacion de la libreria puede reiniciar Wire.
  Wire.setClock(I2C_CLOCK_HZ);
  mpu.setAccelerometerRange(RANGO);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  mpu.setSampleRateDivisor(0);
  Serial.println(F("Ensayo estatico listo. Enviar m para medir (~28 s)."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'm') {
    medir();
    Serial.println(F("Enviar m para repetir."));
  }
}
