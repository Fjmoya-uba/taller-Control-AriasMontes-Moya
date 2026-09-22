
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;
const uint8_t MPU_ADDR = 0x68;
const uint32_t I2C_CLOCK_HZ = 400000;
const mpu6050_accel_range_t RANGO = MPU6050_RANGE_2_G;
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

bool lecturaValida(const sensors_event_t &a) {
  const float norma2 = a.acceleration.x * a.acceleration.x +
                       a.acceleration.y * a.acceleration.y +
                       a.acceleration.z * a.acceleration.z;
  bool valida = isfinite(norma2) && norma2 >= 64.0 && norma2 <= 144.0;
#if defined(WIRE_HAS_TIMEOUT)
  valida = valida && !Wire.getWireTimeoutFlag();
#endif
  return valida;
}

void medir() {
  Estadistica angulos, total;
  uint32_t omitidos = 0;
  Serial.println(F("Mantener quieta la IMU: espera y ensayo (~23 s)."));
  Serial.flush();
  delay(3000);
  // No se imprime ni transmite durante el ensayo.
  const uint32_t inicio = micros();
  uint32_t proxima = inicio + PERIODO_US;
  for (uint16_t i = 0; i < N; ++i) {
    while ((int32_t)(micros() - proxima) < 0) {}
    proxima += PERIODO_US;
    sensors_event_t a, g, temperatura;
#if defined(WIRE_HAS_TIMEOUT)
    Wire.clearWireTimeoutFlag();
#endif
    const uint32_t t0 = micros();
    const bool ok = mpu.getEvent(&a, &g, &temperatura);
    anguloMedido = atan2(a.acceleration.y,
                        sqrt(a.acceleration.x * a.acceleration.x +
                             a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    const uint32_t t2 = micros();
    if (!ok || !lecturaValida(a) || !isfinite(anguloMedido)) {
      Serial.println(F("Ensayo invalidado por lectura erronea. Enviar m para repetir."));
      return;
    }
    angulos.agregar(anguloMedido);
    total.agregar(t2 - t0);
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
  Serial.print(F("Tiempo medio de medicion (getEvent + angulo acel.) [us]: "));
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
  Serial.println(F("Ensayo estatico (solo angulo acel.) listo. Enviar m para medir (~23 s)."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'm') {
    medir();
    Serial.println(F("Enviar m para repetir."));
  }
}
