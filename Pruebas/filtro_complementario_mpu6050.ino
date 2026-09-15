/*
  FILTRO COMPLEMENTARIO - Ángulo con MPU6050 (Acelerómetro + Giroscopo)
  ----------------------------------------------------------------------
  Combina las lecturas de acelerómetro y giroscopo para obtener un ángulo
  estable, sin ruido (del acelerómetro) y sin deriva/drift (del giroscopo).

  Conexión típica MPU6050 -> Arduino:
    VCC -> 5V (o 3.3V según tu módulo)
    GND -> GND
    SCL -> A5 (Uno/Nano) 
    SDA -> A4 (Uno/Nano)

  No requiere librerías externas: se comunica directo por I2C con Wire.h
*/

#include <Wire.h>

const byte MPU_ADDR = 0x68; // Dirección I2C del MPU6050 (AD0 a GND)

// Variables crudas del sensor
int16_t accX_raw, accY_raw, accZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

// Offset del giroscopo (para calibrar el error de reposo)
float gyroX_offset = 0;
float gyroY_offset = 0;

// Ángulos calculados
float anguloAccX = 0;
float anguloAccY = 0;
float anguloGyroX = 0;
float anguloGyroY = 0;

// Ángulo final combinado (filtro complementario)
float anguloFiltradoX = 0;
float anguloFiltradoY = 0;

// Constante del filtro: cuánto confiar en el giroscopo (0.90 a 0.98 típico)
// Más cerca de 1.0 = más peso al giroscopo (más suave, pero puede derivar más)
// Más cerca de 0.90 = más peso al acelerómetro (menos deriva, pero más ruido)
const float ALPHA = 0.96;

// Sensibilidades del MPU6050 en configuración por defecto
const float SENS_ACC = 16384.0;   // LSB/g para rango ±2g
const float SENS_GYRO = 131.0;    // LSB/(°/s) para rango ±250°/s

unsigned long tiempoAnterior;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Despertar al MPU6050 (por defecto arranca en modo sleep)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0);    // Poner en 0 = despertar el sensor
  Wire.endTransmission(true);

  Serial.println("Calibrando giroscopo, no mover el sensor...");
  calibrarGyro();
  Serial.println("Listo.");

  tiempoAnterior = millis();
}

void loop() {
  leerSensores();

  // dt = tiempo transcurrido desde la última lectura, en segundos
  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - tiempoAnterior) / 1000.0;
  tiempoAnterior = tiempoActual;

  // --- Ángulo desde el ACELERÓMETRO (usando trigonometría) ---
  // Válido cuando no hay mucha aceleración lineal (solo gravedad)
  anguloAccX = atan2(accY_raw, sqrt(pow(accX_raw, 2) + pow(accZ_raw, 2))) * 180.0 / PI; // -> atan2 devuelve en radianes

  // --- Velocidad angular del GIROSCOPO en grados/segundo ---
  float gyroX_dps = (gyroX_raw / SENS_GYRO) - gyroX_offset;
  

  // --- FILTRO COMPLEMENTARIO ---
  // Se integra el giroscopo (rápido, sin ruido) y se corrige con el
  // acelerómetro (preciso a largo plazo, sin deriva) según el peso ALPHA.
  anguloFiltradoX = ALPHA * (anguloFiltradoX + gyroX_dps * dt) + (1 - ALPHA) * anguloAccX;
  

  // --- Salida para comparar los 3 métodos ---
  Serial.print("AccX: ");
  Serial.print(anguloAccX);
  Serial.print("\tGyroX_int: ");
  Serial.print(anguloGyroX += gyroX_dps * dt); // solo giroscopo, para comparar el drift
  Serial.print("\tFiltradoX: ");
  Serial.println(anguloFiltradoX);

  delay(10); // ~100 Hz de muestreo, ajustable según necesidad
}

// Lee los 6 valores crudos (acc XYZ + gyro XYZ) del MPU6050
void leerSensores() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Registro de inicio: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  accX_raw = Wire.read() << 8 | Wire.read();
  accY_raw = Wire.read() << 8 | Wire.read();
  accZ_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Salteamos temperatura (2 bytes)
  gyroX_raw = Wire.read() << 8 | Wire.read();
  gyroY_raw = Wire.read() << 8 | Wire.read();
  gyroZ_raw = Wire.read() << 8 | Wire.read();
}

// Calcula el offset del giroscopo promediando lecturas en reposo
void calibrarGyro() {
  long sumaX = 0, sumaY = 0;
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    leerSensores();
    sumaX += gyroX_raw;
    sumaY += gyroY_raw;
    delay(3);
  }

  gyroX_offset = (sumaX / (float)muestras) / SENS_GYRO;
  gyroY_offset = (sumaY / (float)muestras) / SENS_GYRO;
}
