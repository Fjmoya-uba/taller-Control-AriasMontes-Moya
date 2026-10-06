/* V2: controlador discreto de posicion del carrito + control P del angulo de la barra.
 * HC-SR04: TRIG 6, ECHO 7. MPU6050 por I2C. Servo en 9.
 * Mantener la barra quieta al arrancar para calibrar el giroscopio.
 * MATLAB: "abcd" + referencia_cm + distancia_cm + servo_us (3 float32 LE).
 * Control y TX: 50 Hz. Sonar: una lectura cada 60 ms, retenida entre ecos.
 * distancia_cm = -1 indica falta de eco. No abrir el Monitor Serie.
 * LED encendido: fallo IMU, periodo excedido o trama omitida (queda latcheado).
 */
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// -------- Parametros para modificar --------
const float REFERENCIA_CM = 17.0f; // Distancia deseada desde el sensor: medir.
// MATLAB: Tustin con Ts = 0.02 s. Incluye el filtro de la derivada.
// u[k] = COEF_A*u[k-1] + COEF_B0*e[k] + COEF_B1*e[k-1].
const float COEF_A = 0.600000f;
const float COEF_B0 = 52.563947f;
const float COEF_B1 = -51.936942f;
const float MAX_ANGULO = 15.0f;    // Limite de inclinacion de la barra.
const float KP_ANGULO = 20.0f;    // us / grado, ajustar primero este lazo.
// Ajustar el lazo de angulo de acuerdo con la planta usada en MATLAB.
// Calibracion de mover_servo.ino: comandos nominales, no angulos de barra.
const int ANGULO_SERVO_CERO = 7;
const int ANGULO_SERVO_MIN = -35; // -42 grados nominales respecto del cero.
const int ANGULO_SERVO_MAX = 81;  // +74 grados nominales respecto del cero.
// Reproduce Servo.write(comando + 90) con el attach() del ensayo.
const int PULSO_CENTRO = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_CERO + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const int PULSO_MIN = 1000;
//const int PULSO_MAX = MIN_PULSE_WIDTH +
    //(long)(ANGULO_SERVO_MAX + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const int PULSO_MAX = 2000;
const float CERO_IMU = 0.0f;      // Angulo del acelerometro con barra horizontal.
// +1 si angulo positivo hace AUMENTAR distancia; -1 si la hace disminuir.
const float SIGNO_POSICION = 1.0f;
// +1 si aumentar pulso AUMENTA angulo medido; -1 si lo disminuye.
const float SIGNO_SERVO = 1.0f;
const float ALFA = 0.92f; //
const unsigned long TS_US = 20000UL; // Si cambia, recalcular COEF_A, COEF_B0 y COEF_B1 en MATLAB.
const unsigned long SONAR_US = 60000UL;
// 16 bytes * 10 bits / 115200 = 1389 us; margen para llamadas e interrupciones.
const unsigned long RESERVA_TX_US = 2500UL;
static_assert(sizeof(float) == 4, "La trama requiere float32");
unsigned long siguienteControl;
unsigned long periodosExcedidos = 0, tramasOmitidas = 0;
unsigned long erroresIMU = 0;
float errorAnteriorControl = 0;
float salidaAnteriorControl = 0;
bool controladorInicializado = false;

Servo servo;
Adafruit_MPU6050 mpu;
NewPing sonar(6, 7, 35);          // Hasta 35 cm.
float angulo = 0, biasGyro = 0;
float distancia = 0;
float referenciaAngulo = 0;
bool distanciaValida = false;
int pulso = PULSO_CENTRO;
unsigned long ultimaIMU, ultimaPosicion;



float anguloAccel(const sensors_event_t &a) {
  // Mismo eje y expresion que en control_angulo.ino.
  return atan2(a.acceleration.y,
               sqrt(sq(a.acceleration.x) + sq(a.acceleration.z))) *
         180.0f / PI - CERO_IMU;
}

bool leerIMU(sensors_event_t &a, sensors_event_t &g, sensors_event_t &t) {
  // Core AVR (Uno/Nano/Mega): evitar una espera infinita ante ruido en I2C.
  Wire.clearWireTimeoutFlag();
  const bool ok = mpu.getEvent(&a, &g, &t);
  // getEvent de Adafruit no propaga todos los errores de lectura de Wire.
  if (!ok || Wire.getWireTimeoutFlag() ||
      !isfinite(a.acceleration.x) || !isfinite(a.acceleration.y) ||
      !isfinite(a.acceleration.z) || !isfinite(g.gyro.x)) {
    ++erroresIMU;
    digitalWrite(LED_BUILTIN, HIGH);
    return false;
  }
  return true;
}

void reiniciarControlador() {
  errorAnteriorControl = 0;
  salidaAnteriorControl = 0;
  controladorInicializado = false;
  referenciaAngulo = 0;
}

void actualizarAngulo() {
  unsigned long ahora = micros();
  // Se llama una sola vez por periodo de control.
  float dt = (ahora - ultimaIMU) * 1e-6f;
  ultimaIMU = ahora;
  sensors_event_t a, g, t;
  if (!leerIMU(a, g, t)) {
    reiniciarControlador();
    // Conservar el ultimo mando y reintentar en el siguiente periodo.
    return;
  }
  float velocidad = (g.gyro.x - biasGyro) * 180.0f / PI;
  angulo = ALFA * (angulo + velocidad * dt) + (1.0f - ALFA) * anguloAccel(a);

  // El controlador de posicion pide grados; este lazo entrega microsegundos.
  float mando = PULSO_CENTRO + SIGNO_SERVO * KP_ANGULO * (referenciaAngulo - angulo);
  pulso = (int)lroundf(constrain(mando, (float)PULSO_MIN, (float)PULSO_MAX));
  servo.writeMicroseconds(pulso);
}


void actualizarPosicion() {
  const unsigned long ahora = micros();
  if (ahora - ultimaPosicion >= SONAR_US) {
    ultimaPosicion = ahora;
    const unsigned int eco = sonar.ping(); // Timeout limitado por alcance de 40 cm.
    distanciaValida = (eco != 0);
    if (distanciaValida) {
      distancia = eco / 58.574f;
    }
  }

  if (!distanciaValida) {
    reiniciarControlador();
    return;
  }

  // Ejecutar cada 20 ms, reteniendo la ultima posicion entre ecos.
  const float error = REFERENCIA_CM - distancia;
  if (!controladorInicializado) {
    // Arranque o recuperacion sin golpe derivativo: u = Kp*error.
    errorAnteriorControl = error;
    salidaAnteriorControl = ((COEF_B0 + COEF_B1) / (1.0f - COEF_A)) * error;
    controladorInicializado = true;
  }

  const float salida = COEF_A * salidaAnteriorControl
                     + COEF_B0 * error
                     + COEF_B1 * errorAnteriorControl;
  errorAnteriorControl = error;
  // Guardar SIN recortar para conservar la dinamica del filtro.
  salidaAnteriorControl = salida;
  referenciaAngulo = SIGNO_POSICION * constrain(salida, -MAX_ANGULO, MAX_ANGULO);
}

void enviarRegistro(unsigned long limite) {
  const long restante = (long)(limite - micros());
  if (restante < (long)RESERVA_TX_US || Serial.availableForWrite() < 16) {
    ++tramasOmitidas;
    digitalWrite(LED_BUILTIN, HIGH);
    return; // No iniciar una trama que no entra en el tiempo disponible.
  }
  const float valores[] = {REFERENCIA_CM, distanciaValida ? distancia : -1.0f,
                          (float)pulso};
  Serial.write("abcd", 4);
  Serial.write(reinterpret_cast<const byte *>(valores), sizeof(valores));
  // No usar Serial.flush(): la UART transmite en segundo plano.
}
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setWireTimeout(3000UL, true); // Por espera I2C; reinicia TWI al vencer.
  Wire.setClock(400000);
  if (!mpu.begin(0x68)) {
    digitalWrite(LED_BUILTIN, HIGH); // Error IMU: no mezclar texto con binario.
    while (true) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  pulso = constrain(PULSO_CENTRO, PULSO_MIN, PULSO_MAX);
  servo.writeMicroseconds(pulso);
  servo.attach(9); // Mismo rango predeterminado que mover_servo.ino.
  delay(1000);

  sensors_event_t a, g, t;
  for (int i = 0; i < 500; i++) {
    if (!leerIMU(a, g, t)) {
      // No arrancar el control con una calibracion incompleta.
      while (true) delay(100); // Corregir conexion y reiniciar.
    }
    biasGyro += g.gyro.x;
    delay(3);
  }
  biasGyro /= 500;
  angulo = anguloAccel(a);
  ultimaIMU = micros();
  ultimaPosicion = ultimaIMU - SONAR_US; // Medir en el primer ciclo.
  siguienteControl = ultimaIMU + TS_US;
}

void loop() {
  const unsigned long ahora = micros();
  if ((long)(ahora - siguienteControl) < 0) return;
  siguienteControl += TS_US;
  // Si ya se perdio un periodo, no ejecutar rafagas de controles atrasados.
  if ((long)(ahora - siguienteControl) >= 0) {
    ++periodosExcedidos;
    digitalWrite(LED_BUILTIN, HIGH);
    siguienteControl = ahora + TS_US;
  }
  actualizarPosicion();
  actualizarAngulo();
  enviarRegistro(siguienteControl);
  if ((long)(micros() - siguienteControl) >= 0) {
    ++periodosExcedidos;
    digitalWrite(LED_BUILTIN, HIGH);
    siguienteControl = micros() + TS_US;
  }
}
