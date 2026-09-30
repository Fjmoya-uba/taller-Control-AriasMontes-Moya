/* PID de posicion del carrito + control P del angulo de la barra.
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
const float ko = 10.0f; 
const float To = 1.34f; // segundos
const float kp = ko * 0.45;            // grados / cm
const float ki = 0; //0.45*(ko/To);            // grados / (cm*s)
const float kd = 0.2;            // grados*s / cm
const float MAX_ANGULO = 15.0f;    // Limite de inclinacion de la barra.
const float KP_ANGULO = 20.0f;    // us / grado, ajustar primero este lazo.
// Ganancias iniciales de prueba, no sintonizadas para el mecanismo.
// Calibracion de mover_servo.ino: comandos nominales, no angulos de barra.
const int ANGULO_SERVO_CERO = 7;
const int ANGULO_SERVO_MIN = -35; // -42 grados nominales respecto del cero.
const int ANGULO_SERVO_MAX = 81;  // +74 grados nominales respecto del cero.
// Reproduce Servo.write(comando + 90) con el attach() del ensayo.
const int PULSO_CENTRO = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_CERO + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const int PULSO_MIN = MIN_PULSE_WIDTH +
    (long)(ANGULO_SERVO_MIN + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
//const int PULSO_MAX = MIN_PULSE_WIDTH +
    //(long)(ANGULO_SERVO_MAX + 90) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180;
const int PULSO_MAX = 2000;
const float CERO_IMU = 0.0f;      // Angulo del acelerometro con barra horizontal.
// +1 si angulo positivo hace AUMENTAR distancia; -1 si la hace disminuir.
const float SIGNO_POSICION = 1.0f;
// +1 si aumentar pulso AUMENTA angulo medido; -1 si lo disminuye.
const float SIGNO_SERVO = 1.0f;
const float ALFA = 0.8464f; // 0.92^2: conserva aproximadamente la constante a 50 Hz.
const unsigned long TS_US = 20000UL;
const unsigned long SONAR_US = 20000UL;
// 16 bytes * 10 bits / 115200 = 1389 us; margen para llamadas e interrupciones.
const unsigned long RESERVA_TX_US = 2500UL;
static_assert(sizeof(float) == 4, "La trama requiere float32");
unsigned long siguienteControl, ultimoControl;
unsigned long periodosExcedidos = 0, tramasOmitidas = 0;
unsigned long erroresIMU = 0;
float derivadaError = 0; // cm/s; conserva d[k-1] entre mediciones.
float errorAnterior = 0; // Error de la ultima medicion valida.

Servo servo;
Adafruit_MPU6050 mpu;
NewPing sonar(6, 7, 40);          // Hasta 40 cm.
float angulo = 0, biasGyro = 0;
float distancia = 0;
float referenciaAngulo = 0, integral = 0;
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

void actualizarAngulo() {
  unsigned long ahora = micros();
  // Se llama una sola vez por periodo de control.
  float dt = (ahora - ultimaIMU) * 1e-6f;
  ultimaIMU = ahora;
  sensors_event_t a, g, t;
  if (!leerIMU(a, g, t)) {
    integral = 0;
    // Conservar el ultimo mando y reintentar en el siguiente periodo.
    return;
  }
  float velocidad = (g.gyro.x - biasGyro) * 180.0f / PI;
  angulo = ALFA * (angulo + velocidad * dt) + (1.0f - ALFA) * anguloAccel(a);

  // El PID de posicion pide grados; este lazo entrega microsegundos.
  float mando = PULSO_CENTRO + SIGNO_SERVO * KP_ANGULO * (referenciaAngulo - angulo);
  pulso = (int)lroundf(constrain(mando, (float)PULSO_MIN, (float)PULSO_MAX));
  servo.writeMicroseconds(pulso);
}


void actualizarPosicion(float dt) {

  const unsigned long ahora = micros();

  if (ahora - ultimaPosicion >= SONAR_US) {

    const float dtMedicion = (ahora - ultimaPosicion) * 1e-6f;

    ultimaPosicion = ahora;

    const unsigned int eco = sonar.ping(); // Timeout limitado por alcance de 40 cm.

    if (eco == 0) {

      distanciaValida = false;

      derivadaError = 0; // Reiniciar memoria si se pierde el eco.

    } else {

      distancia = eco / 58.574f;

      const float errorMedicion = REFERENCIA_CM - distancia;

     

      if (distanciaValida) {

        // Tustin: s = (2/T)*(1-z^-1)/(1+z^-1).

        // d[k] = (2/T)*(e[k]-e[k-1]) - d[k-1].

        // Derivador ideal: puede mantener oscilaciones por ruido (polo z=-1).

        derivadaError = (2.0f / dtMedicion) *

                       (errorMedicion - errorAnterior) - derivadaError;

      } else {

        // Primera lectura o recuperacion del eco: iniciar sin salto derivativo.

        derivadaError = 0;

      }

     

      errorAnterior = errorMedicion;

      distanciaValida = true;

    }

  }

  if (!distanciaValida) {
    referenciaAngulo = 0;
    integral = 0;
    return;
  }
  // PID a 50 Hz con la ultima medicion y derivada disponibles.
  const float error = REFERENCIA_CM - distancia;
  const float nuevaIntegral = ki == 0 ? 0 : integral + ki * error * dt;
  float salida = kp * error + nuevaIntegral + kd * derivadaError;
  if (ki == 0 || (salida >= -MAX_ANGULO && salida <= MAX_ANGULO) ||
      (salida > MAX_ANGULO && ki * error < 0) ||
      (salida < -MAX_ANGULO && ki * error > 0)) {
    integral = nuevaIntegral;
  }
  salida = kp * error + integral + kd * derivadaError;
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
  ultimoControl = ultimaIMU;
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
  const float dt = (ahora - ultimoControl) * 1e-6f;
  ultimoControl = ahora;
  actualizarPosicion(dt);
  actualizarAngulo();
  enviarRegistro(siguienteControl);
  if ((long)(micros() - siguienteControl) >= 0) {
    ++periodosExcedidos;
    digitalWrite(LED_BUILTIN, HIGH);
    siguienteControl = micros() + TS_US;
  }
}

