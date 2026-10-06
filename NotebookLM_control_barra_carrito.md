# Control de una barra y un carrito: códigos explicados

Este documento reúne tres sketches completos de Arduino y explica sus mediciones, cálculos, funciones y comunicaciones. Cada bloque de código es un programa independiente que se carga por separado en la placa.

El mecanismo utiliza un servo para inclinar una barra. Una MPU6050 fija a la barra permite estimar su inclinación mediante acelerómetro y giroscopio. Para controlar la posición del carrito se agrega un sensor ultrasónico HC-SR04. El pulso enviado al servo se expresa en microsegundos; el ángulo de la barra se expresa en grados y la distancia del carrito en centímetros. Son magnitudes diferentes.

Las bibliotecas Wire, Servo, Adafruit MPU6050 y Adafruit Unified Sensor permiten comunicarse con la IMU y accionar el servo. El programa del carrito también utiliza NewPing para el ultrasonido. Los tres programas usan el servo en el pin 9, la IMU por I2C en la dirección 0x68 y el puerto serie a 115200 baudios. MATLAB recibe mediciones; estos sketches no reciben consignas desde MATLAB.

## 1. Control de ángulo

### Qué hace

Mantiene la inclinación de la barra alrededor de una referencia definida en el código. La cadena de cálculo es: referencia angular → error respecto del ángulo medido → controlador → conversión a pulso → servo → movimiento de la barra → nueva medición de la IMU.

### Medición y filtro complementario

`anguloAccel()` estima la inclinación alrededor del eje X con la expresión `atan2(ay, sqrt(ax² + az²))`, convertida a grados, y resta `CERO_IMU`. El acelerómetro aporta una referencia basada en la gravedad. El giroscopio aporta velocidad angular; su lectura en radianes por segundo se convierte a grados por segundo y se le descuenta el sesgo medido al arrancar.

`actualizarIMU()` combina ambos sensores:

    ángulo[k] = alfa × (ángulo[k-1] + velocidad × dt)
                + (1 - alfa) × ángulo_acelerómetro[k]

Con `ALFA = 0.92`, la predicción integrada del giroscopio pesa 0.92 y la medida del acelerómetro pesa 0.08 en cada actualización. El intervalo mínimo de lectura es 10 ms, equivalente a una frecuencia nominal de 100 Hz. Se integra usando el tiempo real transcurrido. La calibración del giroscopio promedia 500 lecturas con la barra quieta. El filtro comienza desde el ángulo del acelerómetro.

### Controlador y discretización

El control se ejecuta nominalmente cada 20 ms, a 50 Hz. Calcula `error = REFERENCIA_DEG - angulo`. La referencia incluida es 10 grados y el código comprueba al compilar que pertenezca al intervalo de -10 a 10 grados.

`CONFIG_ETAPAS` describe factores continuos de la forma:

    C_i(s) = K × (s + cero) / (s + polo)

Las etapas se aplican sucesivamente y sus funciones de transferencia se multiplican. Esta cascada de factores matemáticos constituye el controlador angular. La fila `{KP, KI/KP, 0}` representa un PI: `C(s) = KP + KI/s`. Con los parámetros incluidos, `KP = 1` y `KI = 2`. Esa escritura requiere que KP sea distinto de cero.

`configurarEtapa()` aplica Tustin, sustituyendo `s = (2/Ts) × (1-z^-1)/(1+z^-1)`. Si `c = 2/Ts`, calcula:

    b0 = K × (c + cero) / (c + polo)
    b1 = K × (cero - c) / (c + polo)
    a1 = (polo - c) / (c + polo)

`procesarEtapa()` ejecuta el filtro discreto mediante una memoria `w`:

    y = b0 × x + w
    w = b1 × x - a1 × y

La memoria conserva la historia necesaria entre llamadas; el controlador no depende únicamente del error instantáneo.

### Saturación, anti-windup y servo

`controlar()` limita la salida a -14.8…22.3 grados de mando. Si el controlador pide más, corrige la memoria de la última etapa restándole `KAW` multiplicado por el exceso. Este seguimiento de saturación reduce la acumulación del integrador. La corrección se aplica solamente a la última etapa, por lo que el integrador debe ubicarse allí para recibirla.

`anguloAPulso()` convierte los grados de mando mediante una calibración por tramos:

    Para grados <= 0: pulso = 900 + grados × 350/14.8
    Para grados > 0:  pulso = 900 + grados × 700/22.3

El resultado se redondea a un entero y se limita a 550…1600 µs. Los grados usados en esta conversión corresponden a la barra. El pulso es el mando del actuador; la inclinación real se conoce por la IMU.

### Arranque, ejecución y datos enviados

`setup()` configura la IMU con acelerómetro de ±8 g, giroscopio de ±500 grados/s y filtro interno de 44 Hz; configura las etapas, comanda 900 µs, espera un segundo y calibra el giroscopio. Si no detecta la IMU, enciende el LED integrado y detiene el arranque.

`loop()` atiende las lecturas de la IMU y ejecuta el control cuando vence su período. Si se atrasa demasiado, reprograma el siguiente control para evitar una ráfaga de cálculos sobre la misma medición.

`enviarRegistro()` transmite una trama binaria de 16 bytes: los cuatro caracteres ASCII `abcd`, seguidos de tres valores float32 en este orden:

1. Referencia angular, en grados.
2. Ángulo filtrado, en grados.
3. Pulso aplicado al servo, en microsegundos.

No agrega separadores ni saltos de línea. Si no hay espacio para la trama en el búfer de transmisión, omite ese envío. El registro permite observar seguimiento de referencia, error y saturación del mando.

### Código completo: control_angulo.ino

```cpp
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

```

## 2. Control del carrito con registro para MATLAB

### Qué hace

Controla la distancia del carrito al sensor mediante dos lazos encadenados. El lazo externo compara la distancia deseada con la medida y pide una inclinación de barra. El lazo interno compara esa inclinación pedida con la medida por la IMU y calcula el pulso del servo.

    referencia de posición → controlador de posición → referencia angular
    referencia angular → controlador angular → servo → barra y carrito

El HC-SR04 usa TRIG en el pin 6 y ECHO en el pin 7, con alcance configurado de 40 cm. La referencia de distancia es 17 cm.

### Medición de posición y PID externo

`actualizarPosicion(dt)` consulta el sonar cuando transcurre `SONAR_US = 20000`, es decir, un intervalo mínimo de 20 ms. Ese valor gobierna la ejecución aunque el comentario del encabezado mencione 60 ms. Convierte el tiempo del eco a centímetros con `distancia = eco / 58.574`.

Si recibe un eco válido, calcula `error = REFERENCIA_CM - distancia`. La estructura del controlador es:

    integral_candidata = integral + ki × error × dt
    salida = kp × error + integral + kd × derivadaError

La integral usa una acumulación rectangular con el tiempo real del ciclo. Solo acepta la integral candidata si la salida queda dentro de los límites o si su incremento ayuda a salir de la saturación. Con `ki = 0` borra el aporte integral.

La derivada se calcula al recibir nuevas mediciones válidas mediante Tustin:

    d[k] = (2/dtMedicion) × (e[k] - e[k-1]) - d[k-1]

Se inicializa en cero en la primera lectura y al recuperar el eco. Es un derivador ideal, sin filtro adicional, cuya recurrencia puede sostener oscilaciones debidas al ruido. El código incluye el término derivativo, pero su contribución al mando depende de `kd`.

En el código incluido, `kp = ko = 7.5`, `ki = 0` y `kd = 0`: el cálculo aplicado a la posición es proporcional. `To` está declarado, pero no participa en las cuentas. La salida se limita a ±15 grados y se multiplica por `SIGNO_POSICION` para formar la referencia angular.

Si no hay eco, marca la distancia como inválida, reinicia derivada e integral y solicita ángulo cero. Un registro de distancia igual a -1 representa esta ausencia de medición.

### Estimación del ángulo y lazo interno

`actualizarAngulo()` lee la IMU y estima el ángulo con el filtro complementario, usando `ALFA = 0.8464`, el sesgo del giroscopio y el tiempo real entre actualizaciones. La lectura y el control angular se ejecutan una vez por ciclo nominal de 20 ms.

El lazo angular es proporcional:

    mando = PULSO_CENTRO
            + SIGNO_SERVO × KP_ANGULO × (referenciaAngulo - angulo)

`KP_ANGULO = 20` se expresa en µs/grado. El pulso se limita a 1000…2000 µs y se redondea antes de enviarlo al servo. `PULSO_CENTRO` se calcula a partir del comando nominal `ANGULO_SERVO_CERO + 90` y de los límites predeterminados de la biblioteca Servo. No se obtiene directamente de una medición de la barra.

`SIGNO_POSICION` representa cómo cambia la distancia ante una inclinación positiva; `SIGNO_SERVO`, cómo cambia el ángulo al aumentar el pulso. Ambos están definidos como +1. Su función es mantener el sentido apropiado de las realimentaciones según el montaje.

### Arranque y manejo de errores

`setup()` configura I2C a 400 kHz y un timeout de 3000 µs por espera de Wire. Configura la IMU con acelerómetro de ±2 g, giroscopio de ±500 grados/s y filtro de 44 Hz. Precarga el pulso central, habilita el servo, espera un segundo y promedia 500 lecturas del giroscopio para estimar el sesgo. Inicializa el ángulo desde el acelerómetro y prepara el sonar para medir en el primer ciclo.

`leerIMU()` comprueba el resultado de la lectura, el indicador de timeout de Wire y que los valores utilizados sean finitos. Ante una falla, incrementa un contador y enciende el LED. Si ocurre durante la calibración, detiene el arranque. Durante el control, se conserva el último pulso y se reinicia la integral de posición; se vuelve a intentar en el próximo ciclo.

### Temporización y comunicación

`loop()` espera la próxima ejecución, actualiza posición, actualiza ángulo y transmite. Usa `micros()` y mide el tiempo real para los cálculos. Si pierde un período o termina fuera de plazo, incrementa `periodosExcedidos`, enciende el LED y reprograma la ejecución sin intentar recuperar todos los ciclos atrasados.

`enviarRegistro()` verifica que queden al menos 2500 µs hasta el próximo control y que haya 16 bytes disponibles en el búfer serie. Si no se cumplen esas condiciones, omite la trama, incrementa `tramasOmitidas` y enciende el LED. La transmisión continúa en segundo plano sin `Serial.flush()`.

La trama binaria contiene `abcd` seguido de tres float32 little-endian:

1. Referencia de posición, en centímetros.
2. Distancia medida, en centímetros, o -1 si no hay eco.
3. Pulso aplicado al servo, en microsegundos.

El ángulo estimado, la referencia angular y los contadores de error son variables internas que no se incluyen en esa trama. El LED queda encendido tras detectar alguno de los eventos señalados; por sí solo no distingue cuál ocurrió.

### Código completo: control_carrito_matlab.ino

```cpp
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
const float ko = 7.5f; 
const float To = 0.0f; // segundos
const float kp = ko;            // grados / cm
const float ki = 0.0f;             // grados / (cm*s)
const float kd = 0.0f;            // grados*s / cm
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


```

## 3. Filtro complementario v5 con excitación del servo

### Qué hace

Aplica una secuencia periódica de pulsos al servo y registra la respuesta angular de la barra. Permite observar el acelerómetro, la integración del giroscopio y su combinación mediante un filtro complementario. También registra el pulso aplicado para disponer de una entrada y una salida con las que estudiar e identificar la dinámica del mecanismo.

La entrada se fija por una secuencia temporal; el ángulo medido no modifica el pulso. Por lo tanto, este ensayo funciona en lazo abierto respecto de la inclinación de la barra.

### Excitación y arranque

`setup()` configura la comunicación serie, I2C y el servo. El rango declarado en `attach()` es 500…2500 µs. Durante la calibración se aplica un pulso de 900 µs.

La IMU se configura con acelerómetro de ±8 g, giroscopio de ±500 grados/s y filtro interno de 44 Hz. Si no se detecta, el LED se enciende y el programa queda detenido. `calibrarGyro()` promedia 500 lecturas, separadas por una espera de 3 ms, para estimar el sesgo de la velocidad angular alrededor de X. El mecanismo debe permanecer quieto durante esa medición.

Finalizada la calibración, comienza la secuencia:

    1100 µs → 900 µs → 700 µs → 900 µs → repetir

Cada valor se mantiene durante 1500 ms; una secuencia completa dura nominalmente 6 segundos. Los cambios se programan con `millis()` para continuar leyendo la IMU entre cambios de pulso.

### Tres estimaciones del ángulo

La lectura se programa cada 10000 µs, a 100 Hz nominales, usando `micros()`. El tiempo efectivo entre muestras se convierte a segundos y se utiliza en la integración.

El ángulo del acelerómetro se obtiene como:

    anguloAccX = atan2(ay, sqrt(ax² + az²)) × 180/π

Esta estimación usa la dirección de la aceleración medida como referencia de inclinación; los movimientos que agregan aceleraciones pueden afectar la lectura.

Para el giroscopio se resta el sesgo y se convierten rad/s a grados/s:

    gyroX_dps = (gyroX - gyroX_offsetRadS) × 180/π
    anguloGyroX = anguloGyroX + gyroX_dps × dt

La integración sigue los cambios de orientación, aunque los errores residuales del sensor se acumulan con el tiempo. `anguloGyroX` comienza en cero, por lo que representa una integración relativa a esa inicialización.

El filtro complementario calcula:

    anguloFiltradoX = 0.96 × (anguloFiltradoX + gyroX_dps × dt)
                     + 0.04 × anguloAccX

La predicción por giroscopio aporta la evolución rápida y el acelerómetro corrige gradualmente la estimación. `anguloFiltradoX` también se inicializa en cero: si la barra parte inclinada puede aparecer un transitorio inicial mientras se aproxima a la referencia del acelerómetro.

### Muestreo y registro

`loop()` atiende primero los cambios del servo y después la adquisición. Si todavía no llegó el instante de muestreo, retorna. Si perdió instantes programados, avanza el próximo vencimiento hasta dejarlo en el futuro y realiza una sola lectura.

`TX_DECIMATION = 2` hace que se transmita una trama por cada dos muestras: con adquisición nominal de 100 Hz, el envío es de 50 Hz. El filtro sigue actualizándose en las muestras que no se transmiten.

`sendFloat()` envía los cuatro bytes de cada valor directamente. `sendBinaryFrame()` forma una trama de 20 bytes, con `abcd` y cuatro float32 en este orden:

1. Ángulo del filtro complementario, en grados.
2. Ángulo integrado del giroscopio, en grados.
3. Ángulo del acelerómetro, en grados.
4. Pulso aplicado al servo, en microsegundos.

No se envía texto, separadores ni salto de línea. La trama no incluye timestamp ni checksum. El receptor debe interpretar los bytes con el formato numérico de la placa; en AVR corresponde a float32 little-endian. Para la identificación, el pulso es la entrada y el ángulo elegido es la salida medida. El sketch adquiere esos datos; la estimación del modelo se realiza posteriormente fuera de Arduino.

### Código completo: filtro_complementario_mpu6050_v5_excitacion.ino

```cpp
/* Filtro complementario con MPU6050 y excitacion periodica del servo.
 *
 * Secuencia de entrada (en microsegundos):
 *   1100 -> 900 -> 700 -> 900 -> 1100 -> 900 -> ...
 *
 * Trama binaria para MATLAB (20 bytes):
 *   "abcd" + anguloFiltradoX + anguloGyroX + anguloAccX + pulsoServoUs
 *
 * Los cuatro datos son float de 32 bits. No se imprime texto por Serial.
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Servo miServo;

const uint8_t PIN_SERVO = 9;
const int PULSO_MIN_ATTACH_US = 500;
const int PULSO_MAX_ATTACH_US = 2500;
const uint32_t INTERVALO_EXCITACION_MS = 1500;

// Al repetirse, genera: 1100, 900, 700, 900, 1100, 900, ...
const uint16_t SECUENCIA_SERVO_US[] = {1100, 900, 700, 900};
const uint8_t CANTIDAD_PULSOS =
    sizeof(SECUENCIA_SERVO_US) / sizeof(SECUENCIA_SERVO_US[0]);

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SAMPLE_PERIOD_US = 10000; // Filtro a 100 Hz
const uint8_t TX_DECIMATION = 2;         // Envio a MATLAB a 50 Hz
const float ALPHA = 0.96;

float gyroX_offsetRadS = 0.0;
float anguloAccX = 0.0;
float anguloGyroX = 0.0;
float anguloFiltradoX = 0.0;
float pulsoServoUs = 1100.0;

uint32_t tiempoAnteriorUs = 0;
uint32_t proximaMuestraUs = 0;
uint32_t ultimoCambioServoMs = 0;
uint8_t indicePulso = 0;
uint8_t contadorTx = 0;

void sendFloat(float value) {
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  Serial.write(bytes, sizeof(value));
}

void sendBinaryFrame() {
  Serial.write("abcd", 4);
  sendFloat(anguloFiltradoX);
  sendFloat(anguloGyroX);
  sendFloat(anguloAccX);
  sendFloat(pulsoServoUs); // Entrada aplicada a la planta, en us
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  // Limites explicitos para evitar el recorte interno de Servo.
  miServo.attach(PIN_SERVO, PULSO_MIN_ATTACH_US, PULSO_MAX_ATTACH_US);
  // Mantener el actuador en el punto medio durante la calibracion.
  pulsoServoUs = 900.0;
  miServo.writeMicroseconds((int)pulsoServoUs);

  pinMode(LED_BUILTIN, OUTPUT);
  if (!mpu.begin(MPU_ADDR)) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Mantener el sistema quieto durante esta calibracion inicial.
  calibrarGyro();

  // La adquisicion comienza junto con la primera entrada de 1100 us.
  pulsoServoUs = (float)SECUENCIA_SERVO_US[indicePulso];
  miServo.writeMicroseconds((int)pulsoServoUs);

  const uint32_t ahoraUs = micros();
  tiempoAnteriorUs = ahoraUs;
  proximaMuestraUs = ahoraUs + SAMPLE_PERIOD_US;
  ultimoCambioServoMs = millis();
}

void loop() {
  // Cambiar la entrada sin bloquear el muestreo de la IMU.
  const uint32_t ahoraMs = millis();
  if ((uint32_t)(ahoraMs - ultimoCambioServoMs) >= INTERVALO_EXCITACION_MS) {
    ultimoCambioServoMs += INTERVALO_EXCITACION_MS;
    indicePulso = (indicePulso + 1) % CANTIDAD_PULSOS;
    pulsoServoUs = (float)SECUENCIA_SERVO_US[indicePulso];
    miServo.writeMicroseconds((int)pulsoServoUs);
  }

  const uint32_t ahoraUs = micros();
  if ((int32_t)(ahoraUs - proximaMuestraUs) < 0) return;
  proximaMuestraUs += SAMPLE_PERIOD_US;
  while ((int32_t)(ahoraUs - proximaMuestraUs) >= 0) {
    proximaMuestraUs += SAMPLE_PERIOD_US;
  }

  sensors_event_t accel, gyro, temperatura;
  mpu.getEvent(&accel, &gyro, &temperatura);

  const float dt = (ahoraUs - tiempoAnteriorUs) * 1.0e-6;
  tiempoAnteriorUs = ahoraUs;

  anguloAccX =
      atan2(accel.acceleration.y,
            sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) *
      180.0 / PI;

  // Adafruit entrega rad/s; el filtro integra grados/s.
  const float gyroX_dps =
      (gyro.gyro.x - gyroX_offsetRadS) * 180.0 / PI;
  anguloGyroX += gyroX_dps * dt;
  anguloFiltradoX =
      ALPHA * (anguloFiltradoX + gyroX_dps * dt) +
      (1.0 - ALPHA) * anguloAccX;

  if (++contadorTx >= TX_DECIMATION) {
    contadorTx = 0;
    sendBinaryFrame();
  }
}

void calibrarGyro() {
  float sumaXRadS = 0.0;
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    sensors_event_t accel, gyro, temperatura;
    mpu.getEvent(&accel, &gyro, &temperatura);
    sumaXRadS += gyro.gyro.x;
    delay(3);
  }

  gyroX_offsetRadS = sumaXRadS / muestras;
}

```

