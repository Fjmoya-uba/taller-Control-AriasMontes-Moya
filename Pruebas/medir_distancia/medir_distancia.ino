// Distancia del carrito: TRIG 6, ECHO 7, monitor serie 115200.
// Mide a 50 Hz; imprime una muestra nueva solo al recibir 'a'.
#include <Servo.h>

const byte PIN_TRIG = 6;
const byte PIN_ECHO = 7;
const byte PIN_SERVO = 9;
const int PULSO_SERVO_US = 1544;
const unsigned long PERIODO_US = 20000UL;
const unsigned long TIMEOUT_US = 5000UL;
const float US_POR_CM = 58.574f; // Misma conversion que en el proyecto.
unsigned long proximaMuestraUs, anteriorUs;
bool referenciaLista = false;
bool calibrando = false;
bool imprimirPendiente = false;
float referenciaCm = 0, sumaCm = 0;
unsigned int validas = 0, intentos = 0;

Servo miServo;

unsigned long medirEco() {
  if (digitalRead(PIN_ECHO) == HIGH) return 0;
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, TIMEOUT_US);
}

void setup() {
  miServo.writeMicroseconds(PULSO_SERVO_US);
  miServo.attach(PIN_SERVO);
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  Serial.println(F("# Alinear la cara elegida con la linea de la barra; mantener quieto y enviar c."));
  Serial.println(F("# c: promediar 100 intentos (2 s) para fijar p=0. No se guarda al apagar."));
  Serial.println(F("# Servo en pin 9: comando fijo de 1544 us."));
  Serial.println(F("# a: imprimir la siguiente medicion. Adquisicion continua a 50 Hz, sin filtro."));
  Serial.println(F("# p = d - d0; positivo al alejarse del sensor."));
  anteriorUs = micros();
  proximaMuestraUs = anteriorUs + PERIODO_US;
}

void loop() {
  if (Serial.available() > 0) {
    const char comando = Serial.read();
    if (comando == 'a') imprimirPendiente = true;
    if (comando == 'c') {
      calibrando = true;
      referenciaLista = false;
      sumaCm = 0;
      validas = intentos = 0;
      Serial.println(F("# Calibrando referencia: mantener carrito quieto durante 2 s."));
    }
  }
  const unsigned long ahora = micros();
  if ((long)(ahora - proximaMuestraUs) < 0) return;
  proximaMuestraUs += PERIODO_US;
  // Evitar disparos consecutivos para recuperar muestras atrasadas.
  if ((long)(ahora - proximaMuestraUs) >= 0)
    proximaMuestraUs = ahora + PERIODO_US;
  const unsigned long dt = ahora - anteriorUs;
  anteriorUs = ahora;
  const unsigned long eco = medirEco();
  const bool valida = eco != 0;
  const float distanciaCm = eco / US_POR_CM;
  if (calibrando) {
    ++intentos;
    if (valida) { sumaCm += distanciaCm; ++validas; }
    if (intentos == 100) {
      calibrando = false;
      if (validas >= 95) {
        referenciaCm = sumaCm / validas;
        referenciaLista = true;
        Serial.print(F("# d0_cm=")); Serial.print(referenciaCm, 3);
        Serial.print(F("; muestras_validas=")); Serial.println(validas);
      } else Serial.println(F("# Referencia rechazada: menos de 95/100 ecos. Repetir con c."));
    }
  }
  if (!imprimirPendiente) return;
  imprimirPendiente = false;
  Serial.println(F("--- Medicion solicitada ---"));
  Serial.print(F("Tiempo desde arranque [ms]: ")); Serial.println(ahora / 1000UL);
  Serial.print(F("Intervalo entre mediciones [us] (objetivo 20000): ")); Serial.println(dt);
  Serial.print(F("Comando fijo al servo [us]: ")); Serial.println(PULSO_SERVO_US);
  Serial.print(F("Duracion del eco [us] (0 = sin eco): ")); Serial.println(eco);
  Serial.print(F("Eco recibido (no garantiza distancia correcta): "));
  Serial.println(valida ? F("SI") : F("NO"));
  Serial.print(F("Distancia sensor-cara del carrito d [cm]: "));
  if (valida) Serial.println(distanciaCm, 3);
  else Serial.println(F("no disponible"));
  Serial.print(F("Distancia de referencia d0 [cm]: "));
  if (referenciaLista) Serial.println(referenciaCm, 3);
  else Serial.println(F("sin calibrar; alinear carrito y enviar c"));
  Serial.print(F("Posicion relativa p = d - d0 [cm]: "));
  if (valida && referenciaLista) Serial.println(distanciaCm - referenciaCm, 3);
  else Serial.println(F("no disponible"));
}
