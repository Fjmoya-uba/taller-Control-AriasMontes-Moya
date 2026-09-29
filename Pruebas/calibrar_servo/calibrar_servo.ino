#include <Servo.h>
#include <stdlib.h>

Servo servo;
const byte PIN_SERVO = 9;
const int PULSO_MIN_US = 1000;
const int PULSO_CENTRO_US = 1500;
const int PULSO_MAX_US = 2000;

char entrada[16];
byte longitud = 0;
bool descartarLinea = false;

// Rechaza valores fuera del rango sin modificar la posicion comandada.
bool aplicarPulso(long pulsoUs) {
  if (pulsoUs < PULSO_MIN_US || pulsoUs > PULSO_MAX_US) {
    Serial.println(F("ERROR: ingrese un entero entre 1000 y 2000 us."));
    return false;
  }
  servo.writeMicroseconds((int)pulsoUs);
  Serial.print(F("Pulso comandado: "));
  Serial.print(pulsoUs);
  Serial.println(F(" us (angulo fisico: medir)."));
  return true;
}

void procesarEntrada() {
  // Solo digitos: evita aceptar comandos parciales o texto como un pulso.
  for (byte i = 0; i < longitud; ++i) {
    if (entrada[i] < '0' || entrada[i] > '9') {
      Serial.println(F("ERROR: envie solo el numero, por ejemplo 1500."));
      return;
    }
  }
  // Los pulsos admitidos tienen cuatro digitos; evita desbordar strtol.
  if (longitud != 4) {
    Serial.println(F("ERROR: ingrese un entero entre 1000 y 2000 us."));
    return;
  }
  entrada[longitud] = '\0';
  aplicarPulso(strtol(entrada, NULL, 10));
}

void leerPuertoSerie() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (descartarLinea) {
        Serial.println(F("ERROR: linea demasiado larga; vuelva a ingresar el pulso."));
      } else if (longitud > 0) {
        procesarEntrada();
      }
      longitud = 0;
      descartarLinea = false;
    } else if (!descartarLinea) {
      if (longitud < sizeof(entrada) - 1) {
        entrada[longitud++] = c;
      } else {
        descartarLinea = true;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Precarga el centro antes de habilitar la salida.
  servo.writeMicroseconds(PULSO_CENTRO_US);
  servo.attach(PIN_SERVO, PULSO_MIN_US, PULSO_MAX_US);
  Serial.println(F("Calibracion: servo en pin 9, rango 1000-2000 us."));
  Serial.println(F("Monitor serie: 115200 baudios y Nueva linea o Ambos NL y CR."));
  Serial.println(F("Envie un pulso (ej. 1500). Empiece con pasos de 25 us."));
  aplicarPulso(PULSO_CENTRO_US);
}

void loop() {
  leerPuertoSerie();
}
