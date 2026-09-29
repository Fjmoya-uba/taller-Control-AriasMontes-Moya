#include <Servo.h>
#include <stdlib.h>

Servo miServo;
const int pinServo = 9;

const int ANGULO_CERO = 7;
const int ANGULO_MIN = -90;
const int ANGULO_MAX = 90;

char entrada[16];
byte longitud = 0;
bool descartarLinea = false;

// Conserva la referencia original: 7 equivale a Servo.write(97).
void moverServo(int angulo) {
  miServo.write(angulo + 90);
  Serial.print(F("Angulo comandado: "));
  Serial.print(angulo);
  Serial.print(F(" grados | Desplazamiento respecto del cero (7): "));
  Serial.print(angulo - ANGULO_CERO);
  Serial.println(F(" grados (nominales; medir el recorrido fisico)."));
}

void procesarEntrada() {
  byte inicio = (entrada[0] == '-' || entrada[0] == '+') ? 1 : 0;
  // Hasta dos digitos, con signo opcional; evita desbordamientos.
  if (longitud <= inicio || longitud - inicio > 2) {
    Serial.println(F("ERROR: envie un entero entre -90 y 90."));
    return;
  }
  for (byte i = inicio; i < longitud; ++i) {
    if (entrada[i] < '0' || entrada[i] > '9') {
      Serial.println(F("ERROR: envie solo un entero, por ejemplo 7, -10 o 20."));
      return;
    }
  }
  entrada[longitud] = '\0';
  const int angulo = (int)strtol(entrada, NULL, 10);
  if (angulo < ANGULO_MIN || angulo > ANGULO_MAX) {
    Serial.println(F("ERROR: angulo fuera del rango -90 a 90."));
    return;
  }
  moverServo(angulo);
}

void leerPuertoSerie() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (descartarLinea) {
        Serial.println(F("ERROR: linea demasiado larga; vuelva a ingresar el angulo."));
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
  miServo.write(ANGULO_CERO + 90);
  miServo.attach(pinServo);
  Serial.println(F("Monitor serie: 115200 baudios y Nueva linea o Ambos NL y CR."));
  Serial.println(F("Envie un angulo entero entre -90 y 90. El cero calibrado es 7."));
  Serial.println(F("Pruebe desde 7 en pasos de 1 grado hacia cada lado."));
  moverServo(ANGULO_CERO);
}

void loop() {
  leerPuertoSerie();
}
