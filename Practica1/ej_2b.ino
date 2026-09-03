#include <NewPing.h>

#define TRIGGER_PIN  6
#define ECHO_PIN     7
#define MAX_DISTANCE 200 // Distancia máxima a sensar en cm

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned long previousMillis = 0;
unsigned long sampleCount = 0;
const float US_POR_CM_IDA_VUELTA = 29.287 * 2.0; // ~58.574 µs/cm
unsigned int uS = 0;

float ultimaDistancia = 0.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Lectura del tiempo de ida y vuelta en microsegundos
  unsigned int uS = sonar.ping();
  
  if (uS > 0) {
    ultimaDistancia = uS / US_POR_CM_IDA_VUELTA;
  }
  
  sampleCount++;
  unsigned long currentMillis = millis();

  // Imprime la distancia y la frecuencia calculada cada 1 segundo
  if (currentMillis - previousMillis >= 1000) {
    float frecuencia = (sampleCount * 1000.0) / (currentMillis - previousMillis);

    Serial.print("Distancia: ");
    Serial.print(ultimaDistancia, 2);
    Serial.print(" cm | Frecuencia: ");
    Serial.print(frecuencia, 2);
    Serial.println(" Hz");

    sampleCount = 0;
    previousMillis = currentMillis;
  }
}
