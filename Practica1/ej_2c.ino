#include <NewPing.h>

# define N 5000
#define TRIGGER_PIN  6
#define ECHO_PIN     7
#define MAX_DISTANCE 200 // Distancia máxima a sensar en cm

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned long previousMillis = 0;
unsigned long sampleCount = 0;
const float US_POR_CM_IDA_VUELTA = 29.287 * 2.0; // ~58.574 µs/cm
unsigned int uS = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // ping() devuelve el tiempo total de ida y vuelta en microsegundos
  unsigned long t0 = micros();
  unsigned int uS = sonar.ping(); 
  float distancia_cm = uS / US_POR_CM_IDA_VUELTA;
  unsigned long t1 = micros() - t0;

  delayMicroseconds(20000 - t1);

  unsigned long t2 = micros();
  Serial.println(t2-t0);  // si realmente estamos midiendo a 50 Hz debería estar siempre alrededor de 20.000 us
  /*
  unsigned long start = micros();

  for (unsigned long i = 0; i < N; i++) {
     uS = sonar.ping(); 
    
  }

  unsigned long elapsed = micros() - start;
  float frecuencia = N * 1000000.0 / elapsed;*/

  float distancia_cm = uS / US_POR_CM_IDA_VUELTA;

  Serial.print("Distancia: ");
  Serial.print(distancia_cm);
  Serial.print(" | Frecuencia: ");
  Serial.print(frecuencia, 1);
  Serial.println(" Hz");
  
}
