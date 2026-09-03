const int pinPot = A0;
const unsigned long N = 5000; // cantidad fija de muestras
int valor = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long start = micros();

  for (unsigned long i = 0; i < N; i++) {
    valor = analogRead(pinPot);
  }

  unsigned long elapsed = micros() - start;
  float frecuencia = N * 1000000.0 / elapsed;

  Serial.print("Ultimo ADC: ");
  Serial.print(valor);
  Serial.print(" | Frecuencia: ");
  Serial.print(frecuencia, 1);
  Serial.println(" Hz");
}


#include <Servo.h>

const int pinPot = A0;
const int pinServo = 9;
Servo miServo;

unsigned long previousMicros = 0;
unsigned long INTERVALO_MICROS = 20000; // Cambiar según frecuencia: 50Hz=20000, 10Hz=100000, 1Hz=1000000

float convertirAngulo(int medicion) {
  float m = 180.0 / 1023.0;
  return m * medicion; // 0 a 180 grados
}

void setup() {
  Serial.begin(115200);
  miServo.attach(pinServo);
  previousMicros = micros();
}

void loop() {
    
    
    unsigned long t0 = micros();
    int rawADC = analogRead(pinPot);
    float angulo = convertirAngulo(rawADC);
    miServo.write(angulo);
    unsigned long t1 = micros() - t0;

    delayMicroseconds(INTERVALO_MICROS - t1)


    Serial.print("ADC: ");
    Serial.print(rawADC);
    Serial.print(" | Angulo: ");
    Serial.println(angulo, 2);
}