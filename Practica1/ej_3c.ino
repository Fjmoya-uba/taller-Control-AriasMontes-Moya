#include <Servo.h>

const int pinPot = A0;
const int pinServo = 9;
Servo miServo;

unsigned long INTERVALO_MICROS = 1000; // Cambiar según frecuencia: 50Hz=20000, 10Hz=100000, 1Hz=1000000

float convertirAngulo(int medicion) {
  float m = 180.0 / 1023.0;
  return m * medicion; // 0 a 180 grados
}

void setup() {
  Serial.begin(115200);
  miServo.attach(pinServo);

}

void loop() {
  unsigned long t0 = millis();

  int rawADC = analogRead(pinPot);
  float angulo = convertirAngulo(rawADC);
  miServo.write(angulo);
  
  unsigned long t1 = millis() - t0;

  delay(INTERVALO_MICROS - t1); // delay trabaja en ms

  Serial.print("ADC: ");
  Serial.print(rawADC);
  Serial.print(" | Angulo: ");
  Serial.println(angulo, 2);
}