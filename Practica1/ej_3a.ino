#include <Servo.h>

Servo miServo;
const int pinServo = 9;

void setup() {
  Serial.begin(115200);
  miServo.attach(pinServo); // Por defecto: 50 Hz, pulso entre 544 y 2400 µs
}

void loop() {
  // Pulso de 1 ms (1000 µs) -> posición extrema (ej. 0°)
  miServo.writeMicroseconds(1000);
  Serial.println("Pulso: 1000 us");
  delay(2000);

  // Pulso de 1.5 ms (1500 µs) -> posición central (ej. 90°)
  miServo.writeMicroseconds(1500);
  Serial.println("Pulso: 1500 us");
  delay(2000);

  // Pulso de 2 ms (2000 µs) -> posición extrema (ej. 180°)
  miServo.writeMicroseconds(2000);
  Serial.println("Pulso: 2000 us");
  delay(2000);
}