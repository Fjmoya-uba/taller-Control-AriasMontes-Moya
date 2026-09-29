#include <Servo.h>

Servo miServo;
const int pinServo = 9;

// recibe un ángulo entre -90 y 90 y mueve el servo en dicho valor
void moverServo(float angulo) {
  miServo.write(angulo + 90);
}

void setup() {
  Serial.begin(115200);
  miServo.attach(pinServo);
  miServo.write(90);
}

void loop() {
  moverServo(-30);
  delay(2000);

  moverServo(0);
  delay(2000);

  moverServo(30);
  delay(2000);
}
