#include <Servo.h>

Servo miServo;
const int pinServo = 9;

// Convierte un ángulo en el rango [-90, 90] al rango [0, 180] que espera write()
int convertirAngulo(float anguloDeseado) {
  return anguloDeseado + 90;
}

void setup() {
  Serial.begin(115200);
  miServo.attach(pinServo);
}

void loop() {
  // -90°
  miServo.write(convertirAngulo(-90));
  Serial.println("Angulo: -90");
  delay(2000);

  // 0°
  miServo.write(convertirAngulo(0));
  Serial.println("Angulo: 0");
  delay(2000);

  // 90°
  miServo.write(convertirAngulo(90));
  Serial.println("Angulo: 90");
  delay(2000);
}
