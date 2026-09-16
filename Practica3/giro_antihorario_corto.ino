#include <Servo.h>

Servo miServo;

const byte PIN_SERVO = 9;

// Posición base de referencia
const int PULSO_INICIO_US = 500;

// Subir el ancho de pulso gira en sentido antihorario (aprox. 5 a 10 grados)
const int PULSO_FINAL_US = 1600;

void setup() {
  miServo.attach(9); // Pin donde está conectado el servo
  miServo.writeMicroseconds(PULSO_INICIO_US); // Posición inicial
}


void loop() {

  miServo.writeMicroseconds(PULSO_FINAL_US); // Mover a la posición final
  delay(1500);
  miServo.writeMicroseconds(PULSO_INICIO_US); // Volver a la posición inicial
  delay(1500);

}