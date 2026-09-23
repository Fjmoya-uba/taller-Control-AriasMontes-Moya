// inicializo variables
int sensorPin = A0;
int sensorValue = 0;
unsigned long t0 = 0;
unsigned long dif = 0;

void setup() {
  Serial.begin(115200);
}


void loop() {
  t0 = micros();
  sensorValue = analogRead(sensorPin);
  float angulo = sensorValue * 180/1023;
  dif = micros() - t0;
  Serial.println("");
  Serial.print("Tiempo de medicion total: ");
  Serial.println(dif);
  delayMicroseconds(20000 - micros());
}