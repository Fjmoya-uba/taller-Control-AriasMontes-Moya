const int pinPot = A0;
unsigned long previousMillis = 0;
unsigned long sampleCount = 0;
int valor = 0;

float convertirAngulo(int medicion) {
  // la función devuelve el ángulo en grados
  float m = 180.0/1024.0;
  //float b = -18.0 / 5.0;
  return m * medicion;

}

void setup() {
  Serial.begin(115200);

}

void loop() {
  valor = analogRead(pinPot);
  float angulo = convertirAngulo(valor);

  Serial.print("ADC: ");
  Serial.print(valor);
  Serial.print(" | Angulo: ");
  Serial.print(angulo, 1);
  Serial.println(" grados");

  /*sampleCount++;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    float frecuencia = sampleCount * 1000.0 / (currentMillis - previousMillis);

    Serial.print("ADC: ");
    Serial.print(valor);
    Serial.print(" | Frecuencia: ");
    Serial.print(frecuencia, 1);
    Serial.println(" Hz");

    sampleCount = 0;
    previousMillis = currentMillis;

  } */

}
