const int pinPot = A0;
const unsigned long INTERVALO_MICROS = 20000; // 20 ms = 50 Hz
unsigned long previousMicros = 0;
int valor = 0;

float convertirAngulo(int medicion) {
  float m = 180.0 / 1000.0;
  float b = 0;
  return m * medicion + b;
}

void setup() {
  Serial.begin(115200);
  //previousMicros = micros();
}

void loop() {
  /*
  unsigned long currentMicros = micros();

  // Se ejecuta exactamente cada 20.000 ms
  if (currentMicros - previousMicros >= INTERVALO_MICROS) {
    previousMicros += INTERVALO_MICROS; // Evita la deriva temporal acumulativa

    int rawADC = analogRead(pinPot);
    float angulo = convertirAngulo(rawADC);

    Serial.print("-------------------------------\n");
    Serial.print("Angulo: ");
    Serial.println(angulo, 2);

    Serial.print("Frecuencia: ");
    Serial.println(frecuencia);
    Serial.print("-------------------------------");

  }
  */
  int unsigned t0 = micros();
  valor = analogRead(pinPot);
  float angulo = 180.0/1024 * valor;
  Serial.println(angulo);
  int unsigned t3 = micros() - t0;

  delayMicroseconds(20000 - t3);

}
