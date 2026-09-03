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
