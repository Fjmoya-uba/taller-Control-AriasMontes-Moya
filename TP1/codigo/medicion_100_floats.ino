void sendFloat(float value) {
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  Serial.write(bytes, sizeof(value));
}


void setup() {
  Serial.begin(115200);
}

void loop() {
  static unsigned long t0 = 0;
  static unsigned long t1 = 0;
  t0 = millis();
  for (float i = 0; i<100.0; i+=1.0) {
    if (int(i) % 16 == 0) { // por default el buffer puede almacenar hasta 64 bytes. Con el if se llama a flush para vaciar el buffer unicamente cuando este esta completamente lleno
      Serial.flush();
    }
    sendFloat(i);
  }
  Serial.flush(); // se vacia todo lo que quede en el buffer de transmision
  t1 = millis() - t0;
  Serial.println("");
  Serial.print("Tiempo de envio total: ");
  Serial.println(t1);
}
