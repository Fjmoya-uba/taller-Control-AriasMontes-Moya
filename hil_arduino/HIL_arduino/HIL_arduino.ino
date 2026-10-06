#include "TimerOne.h"

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// Variables de estado del control (conservan memoria entre iteraciones)
float u0 = 0.5;
float h_ref = 0.45;
float h = 0.45;
float u_anterior = 0.5;
float error_anterior = 0.0;
float error = 0.0;
float u = 0.5;

// Parámetros calculados por Tustin en MATLAB
const float alpha_1 = 1.0f;
const float alpha_2 = -143.878928f;
const float alpha_3 = 101.061654f;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // 1. Esperamos a que Simulink envíe la cabecera 'ABCD'
  if (waitForHeader()) {
    
    // 2. Leemos exactamente los 8 bytes que siguen (h y luego h_ref)
    h = readFloat();
    h_ref = readFloat();

    // 3. CÁLCULO DE CONTROL (se ejecuta solo cuando llegan datos nuevos)
    error = h_ref - h;
    u = alpha_1 * u_anterior + alpha_2 * error + alpha_3 * error_anterior;

    // Saturación de seguridad para actuador físico/PWM (ajustar límites si difieren)
    if (u > 1.0f) u = 1.0f;
    if (u < 0.0f) u = 0.0f;

    // Actualización de memoria para el siguiente paso (k-1)
    error_anterior = error;
    u_anterior = u;

    // 4. Respondemos inmediatamente a Simulink con la cabecera 'abcd' y las variables
    matlab_send(u, h, u0);
  }
  // Notar que NO hay delay(): Simulink marca el tiempo de muestreo (1 seg).
}

// Busca sincronizar la trama reconociendo 'A', 'B', 'C', 'D' consecutivas
bool waitForHeader() {
  static uint8_t headerIndex = 0;
  const uint8_t targetHeader[4] = {'A', 'B', 'C', 'D'};

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    if (b == targetHeader[headerIndex]) {
      headerIndex++;
      if (headerIndex == 4) {
        headerIndex = 0; // Trama encontrada
        return true;
      }
    } else {
      // Si no coincide, reinicia la búsqueda (o chequea si coincide con el inicio 'A')
      headerIndex = (b == targetHeader[0]) ? 1 : 0;
    }
  }
  return false;
}

// Lee 4 bytes bloqueantes para asegurar el float completo
float readFloat() {
  FLOATUNION_t f;
  for (int cont = 0; cont < 4; cont++) {
    while (Serial.available() == 0); // Espera activa muy breve por cada byte del float
    f.bytes[cont] = Serial.read();
  }
  return f.number;
}

void matlab_send(float u_val, float h_val, float u0_val) {
  Serial.write("abcd"); // Cabecera que espera el bloque Serial Receive de Simulink
  
  byte *b = (byte *)&u_val;
  Serial.write(b, 4);
  
  b = (byte *)&h_val;
  Serial.write(b, 4);
  
  b = (byte *)&u0_val;
  Serial.write(b, 4);
}