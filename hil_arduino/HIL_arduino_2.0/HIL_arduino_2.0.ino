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

const unsigned long Ts_ms = 1000;   // Período de muestreo: 1 Hz
unsigned long proximoTick = 0;

void setup() {
  Serial.begin(115200);
  proximoTick = millis() + Ts_ms;
}

void loop() {
  // 1. Siempre atendemos el puerto serie sin bloquear:
  //    actualiza h y h_ref con la última trama completa recibida
  pollSerial();

  // 2. El control se ejecuta cada Ts_ms, lleguen o no datos nuevos
  unsigned long ahora = millis();
  if ((long)(ahora - proximoTick) >= 0) {
    proximoTick += Ts_ms;   // suma fija: evita que se acumule deriva

    error = h_ref - h;
    u = alpha_1 * u_anterior + alpha_2 * error + alpha_3 * error_anterior;

    if (u > 1.0f) u = 1.0f;
    if (u < 0.0f) u = 0.0f;

    error_anterior = error;
    u_anterior = u;

    matlab_send(u, h, u0);
  }
}

// Parser no bloqueante: máquina de estados 'ABCD' + 8 bytes (h, h_ref)
void pollSerial() {
  static uint8_t estado = 0;          // 0: buscando cabecera, 1: leyendo payload
  static uint8_t headerIndex = 0;
  static uint8_t payload[8];
  static uint8_t payloadIndex = 0;
  const uint8_t targetHeader[4] = {'A', 'B', 'C', 'D'};

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (estado == 0) {
      if (b == targetHeader[headerIndex]) {
        if (++headerIndex == 4) {
          headerIndex = 0;
          payloadIndex = 0;
          estado = 1;
        }
      } else {
        headerIndex = (b == targetHeader[0]) ? 1 : 0;
      }
    } else {
      payload[payloadIndex++] = b;
      if (payloadIndex == 8) {
        FLOATUNION_t f;
        memcpy(f.bytes, payload, 4);     h     = f.number;
        memcpy(f.bytes, payload + 4, 4); h_ref = f.number;
        estado = 0;
      }
    }
  }
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

// Parser no bloqueante: máquina de estados 'ABCD' + 8 bytes (h, h_ref)
void pollSerial() {
  static uint8_t estado = 0;          // 0: buscando cabecera, 1: leyendo payload
  static uint8_t headerIndex = 0;
  static uint8_t payload[8];
  static uint8_t payloadIndex = 0;
  const uint8_t targetHeader[4] = {'A', 'B', 'C', 'D'};

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (estado == 0) {
      if (b == targetHeader[headerIndex]) {
        if (++headerIndex == 4) {
          headerIndex = 0;
          payloadIndex = 0;
          estado = 1;
        }
      } else {
        headerIndex = (b == targetHeader[0]) ? 1 : 0;
      }
    } else {
      payload[payloadIndex++] = b;
      if (payloadIndex == 8) {
        FLOATUNION_t f;
        memcpy(f.bytes, payload, 4);     h     = f.number;
        memcpy(f.bytes, payload + 4, 4); h_ref = f.number;
        estado = 0;
      }
    }
  }
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
