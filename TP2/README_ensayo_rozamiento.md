# Ensayos libres para identificar el carrito

Sketch: `Codigos Arduino/ensayo_rozamiento/ensayo_rozamiento.ino`.
Arranca automaticamente al encender o reiniciar Arduino. Se recibe desde Simulink;
el receptor `recibir_ensayo_rozamiento.m` anterior ya no corresponde a este arranque.

## Conexion y medicion

- Servo en 9; HC-SR04 TRIG=6, ECHO=7; MPU6050 por I2C, direccion 0x68.
- Bibliotecas Servo, Adafruit MPU6050 y Adafruit Unified Sensor.
- Horizontal comprobada experimentalmente: **1544 us**.
- Sin control angular ni de posicion: aplica un pulso fijo y mide el angulo real.
- Filtro complementario alpha=0.94, gyro X y angulo de acelerometro con Y.
- `CERO_IMU_DEG` permite corregir la lectura IMU respecto de la horizontal.

## Arranque y secuencia

Al encender/resetear, deja el servo en 1544 us y calibra el gyro durante unos
4 segundos. Mantener quieta la barra. Luego empieza a transmitir y ejecutar
la tanda, sin esperar comandos ni conexion del PC.

Configurar la recepcion en Simulink y reiniciar Arduino para capturar desde
el inicio. Si abrir el puerto produce un reset, comienza nuevamente la calibracion.
No abrir el Monitor Serie simultaneamente.

Secuencia: **1604, 1484, 1644, 1444 us**, repetida tres veces (12 ensayos).
Se configura con `DELTAS_US = {60, -60, 100, -100}` respecto de 1544 us.
Son dos amplitudes de pulso en ambos sentidos, no angulos garantizados.

Cada ensayo dura 17 segundos:

1. **8 s a 1544 us:** reubicar y sostener el carrito en el extremo de partida.
2. **2 s al pulso del ensayo:** sostener mientras se asienta la barra.
3. **7 s al mismo pulso:** LED integrado encendido; soltar sin empujar.

Elegir el extremo de partida para que el carrito baje a lo largo de la barra.
Recogerlo antes de que caiga y excluir del analisis el contacto con topes o manos.
Si no arranca, registrar ese resultado sin empujarlo. La posicion no detiene
el ensayo; ecos invalidos tampoco. Volver a 1544 us no frena instantaneamente
el carrito ni lo devuelve al inicio.

La captura completa tiene **204 s y 10200 tramas**, posteriores a la calibracion.
Al finalizar vuelve a 1544 us y deja de transmitir. No repite hasta RESET.
El puerto serie es solo de salida: no interpreta G ni X. Cerrar Simulink no
interrumpe la secuencia autonoma del Arduino.

## Recepcion en Simulink

- Baud rate: **115200**.
- Cabecera: **abcd**, bytes `[97 98 99 100]`.
- Payload: **dos single (float32) little endian**, primero angulo [deg], luego
  distancia [cm]. Trama completa de 12 bytes; payload de 8 bytes.
- Periodo de muestreo y transmision: **0.02 s (50 Hz)**.
- Distancia **-1**: eco invalido. No es una posicion fisica.

Guardar ambas medidas durante toda la tanda para recortar despues los recorridos.
La IMU y el sonar se leen en el mismo ciclo, separados unos milisegundos.
El HC-SR04 se dispara cada 20 ms con timeout acotado de 4 ms; revisar la calidad
de los ecos a esa frecuencia en el montaje.

Si la captura empieza con la primera trama y no pierde muestras, cada ensayo
ocupa 850 muestras: 400 de pausa, 100 de asentamiento y 350 de recorrido.
La trama no tiene contador ni checksum, por lo que el indice no permite detectar
todas las perdidas. Usar tambien el angulo y la posicion para recortar los ensayos.
Para identificar conservar el angulo respecto de la horizontal, sin restarle
la media de cada ensayo de inclinacion constante.

LED fijo y transmision detenida antes del final indican fallo: el firmware
centra y se detiene ante errores de IMU, plazo de muestreo excedido o UART sin
espacio. Revisar y reiniciar antes de repetir.

No se ha validado en el montaje ni compilado para la placa instalada.
