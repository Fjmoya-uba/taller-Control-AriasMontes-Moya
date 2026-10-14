# Validacion del observador en la planta

1. En MATLAB, ubicarse en `Practica7` y ejecutar `exportar_observador`.
   Requiere Control System Toolbox y genera
   `validar_observador/coeficientes_observador.h` desde `modelo_estimado.mat`.
   Repetir este paso si cambia el modelo o el factor de los polos.
2. Abrir `validar_observador/validar_observador.ino` y cargarlo en Arduino.
   Bibliotecas: Servo, Adafruit MPU6050 y Adafruit Unified Sensor.
   Servo en pin 9; MPU6050 por I2C, direccion 0x68.
3. Cerrar el monitor serie y ejecutar `datos = recibir_observador("COM7", 30)`
   cambiando COM7 por el puerto conectado. Mantener la barra quieta durante
   la calibracion inicial (~8 s). Si abrir el puerto no reinicia la placa,
   reiniciarla para registrar el inicio de la secuencia.

La secuencia periodica es 1544, 1750, 1544, 1350 us, con 3 s por nivel.
El filtro complementario conserva alpha=0.94 y el eje X de Practica3.
El observador usa Euler con Ts=0.02 y los polos
`exp(2*real(eig(A_cont))*Ts)`, como `estimar_theta_punto.m`.
Ambos estados se actualizan simultaneamente desde el estado anterior.
La actualizacion usa `B*u[k-2]`: el historial de dos muestras agrega
40 ms de retardo con Ts=20 ms. Se inicializa con entrada relativa cero
(servo en 1544 us) y se desplaza despues de actualizar ambos estados.
La medida de angulo corrige el observador; el giroscopo se usa para comparar
y dentro del filtro complementario, sin introducirlo como segunda medida
en el observador.

El modelo identificado en Practica3 convierte microsegundos en grados.
El exportador convierte B a radianes; la entrada es el pulso relativo a
1544 us y la salida es el angulo relativo al cero calibrado en esa posicion.
Esto supone que el mismo modelo lineal describe las variaciones alrededor
de ese equilibrio. Revisar esta conversion si se reemplaza G por otro
modelo con unidades distintas.

Serial a 115200 baud: cabecera `abcd` y cuatro float32 little endian:
velocidad estimada [rad/s], velocidad del gyro sin bias [rad/s], angulo
estimado [rad], angulo del filtro [rad]. Cada trama ocupa 20 bytes.
Se transmite x_hat[k] junto con las medidas del instante k y luego se
calcula x_hat[k+1] usando el comando de dos muestras atras. El servo
recibe el comando actual u[k]; el retardo se aplica a la entrada del observador.

MATLAB grafica y guarda la tabla `datos` en `mediciones_observador.mat`.
El tiempo se reconstruye con Ts=20 ms. El LED queda encendido si hubo un
plazo perdido o una trama omitida: descartar ese ensayo, pues el tiempo
reconstruido y el modelo de muestreo fijo dejan de ser validos.

La generacion de coeficientes y la compilacion no fueron ejecutadas en
este entorno. Ejecutar el paso 1 antes de compilar: no se incluyen
ganancias supuestas ni coeficientes de otro modelo.
