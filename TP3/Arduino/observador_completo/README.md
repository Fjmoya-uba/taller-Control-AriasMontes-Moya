# Validacion del observador completo

Ejecutar `TP3/Matlab/observador_completo.m` para regenerar el encabezado y
abrir `observador_completo.ino` en Arduino IDE. Dependencias: Servo,
Adafruit MPU6050, Adafruit Unified Sensor y Adafruit BusIO.

Servo: pin 9, equilibrio 1544 us. Sonar: trigger 6, echo 7. MPU6050: I2C,
direccion 0x68. Medicion, observador y transmision cada 20 ms. Mantener la
barra horizontal y el sistema quieto durante los dos segundos de calibracion.
Posicion relativa: distancia menos 15.612 cm; angulo relativo en radianes.

Se conserva literalmente la secuencia de `TP2/Codigos Arduino/dinamica-carrito`.
El ensayo termina al completar sus 2695 muestras (53.9 s), incluido el
descanso final. No hay corte anticipado a los 45 s.
No hay controlador de posicion ni calculo de velocidad por diferencias.
La derivada por ventanas para comparar la velocidad se hara en MATLAB.

El observador conserva los estados fisicos `[theta, omega, p, v]` y utiliza
las dos mediciones, sin realimentar el gyro como medicion adicional.
`RETARDO_MUESTRAS=2` aplica el retardo externo de entrada de Practica7;
su valor sigue pendiente de confirmar contra TP2. No esta en las matrices.
Los cuatro estados nuevos se calculan antes de reemplazar los anteriores.
Primera lectura valida: inicializar posiciones medidas y velocidades cero.
Sin eco: detener excitacion, invalidar observador y transmitir NaN en sus
estados; al recuperar medicion se reinicializa. La posicion cruda no se filtra.

## Registro

115200 baud, cabecera ASCII `abcd` y 12 float32 little endian: 52 bytes por
trama. Los receptores antiguos de tres o cuatro canales no son compatibles.

1. Tiempo desde primer ciclo [s], tomado al inicio del ciclo.
2. Angulo medido [rad].
3. Posicion relativa medida [cm], NaN sin lectura valida.
4. Comando absoluto al servo [us], aplicado despues de medir.
5. Angulo estimado [rad].
6. Velocidad angular estimada [rad/s].
7. Posicion estimada [cm].
8. Velocidad del carrito estimada [cm/s].
9. Gyro corregido por bias [rad/s].
10. Distancia original [cm], -1 sin eco.
11. Medicion valida: 1 o 0 (no indica estar dentro de limites).
12. Estado, mascara de bits: 1 sensor/limite; 2 plazo perdido;
    4 trama omitida; 8 estimacion no finita; 16 fin normal de secuencia.

El LED queda encendido al detener. Fuera de limites o ante fallo se manda
horizontal; esto no frena instantaneamente el carrito. Tras finalizar o
fallar sensor sigue transmitiendo. Un plazo perdido detiene la ejecucion;
una trama omitida deja el fallo enclavado. Descartar ensayos con fallos de
tiempo o transmision. Reset para repetir.
