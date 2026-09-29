# Control generico y registro en MATLAB

El sketch activo es `control_angulo/control_angulo.ino`, en Practica4.
El lazo usa error `referencia - angulo`, controlador, pulso del servo e IMU.
Control y transmision: 50 Hz (Ts=0.02 s); IMU: 100 Hz.

## Configurar la cascada

Cada fila de `CONFIG_ETAPAS` contiene `{K, cero, polo}` y representa
`C_i(s) = K * (s + cero) / (s + polo)`. Las etapas se multiplican.
Los parametros son continuos, en rad/s, con la convencion de signo de esa
formula: por ejemplo, polo=5 representa un polo en s=-5.
Se discretizan al iniciar mediante Tustin con el periodo `TS`.

La fila inicial `{KP, KI/KP, 0}` reproduce `KP + KI/s`, con los valores
actuales KP=1 y KI=0; requiere KP distinto de cero. Para ganancia pura
usar `{K, 0, 0}`. Para una etapa de adelanto/atraso usar `{K, cero, polo}`.
La tabla determina automaticamente la cantidad de etapas.
Esta representacion admite factores reales de primer orden con igual grado
en numerador y denominador; no representa cualquier funcion de transferencia.

La salida del controlador se interpreta en grados de mando y se limita a
-14.8..22.3 grados. `anguloAPulso` usa la calibracion de TP1: para valores
negativos, `900 + grados * 350/14.8`; para positivos, `900 + grados * 700/22.3`.
El pulso se redondea y limita a 550..1600 us. La calibracion corresponde al
angulo de la barra, no al eje del servo. El tracking se calcula en grados.
Este mapeo cambia la ganancia del lazo: un controlador disenado para una
planta con entrada en microsegundos debe revisarse antes de reutilizarlo.
El tracking de saturacion corrige la memoria de la ultima etapa con `KAW`.
Colocar el integrador al final: esta correccion no evita windup de integradores
en etapas anteriores. Ganancias, equilibrio y limites requieren validacion
en el mecanismo; no hay una sintonizacion experimental nueva.

`disenar_control.m` actualmente define `Cz` con polos y ceros **discretos**.
No copiar esos valores directamente a la tabla continua. Para comprobar
la cascada en MATLAB, construir sus factores continuos y usar
`Cz = c2d(C, 0.02, 'tustin')`; el periodo de la planta discreta debe coincidir.

## Recibir y graficar

1. Cargar el sketch y cerrar el Monitor Serie.
2. En MATLAB, ubicarse en Practica4 y ejecutar:
   `datos = recibir_guardar_control("COM3", 60);`
   Cambiar COM3 por el puerto real. Registra 60 segundos. La referencia se
   fija en `REFERENCIA_DEG` del sketch; MATLAB no envia comandos.
   Mantener quieta la barra al arrancar durante la calibracion.
3. La figura muestra referencia y angulo del filtro complementario, error
   r-y y pulso del servo con limites de saturacion. El titulo muestra los
   valores actuales. Guarda un MAT con fecha en Practica4, incluyendo
   `datos.error_deg`. Cerrar la figura
   finaliza el registro y guarda lo recibido.

Trama binaria de 16 bytes: ASCII `abcd` seguido de tres float32 little-endian,
en orden `referencia_deg`, `angulo_deg`, `servo_us`. No lleva comas ni salto
de linea. Es el protocolo de la IMU con **3 valores, no 6**.
En Simulink configurar cabecera abcd y payload de 12 bytes / 3 singles.
La entrada del controlador es `referencia_deg - angulo_deg`; su salida antes
del equilibrio y la saturacion no se transmite. El pulso registrado es el
entero aplicado al servo, entrada de la planta.

Arduino usa una referencia constante entre -10 y 10 grados definida en el
codigo. La segunda variable transmitida es la salida del filtro complementario.
Si no encuentra la IMU, enciende el LED integrado y no transmite.
Los tiempos del MAT son de recepcion en PC, no timestamps del Arduino.
Si no hay espacio en el buffer TX se omite una trama para no bloquear el lazo.
El protocolo heredado no incluye checksum ni contador de muestras.

Para observar la respuesta a una perturbacion, mover ligeramente la barra
y soltarla tras la calibracion. Ver si el angulo vuelve a un valor acotado,
si las oscilaciones decrecen y si el PWM llega a los limites. Con KI=0 puede
quedar error permanente: eso no implica por si solo inestabilidad. Estas
graficas muestran el comportamiento observado, no demuestran estabilidad
ni calculan margenes de ganancia/fase. El tiempo es de recepcion en PC.
