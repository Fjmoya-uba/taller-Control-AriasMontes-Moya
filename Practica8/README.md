# Control de angulo por estados a 50 Hz

1. Ejecutar `controlador_variable_estados.m` para obtener K y H.
2. Abrir `control_servo_barra/control_servo_barra.ino` en Arduino IDE.
3. Configurar `K_THETA = K(1)`, `K_OMEGA = K(2)`, `H_INTEGRAL = H` y
   `REFERENCIA_GRADOS`. Las ganancias incluidas corresponden a polos
   [-20 -30 -0.5], con estados en radianes y entrada relativa en microsegundos.
4. Confirmar centro (1544 us), limites (1100 a 2000 us), eje y pin del servo.
   Librerias: Servo, Adafruit MPU6050 y Adafruit Unified Sensor; Wire del core.
5. Cargar el sketch. Mantener inmovil durante calibracion (aprox. 8 s).
   El cero es el angulo calibrado con el servo en el centro. Un segundo despues
   de comenzar el control se aplica la referencia configurada (5 grados).
6. Cerrar el monitor serie. Desde MATLAB, en Practica8:
   usar el mismo modelo de Simulink y receptor de la Practica7 (cabecera `abcd`, cuatro single).
   Cambiar COM7 por el puerto de la placa. La apertura puede reiniciar Arduino;
   el receptor espera la calibracion. Para guardar: `save('ensayo_control.mat','datos')`.

## Control y observador

Se conserva el filtro complementario y la calibracion de Practica7.
El filtro calcula grados internamente; la medicion para el observador y control
se convierte a radianes. Omega medida es el giroscopio corregido por bias.
Se reutiliza una copia exacta del encabezado Tustin de Practica7, no otra L.
Si se regenera ese encabezado, copiarlo de nuevo a esta carpeta del sketch.

Cada 20 ms: medir, observar, calcular `u = -K*xhat + H*q`, saturar y redondear
el pulso, actualizar `q += 0.02*(referencia-theta_medida)` y las memorias.
La integracion se congela cuando aumentaria una saturacion existente.
El observador conserva u[k-2]+u[k-3]: supone un retardo externo de 40 ms,
heredado de Practica7, cuya correspondencia con la planta sigue pendiente.
No se agrega un buffer que retrase la orden fisica al servo.

## Telemetria y plazos

Una trama binaria de 20 bytes por ciclo, a 115200 baudios (1000 bytes/s), compatible con Practica7.
Cabecera `abcd`, seguida de cuatro float32 little endian: omega estimada (rad/s), omega medida (rad/s), theta estimada (rad), theta medida (rad).
No enviar texto por este puerto. No se transmiten referencia, pulso, secuencia, tiempo ni bits de fallo.
La transmision UART es asincrona; no se promete latencia constante en USB/PC.

LED encendido indica fallo: bit 0 = plazo perdido, bit 1 = trama omitida,
bit 2 = valor no finito. Los bits quedan enclavados. Un plazo perdido o valor
no finito detiene el control y manda centro; reiniciar para rearmar.
Si falta espacio UART o tiempo para enviar, se omite la trama y se registra
el fallo internamente y en el LED, sin bloquear el control. Descartar ese ensayo.
El control empieza despues de la calibracion; durante calibracion no hay tramas.
Los fallos se indican por LED; no viajan en la trama compatible con Practica7.

Las ganancias incluidas son un punto inicial, no una validacion sobre hardware.
No se modificaron los archivos de Practica7 ni se cargo la placa automaticamente.
