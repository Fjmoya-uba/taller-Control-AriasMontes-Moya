# Control de angulo por estados a 50 Hz

1. Ejecutar `controlador_variable_estados.m` para obtener `K_ref`, `Fr`, `K_int` y `H`.
   El script imprime las seis constantes para Arduino sin sobrescribir ganancias.
2. Abrir `control_servo_barra/control_servo_barra.ino` en Arduino IDE.
3. Elegir `MODO_CONTROL = REFERENCIA_FR` (inicial) o `MODO_CONTROL = INTEGRAL`.
   Copiar las constantes impresas por MATLAB y configurar `REFERENCIA_GRADOS`.
   Fr usa polos [-20 -30]; integral usa [-20 -30 -4]. Estados en radianes,
   entrada relativa en microsegundos. Se disena en continuo y se ejecuta a 50 Hz;
   verificar la respuesta muestreada en el equipo.
   Cambiar el modo requiere recompilar y cargar; el reinicio pone q en cero.
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

Cada 20 ms: medir, observar, calcular la orden segun el modo, saturar y redondear
el pulso y actualizar las memorias. El observador recibe la entrada total aplicada,
luego de saturacion y redondeo, relativa al centro.

- `REFERENCIA_FR`: `u = -K_ref*xhat + Fr*referencia`. El integrador queda en cero.
  Fr da ganancia estatica unitaria para referencias constantes en el modelo nominal;
  por si solo no corrige perturbaciones constantes ni errores de modelo.
- `INTEGRAL`: `u = -K_int*xhat + H*q`. Actualizar
  `q += 0.02*(referencia-theta_medida)`; congelar si agrava la saturacion.
  No se suma Fr en este modo.

El observador conserva u[k-2]+u[k-3]: supone un retardo externo de 40 ms,
heredado de Practica7, cuya correspondencia con la planta sigue pendiente.
No se agrega un buffer que retrase la orden fisica al servo.

## Telemetria y plazos

Una trama binaria de 20 bytes por ciclo, a 115200 baudios (1000 bytes/s), compatible con Practica7.
Cabecera `abcd`, seguida de cuatro float32 little endian: omega estimada (rad/s), omega medida (rad/s), theta estimada (grados), theta medida (grados).
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
