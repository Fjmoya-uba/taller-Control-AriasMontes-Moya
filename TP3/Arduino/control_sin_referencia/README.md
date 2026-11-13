# Regulador sin referencia TP3

Ley: u = -CONTROL_K*xhat. Estados: [theta, omega, p, v].
No hay referencia, feedforward ni integrador. No se carga la placa automaticamente.

1. Ejecutar el archivo guardado TP3/Matlab/observador_completo.m.
   Exporta el mismo encabezado a observador_completo y control_sin_referencia.
2. Ejecutar el archivo guardado TP3/Matlab/controlador_sin_referencia.m.
   Genera coeficientes_controlador.h con las ganancias calculadas de los polos actuales.
3. Abrir control_sin_referencia.ino y compilar en Arduino IDE para la placa real.
   Cada cambio de polos requiere ejecutar el script correspondiente y recompilar/cargar.
   Los encabezados se incorporan al compilar; no se actualizan en una placa ya cargada.

Se conservan sensores, filtro, calibracion, centro 1544 us, limites 1100 a
2000 us y cero de posicion 15.612 cm del sketch del observador. Mantener
barra horizontal y equipo quieto durante los dos segundos de calibracion.
El control comienza en la primera medicion valida; iniciar cerca del centro.
No se ha validado estabilidad sobre hardware ni compilado para la placa.

Retardo asumido de planta: 40 ms, dos muestras. Antes de observar,
historialU[j] = u[k-1-j], por lo que se usan los indices 1 y 2 para
u[k-2]+u[k-3]. La orden al servo se aplica en el ciclo actual, sin espera
adicional. El historial registra la orden despues de saturar y redondear.

## Registro MATLAB

115200 baud, cabecera ASCII abcd y DIEZ float32 little endian:
40 bytes de datos, 44 bytes por trama, 50 tramas/s. Configurar diez salidas
single y 40 bytes de datos en el receptor, ademas de la cabecera abcd.
No se modificaron los modelos Simulink existentes.

| Canal | Senal | Unidad |
| --- | --- | --- |
| d1 | Angulo medido, filtro complementario | rad |
| d2 | Angulo estimado | rad |
| d3 | Velocidad angular medida, gyro sin bias | rad/s |
| d4 | Velocidad angular estimada | rad/s |
| d5 | Posicion medida relativa al cero | cm |
| d6 | Posicion estimada | cm |
| d7 | Velocidad de posicion por ventana de 100 ms | cm/s |
| d8 | Velocidad del carrito estimada | cm/s |
| d9 | Comando aplicado relativo a 1544 us | us |
| d10 | Comando pedido sin saturar, relativo a 1544 us | us |

La velocidad por ventana es una comparacion aproximada con retardo, no una
medicion directa. Se envia NaN hasta completar la ventana o si no hay dato.
El tiempo se toma de out.tout en Simulink; no se transmite como canal.
El pulso absoluto aplicado al servo es 1544 + d9 us.

Graficar cuatro comparaciones medido/estimado, una por estado; ademas,
comando pedido/aplicado para detectar saturacion.
Para el informe, mostrar los cuatro estados ante perturbaciones hacia ambos lados.
Guardar con TP3/Matlab/guardar_control_sin_referencia.m tras adquirir out.

Fallos enclavados: 1 sensor/limites; 2 plazo; 4 transmision; 8 no finito.
Cualquier fallo manda centro y enciende LED; reset para rearmar. Horizontal
no detiene instantaneamente al carrito. Plazo perdido puede detener tambien
la telemetria. La mascara de fallos es interna y no se transmite.
Descartar ensayos con fallo, LED encendido o perdida de tramas.
