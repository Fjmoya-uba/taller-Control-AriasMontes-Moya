# Controlador con referencia de posicion TP3

Ley: u = -CONTROL_K*xhat + CONTROL_FR*POSICION_REFERENCIA_CM.
Estados: [theta, omega, p, v]. Sin integrador; theta de equilibrio es cero.

1. Ejecutar TP3/Matlab/observador_completo.m: actualiza el encabezado del observador tambien en esta carpeta.
2. Ejecutar TP3/Matlab/controlador_con_referencia.m: exporta CONTROL_TS, CONTROL_K y CONTROL_FR a coeficientes_controlador.h en esta carpeta.
3. Abrir controlador_con_referencia.ino y configurar POSICION_REFERENCIA_CM (inicialmente +3 cm respecto al cero).
4. Compilar y cargar para la placa real. Cambiar ganancias o referencia requiere recompilar/cargar.

CERO_POSICION_CM = 15.612 cm es el cero del sonar, no la consigna.
La referencia se aplica en la primera medicion valida tras calibrar; iniciar cerca del centro y mantener la barra horizontal y quieta durante la calibracion de unos 2 s.
Se conservan sensores, filtro, saturacion, fallos enclavados, observador y retardo del regulador sin referencia. El observador recibe la entrada total realmente aplicada, incluyendo feedforward, saturacion y redondeo.

Telemetria identica al regulador: 115200 baud, abcd + diez float32 little endian (44 bytes por trama). Canales: angulo medido/estimado, omega medida/estimada, posicion medida/estimada, velocidad por ventana/estimada, comando aplicado/pedido. La referencia no se transmite; registrar la constante usada para cada ensayo. Es compatible con el receptor de diez canales existente.

No se carga la placa automaticamente. La respuesta con referencia debe validarse sobre hardware. Consultar ../control_sin_referencia/README.md para unidades, limites y diagnostico de fallos.
