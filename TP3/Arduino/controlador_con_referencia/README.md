# Controlador con referencia de posicion TP3

Ley: u = -CONTROL_K*xhat + CONTROL_FR*REFERENCIAS_CM[indiceReferencia].
Estados: [theta, omega, p, v]. Sin integrador; theta de equilibrio es cero.

1. Ejecutar TP3/Matlab/observador_completo.m: actualiza el encabezado del observador tambien en esta carpeta.
2. Ejecutar TP3/Matlab/controlador_con_referencia.m: exporta CONTROL_TS, CONTROL_K y CONTROL_FR a coeficientes_controlador.h en esta carpeta.
3. Abrir controlador_con_referencia.ino. REFERENCIAS_CM define la secuencia [-7, +3, +8, -3] cm respecto al cero, con 8 s por escalon.
4. Compilar y cargar para la placa real. Cambiar ganancias o referencia requiere recompilar/cargar.

CERO_POSICION_CM = 15.612 cm es el cero del sonar, no la consigna.
La referencia se aplica en la primera medicion valida tras calibrar; iniciar cerca del centro y mantener la barra horizontal y quieta durante la calibracion de unos 2 s.
Desde ese primer ciclo activo: 0 a 8 s, -7 cm; 8 a 16 s, +3 cm; 16 a 24 s, +8 cm; desde 24 s, -3 cm. Registrar al menos 32 s de control para observar los cuatro escalones completos. El ultimo valor se mantiene indefinidamente, sin repetir la secuencia ni volver automaticamente a cero. Reiniciar la placa para repetir. La temporizacion cuenta 400 ciclos de 20 ms por escalon y no incluye la calibracion.
Se conservan sensores, filtro, saturacion, fallos enclavados, observador y retardo del regulador sin referencia. El observador recibe la entrada total realmente aplicada, incluyendo feedforward, saturacion y redondeo.

Telemetria identica al regulador: 115200 baud, abcd + diez float32 little endian (44 bytes por trama). Canales: angulo medido/estimado, omega medida/estimada, posicion medida/estimada, velocidad por ventana/estimada, comando aplicado/pedido. La referencia no se transmite; registrar la secuencia usada para cada ensayo y alinear su inicio con la primera trama de control. Es compatible con el receptor de diez canales existente.

No se carga la placa automaticamente. La respuesta con referencia debe validarse sobre hardware. Consultar ../control_sin_referencia/README.md para unidades, limites y diagnostico de fallos.
