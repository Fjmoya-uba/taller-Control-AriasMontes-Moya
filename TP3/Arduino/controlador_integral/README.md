# Control integral de posicion TP3

Ley: u = -CONTROL_K*xhat + CONTROL_KI*xi, con xi_dot = p_ref - p_medida.
Estados fisicos: [theta, omega, p, v]. El quinto estado es la integral del error;
se calcula con el sonar y no requiere aumentar el observador. Sin feedforward.

1. Ejecutar TP3/Matlab/observador_completo.m.
2. Ejecutar TP3/Matlab/controlador_integral.m para disenar y exportar ganancias.
3. Compilar y cargar controlador_integral.ino para la placa real.

Polos iniciales continuos: [-4, -8, -20, -25, -3]. El script comprueba
establecimiento nominal al 2% menor a 2.5 s y sobrepico menor a 15%.
Estos resultados continuos no garantizan el desempeno real: validar con
muestreo de 20 ms, retardo de planta, sensores y saturacion del servo.

Secuencia relativa al cero del sonar (15.612 cm): 0 cm de 0 a 10 s,
+5 cm de 10 a 20 s, 0 cm de 20 a 30 s, +5 cm de 30 a 40 s, etc.
Comienza tras calibrar, en el primer ciclo activo. Registrar al menos 40 s.
Repite indefinidamente y se reinicia con reset. La integral empieza en cero,
se conserva entre escalones y deja de actualizarse ante un fallo enclavado.
Anti-windup condicional: no acumula incrementos que agraven la saturacion.
Mantener el equipo quieto y horizontal durante la calibracion inicial.

Misma trama abcd + diez float32, 115200 baud: d1..d8 estados medidos/estimados,
d9 comando aplicado y d10 pedido. La referencia y la integral no se transmiten.
Compatible con el receptor existente y guardar_control_sin_referencia.m.
Reconstruir la referencia desde la primera trama: alterna 0 y +5 cm cada 10 s.
El ensayo 01 guardado anteriormente usa la secuencia vieja (-7, +3, +8, -3 cm,
con cambios a 8, 16 y 24 s); no aplicarle la secuencia nueva.
Comparar posicion medida con referencia y medir establecimiento/sobrepico
respecto a cada salto. Repetir tambien las perturbaciones al carrito.

Se conservan limites, fallos y observador del control con referencia.
No se ha compilado ni cargado en hardware desde este entorno.
