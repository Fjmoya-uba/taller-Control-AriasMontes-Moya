# Version Tustin

El observador que funciono en el ensayo fue el de Tustin, no el de Euler.
El exportador utiliza el modelo corregido del TP2:
`G(s) = 36.82/(s^2 + 50.12*s + 967.5)`, con salida en grados y entrada en us,
como en `TP2/Codigos Matlab/disenar_control.m`. Ya no carga el antiguo
`modelo_estimado.mat`. La conversion a radianes se hace en Bc.
El retardo externo de 40 ms del sketch se conserva; queda revisar su
consistencia con el modelo del TP2 antes del proximo ensayo.

En MATLAB, desde Practica7, ejecutar `exportar_observador_tustin`.
Esto genera `validar_observador_tustin/coeficientes_observador_tustin.h`.
Despues cargar `validar_observador_tustin/validar_observador_tustin.ino`.
El encabezado debe generarse antes de compilar; requiere Control System Toolbox.
Para medir usar el mismo receptor: `datos = recibir_observador("COM7", 30)`.
Las unidades y la trama de cuatro senales se conservan.

Se discretiza por Tustin el observador continuo completo:

`F = Ac - Lc*C`, `h = Ts/2`, `M = I-h*F`

`Ad = M\(I+h*F)`, `Bu = M\(h*Bc)`, `By = M\(h*Lc)`

`xhat[k] = Ad*xhat[k-1] + Bu*(u[k-2]+u[k-3]) + By*(y[k]+y[k-1])`

Los dos valores de entrada son los extremos de la entrada retardada en
el intervalo integrado. No representan un retardo adicional: la senal
que entra al observador sigue siendo u[k-2]. Se transmite el estado fisico
actual despues de integrar; no se transmiten coordenadas transformadas.
La prehistoria de entradas y mediciones relativas, y el estado inicial,
se inicializan en cero respecto del centro calibrado.

Lc se calcula en continuo con omega_n=2*max(abs(real(polos_planta)))
y zeta=0.58, multiplicando la parte imaginaria por 1.25. Los polos discretos
se obtienen con `(1+h*p)/(1-h*p)`. No se usa exponencial ni el override
discreto `0.35 +/- 0.15j` del exportador Euler. Ajustar los parametros
continuos al principio del nuevo exportador para modificar la respuesta.

El filtro complementario tambien se integra por Tustin, con
`theta_dot = gyro + (theta_acc-theta)/tau`,
`tau = -Ts/log(0.94)` y Ts=20 ms. Se usa el promedio trapezoidal de
gyro y acelerometro; la calibracion utiliza el mismo filtro.
El filtro cambia su respuesta respecto del original con alpha fijo.

Se conserva la excitacion actual del archivo Euler: 1544, 1800, 1544,
1300 us, cada 2 segundos, y el LED de fallo de plazos/tramas.
Las versiones anteriores no se modifican. Volver a cargar el sketch tras
regenerar los coeficientes. La validacion del nuevo modelo en planta queda pendiente.
