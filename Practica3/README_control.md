# Control de angulo de la barra

El lazo es: **referencia [grados] -> error = referencia - IMU -> controlador
-> pulso del servo [us] -> mecanismo -> IMU [grados]**.
La IMU debe estar fija a la barra. Asi se mide su inclinacion respecto de la
gravedad, no solo respecto de la base. El montaje y el signo del eje deben ser
los mismos que al identificar la planta.

## Que se discretiza

La planta ya existe fisicamente: Arduino no ejecuta su modelo. El modelo G sirve
para disenar y simular el controlador. En esta propuesta se elige un PI continuo
y se discretiza **el controlador** usando el metodo bilineal (Tustin):

```text
C(s) = Kp + Ki/s
s = (2/Ts) * (1-z^-1)/(1+z^-1)

I[k] = I[k-1] + Ki*Ts/2 * (e[k] + e[k-1])
u[k] = u0 + Kp*e[k] + I[k]
```

Esta es la cuenta de `controlar()`. Sin saturacion equivale a:

```text
u[k] = u[k-1] + (Kp+Ki*Ts/2)*e[k] + (-Kp+Ki*Ts/2)*e[k-1]
```

La implementacion separa la integral para poder frenar su acumulacion en los
topes (anti-windup). `KI = 0` permite probar un P. Para otro controlador,
reemplazar `controlar()` y sus estados; para un PID conviene filtrar la derivada.
No basta con agregar un Kd a esta ecuacion.

## Puesta en marcha

1. En MATLAB, ubicarse en `Practica3` y ejecutar `estimar-planta.m`, luego
   `disenar_control.m`. El segundo usa las variables del primero; no hacer
   `clear` entre ambos. Requiere Control System Toolbox.
2. Copiar `KP`, `KI` y `U0_US` que imprime MATLAB al inicio de
   `control_angulo/control_angulo.ino`. Conservar el signo de las ganancias:
   depende del sentido de respuesta de la planta. El sketch viene con ganancias
   cero y no activa el servo hasta configurarlas; no se han validado ganancias
   numericas en este entorno.
3. Abrir ese sketch en Arduino IDE. Usa Servo, Adafruit MPU6050,
   Adafruit Unified Sensor y Wire, como los sketches existentes. Servo en pin 9,
   MPU6050 por I2C en direccion 0x68. Elegir la placa y el puerto utilizados.
4. Confirmar los limites mecanicos. Se restringe inicialmente a **700..1100 us**,
   el rango de identificacion. Los limites de referencia **-10..10 grados** son
   limites de entrada editables, no una garantia de que todo ese rango se alcance.
   Ampliar pulsos requiere comprobar recorrido y validez del modelo.
5. Mantener quieto el mecanismo durante el arranque: el servo se ubica en `U0_US`
   y se calibra el bias del giroscopio. Arranca con referencia igual a la lectura
   actual. Si horizontal no mide cero, ajustar `CERO_IMU` con la lectura en esa
   posicion. Esto corrige el montaje, no debe recalibrarse al inclinar la base.
6. Monitor Serie a **115200**, con nueva linea. Enviar `0`, `2` o `-2.5`
   (punto decimal). La salida CSV a 10 Hz es `ref_deg,angulo_deg,servo_us`.
   No usa la trama binaria `abcd` de los registros anteriores.

El PI corre a 50 Hz (`Ts = 0.02 s`) y la IMU a 100 Hz. Si se modifica Ts,
cambiar tanto `Ts_control` en MATLAB como `TS_US` en Arduino. El periodo de
identificacion no se debe confundir con el del controlador.

`wc = 1 rad/s` en MATLAB es un punto de partida lento, no una sintonizacion
experimental validada. El script comprueba estabilidad nominal y muestra la
respuesta y el pulso solicitado. Si pide pulsos fuera del rango, la respuesta
lineal mostrada no representa lo que hara el servo saturado. Probar primero
cambios pequenos de referencia. Para cambiar la rapidez, modificar `wc`, volver
a ejecutar y copiar las ganancias; el modelo identificado es una aproximacion
local y no garantiza el comportamiento para todas las inclinaciones.

## Ensayo de perturbacion y que concluir

Con una referencia alcanzable (por ejemplo cero), esperar a que se estabilice,
inclinar lentamente la base hacia un lado, mantenerla y volver. Repetir al otro
lado y con distintas inclinaciones. Registrar referencia, angulo y pulso; anotar
tambien cuanto se inclino la base y cuando, mediante una medida externa. Comparar
error final, desviacion maxima, tiempo de recuperacion, oscilaciones y saturacion.
Definir la tolerancia admitida antes de concluir si cumple.

El PI incorpora un integrador: puede eliminar el error ante una perturbacion
constante si el lazo es estable y existe un pulso dentro de los limites que
compense la perturbacion. No garantiza angulo perfectamente constante durante
el movimiento ni rechazo de cualquier inclinacion. La simulacion aproxima la
perturbacion como un angulo sumado a la salida; inclinar la base tambien puede
cambiar la dinamica y la geometria, algo que ese ensayo lineal no modela.

- Si queda error con el servo en el tope, falta recorrido o autoridad del actuador.
  Hace falta modificar la mecanica, el rango permitido o la referencia. Subir Ki
  no resuelve esta limitacion.
- Si queda error sin saturar, comprobar Ki, calibracion, signo y ubicacion de la
  IMU; dar tiempo suficiente al integrador.
- Si oscila, revisar la sintonizacion, retardos, filtro y modelo en las distintas
  inclinaciones. Puede requerir menor ancho de banda, compensacion adicional o
  modelos y ganancias para distintos puntos de trabajo.
- Para movimientos rapidos, el acelerometro tambien mide aceleracion del
  mecanismo y puede sesgar transitoriamente el angulo. Puede hacer falta mejorar
  la estimacion; una segunda IMU en la base puede aportar anticipacion de la
  perturbacion, pero no es requisito para rechazar una inclinacion constante.
- Un servo corrige un solo grado de libertad. Si inclinar "a los lados" exige
  corregir otro eje independiente, hace falta otro actuador y medir/controlar
  ese eje. El angulo usado conserva la formula de identificacion, pensada para
  inclinaciones pequenas alrededor del eje X, no orientaciones arbitrarias.

No hay resultados de robustez experimental incluidos: se completan con el ensayo
fisico. Tampoco se ha compilado el sketch para una placa en este entorno.

Referencia del metodo: [MathWorks: conversion continua/discreta y Tustin](https://www.mathworks.com/help/control/ug/continuous-discrete-conversion-methods.html).
