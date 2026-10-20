# Ensayos de control del TP2

Adaptación de `Practica5/control_carrito_matlab_v3/control_carrito_matlab_v3.ino`.
Conserva los pines (TRIG 6, ECHO 7, servo 9), sonar no bloqueante, control
a 50 Hz, límites de 1100 a 2000 µs y centro de 1544 µs.

## Proporcional

El sketch viene con `B0 = 19.95262315` y los otros cuatro coeficientes en cero.
El error se expresa en cm y la salida incremental en µs.

- Escalón: dejar `ENSAYO_ESCALON = true`. La referencia relativa comienza
  en cero y pasa a 10 cm a los 5 s desde el arranque del Arduino.
- Golpe: cargar con `ENSAYO_ESCALON = false`. La referencia permanece
  en cero. Con el carrito estabilizado, aplicar un ligero golpe y registrar
  la recuperación.

Iniciar la adquisición antes del escalón. El tiempo del escalón se cuenta
desde el arranque, no desde que MATLAB comienza a guardar. Si abrir el
puerto reinicia la placa, ese reinicio vuelve a iniciar el ensayo;
de lo contrario, reiniciar la placa con la adquisición preparada.
Para comparar curvas, alinear el cambio de referencia registrado con el
de la simulación (que actualmente ocurre en t = 2 s).

## Registro en MATLAB

Se conserva exactamente la trama de v3: cabecera ASCII `abcd` seguida de
tres `single` little endian, a 115200 baudios (16 bytes por registro):

1. Referencia de **distancia absoluta** en cm: 15.612 o 25.612.
2. Distancia medida en cm; -1 indica lectura inválida.
3. Pulso aplicado al servo en µs, después de saturar y redondear.

El pulso ya estaba incluido en v3. En el receptor existente, guardar también
el tercer canal. Para comparar contra las simulaciones del TP2:

```matlab
% Aplicar a los canales recibidos, excluyendo distancia_cm == -1.
referencia_posicion_cm = referencia_distancia_cm - 15.612;
posicion_cm = distancia_cm - 15.612;
error_cm = referencia_posicion_cm - posicion_cm;
% pulso_us es el tercer canal y no requiere conversion.
```

Guardar referencia, posición, pulso y tiempo de ambos ensayos por separado.
La trama original no incluye tiempo ni contador de muestras: conservar el
tiempo del receptor y no suponer continuidad si se omitieron registros.
El LED encendido indica un plazo excedido o una trama omitida.

## PI y PD

Mantener el mismo sketch y los mismos dos ensayos. Obtener el controlador
discreto con Tustin a Ts = 0.02 s y copiar sus coeficientes:

```matlab
Cz = c2d(C, 0.02, 'tustin');
[b, a] = tfdata(tf(Cz), 'v');
b = b / a(1);
a = a / a(1);
b(end+1:3) = 0;
a(end+1:3) = 0;
% COEF_A1 = -a(2); COEF_A2 = -a(3);
% COEF_B0 = b(1); COEF_B1 = b(2); COEF_B2 = b(3);
```

La estructura admite controladores propios de orden hasta dos. Un PD
filtrado permite limitar la amplificación del ruido. La realimentación de
la salida limitada en la recurrencia se conserva de v3; su comportamiento
al saturar debe evaluarse para cada PI/PD y no equivale a un anti-windup
general. Las lecturas inválidas reinician la memoria y llevan el servo al
pulso de equilibrio, como en v3.
