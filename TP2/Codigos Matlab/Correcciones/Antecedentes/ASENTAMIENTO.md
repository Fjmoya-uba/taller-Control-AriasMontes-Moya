# Cero angular despues del asentamiento del servo

Experimento del 27/10/2026. Ejecutar `corregir_angulo_tras_asentamiento` y luego
`comparar_asentamiento`. Salidas separadas en `Asentamiento/`. Requieren MATLAB
y Control System Toolbox para la identificacion. No se disena el observador.

Se parte directamente de `../../Mediciones/mediciones_dinamica_carrito_1.mat`.
Esta es una hipotesis experimental sobre el residual angular, no una nueva
medicion. No se aplica encima de la resta de constantes en descansos negativos
ni del cero inmediato de `Variantes/`. El peor resultado del cero inmediato
no permite descartar esta alternativa, que conserva el retorno inicial.

## Criterio previo a la identificacion

Modelo fijo: `36.82/(s^2+50.12*s+967.5)`. Para un escalon desde equilibrio,
el error respecto del valor final, normalizado por la amplitud del cambio, es

`e(t) = exp(-alpha*t) * (cos(wd*t) + alpha/wd*sin(wd*t))`,

con `alpha=25.06` y `wd=sqrt(967.5-alpha^2)`. Su envolvente cumple
`abs(e(t)) <= M*exp(-alpha*t)`, `M=sqrt(1+(alpha/wd)^2)`.
Se exige que esa envolvente sea menor o igual al **2 % del cambio angular**
y permanezca asi para todo tiempo posterior. Por lo tanto,
`ts=log(M/0.02)/alpha = 0.177001159 s`.
Es una cota conservadora, no el ultimo cruce exacto de la respuesta oscilatoria.
La ganancia 36.82 se cancela al normalizar. No se usa posicion ni polo del
carrito para elegir banda, tiempo o intervalos.

El criterio corresponde al modelo nominal de retorno desde equilibrio;
no certifica que la medicion real ya este en banda ni que no exista sesgo.
Los pulsos duran al menos 0.6 s, muy por encima de este tiempo nominal.
Se conserva el angulo medido durante el pulso y hasta `retorno+ts`.
Desde la primera muestra con `t>=retorno+ts`, se asigna cero hasta la muestra
anterior al siguiente pulso. Con Ts=0.02 s, se empieza a los **0.18 s**:
se preservan nueve muestras desde cada retorno, de 0 a 0.16 s inclusive.
Puede quedar un salto al comenzar el cero o el siguiente pulso; no se suaviza
porque eso alteraria las muestras que se solicito preservar.

Se detecta comando activo con distancia al centro mayor a 1, igual que en
los scripts anteriores; centro = mediana del comando en el primer segundo.
Se aplica a los descansos de ambos signos en el registro completo, incluido
el noveno episodio posterior a la ventana de identificacion. El reposo
inicial se conserva: no tiene un retorno previo del cual medir asentamiento.
Se modifican 1269 muestras, 141 por cada uno de los nueve descansos.

## Auditoria y archivos

- `mediciones_angulo_cero_tras_asentamiento.mat`: copia completa, solo cambia
  `out.d1`. Conserva nombres, clases y dimensiones.
- `intervalos.csv`: retorno, umbral teorico, primera y ultima muestra a cero,
  signo y cantidades de muestras preservadas y modificadas.
- `auditoria.mat`: parametros, mascaras y ambas señales angulares.
- `comparacion.png` y `retornos.png`: vista completa y detalle de cada retorno.
  Gris = original, azul = copia; vertical negra = retorno del comando,
  roja = umbral nominal de asentamiento.

El script verifica con igualdad exacta (incluidos NaN) todas las variables
restantes, todas las muestras fuera de la mascara, pulsos y transitorios.
Recarga la copia para verificar su guardado y vuelve a cargar el original
para comprobar que permanece intacto. Verificaciones aprobadas en MATLAB.

## Comparacion de identificaciones

Modelo `p''=K*angulo-a*p'+c`, y0=0.2995 cm, v0=0. Se conserva el recorte
hasta 31.8 s y antes del primer dato invalido, primer 75 % para ajuste y
ultimo 25 % para validacion continua, sin reiniciar estados. La misma
posicion real se usa para las dos entradas. K y c libres por minimos
cuadrados para cada a; mismos pesos uniformes y condiciones iniciales.

Se comparan original y nueva copia, cada uno con a libre en [0,100] y
restringido en [2,5]. Se evaluan los extremos y se refinan los minimos
interiores detectados por una grilla; no se elige un polo manualmente.
Un resultado en el borde debe interpretarse como efecto de la restriccion,
no como evidencia independiente de un polo dentro del intervalo.
La validacion no participa en la estimacion. Sigue siendo una unica
particion ya explorada, no una nueva prueba independiente.

`identificacion.csv`, `identificacion.mat` e `identificacion.png` guardan
parametros, errores, curvas y perfiles de ECM comparables entre los cuatro
ajustes. Los resultados no sustituyen automaticamente la planta del informe.

Resultados ejecutados en MATLAB el 27/10/2026:

| Entrada / ajuste | Polo [1/s] | K | c [cm/s2] | RMSE entrenamiento [cm] | RMSE validacion [cm] | FIT validacion [%] |
|---|---:|---:|---:|---:|---:|---:|
| Original libre | -6.8150 | 24.428 | -0.5692 | 1.0141 | 1.1524 | 76.13 |
| Original restringido | -5.0000 | 18.074 | -0.4234 | 1.0413 | 1.1508 | 76.17 |
| Cero tras asentamiento libre | -3.6555 | 14.737 | -2.7479 | 1.5014 | 3.9044 | 19.13 |
| Cero tras asentamiento restringido | -3.6555 | 14.737 | -2.7479 | 1.5014 | 3.9044 | 19.13 |

La nueva entrada entrega espontaneamente un polo dentro de [−5,−2], pero
predice peor la posicion reservada y requiere un sesgo de aceleracion de
mayor magnitud. Esto no justifica adoptar la correccion angular.
El original restringido pierde algo de ajuste de entrenamiento y practicamente
empata la validacion del libre (diferencia RMSE de unos 0.0016 cm). Su optimo
esta en a=5: la restriccion esta activa y el entrenamiento prefiere un polo
mas rapido. Puede conservarse como candidato restringido para contrastar,
pero estos datos no prueban que el polo fisico sea −5 ni distinguen con
solidez ambos modelos. No se modificaron la planta del informe ni los
controladores, y no se diseno el observador de Luenberger.
