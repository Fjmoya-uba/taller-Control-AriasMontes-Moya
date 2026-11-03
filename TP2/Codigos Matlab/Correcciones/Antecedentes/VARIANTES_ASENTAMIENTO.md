# Comparacion ampliada despues del asentamiento

Ejecutar `evaluar_variantes_asentamiento` desde MATLAB. Requiere las copias
ya generadas por los experimentos anteriores y Control System Toolbox.
Salidas nuevas en `Asentamiento/Variantes/`; no se sobrescriben experimentos
anteriores, informe, controladores ni planta de control u observacion.

## Protocolo fijado

Se comparan original, resta de constantes negativas, cero inmediato y cero
tras asentamiento. Para cada entrada se ajustan base (v0=0, K/a/c libres),
c=0 (v0=0), v0 libre (posicion inicial fija, c libre) y pesos equilibrados
(v0=0, c libre). Cada ajuste se repite con a en [0,100] y en [2,5].
K y c, y v0 cuando corresponde, se resuelven por minimos cuadrados para
cada a. Se incluyen extremos y se refinan todos los minimos interiores
detectados en la grilla. K no se fuerza positivo: K<=0 queda marcado como
fallo fisico del ajuste, no como un modelo aceptable.

Registro comun hasta 31.8 s y antes del primer dato invalido; primer 75 %
para ajustar y ultimo 25 % para validar. y0=0.2995 cm. Simulacion ZOH
continua desde t=0, sin reiniciar en validacion. No se estima posicion
inicial. Se guarda tanto posicion como velocidad del mismo modelo.
La recurrencia exacta ZOH se contrasta contra `lsim` para a=0, 1e-6,
3.5 y 100 con tolerancia absoluta 1e-7.

Pesos equilibrados: igual peso total en entrenamiento para pulso positivo,
pulso negativo, descanso tras positivo, descanso tras negativo y reposo
inicial como quinta categoria, igual que en el experimento anterior.
No se equilibra por episodio. Cada muestra pesa inversamente a la cantidad
de muestras de su categoria en entrenamiento, normalizado a media uno.
Los descansos de los pesos incluyen el retorno del servo. Ninguna muestra
de validacion interviene en pesos ni parametros. Los ECM ponderados solo
son comparables si usan el mismo criterio de pesos.

## Evaluaciones

`resumen.csv`: parametros, RMSE y sesgo de entrenamiento y validacion,
objetivo ponderado, limite de busqueda, ganancia no fisica y velocidad
de equilibrio a angulo cero c/a (NaN cuando a=0). K en cm/(grado*s2),
c en cm/s2, v0 y c/a en cm/s. No se imponen umbrales de plausibilidad
de c o v0 sin mediciones independientes de aceleracion o velocidad inicial.

`posicion.csv`: RMSE y sesgo (simulada menos medida) globales y por grupo.
Particion 0 = registro completo, 1 = entrenamiento, 2 = validacion.
Grupo -1 = global, 0 = reposo inicial, 1/2 = pulsos positivos/negativos,
3/4 = descansos tras positivos/negativos. Los grupos de descanso incluyen
el transitorio; las pendientes se evalan por separado despues de asentarse.

`pendientes.csv`: rectas de posicion real y simulada en cada descanso
desde retorno+0.177001159 s. Se separan por particion si un descanso cruza
el corte temporal, sin mezclar datos. Se exige al menos diez muestras.
Esta ventana comienza cuando se asienta el **servo nominal**, no cuando
se supone que se detuvo el carrito: incluye su frenado fisico inicial.
Las pendientes anteriores desde retorno+0.8 s pertenecen a otra ventana
y no se deben comparar directamente. Se informa tambien velocidad media
simulada, que no tiene por que coincidir exactamente con la pendiente OLS.

`velocidad.csv`: RMSE y sesgo frente a derivada local de un ajuste cuadratico
centrado sobre posicion medida, ventanas fijas de 11 y 21 muestras a Ts=0.02 s
(distancia entre extremos 0.20 y 0.40 s). Es una referencia aproximada,
no velocidad real medida. El nivel corto conserva mas cambios rapidos y
ruido; el largo reduce ruido y atenúa transitorios. Se calcula por particion
por separado, descartando medio ancho de ventana en cada extremo. No se
usa esta referencia para identificar, ajustar suavizado ni elegir pesos.
Las ventanas pueden cruzar limites de pulsos dentro de la misma particion,
por lo que hay suavizado de esos cambios. No hay referencias en bordes
ni mezcla de entrenamiento con validacion.

`estabilidad_entrenamiento.csv`: sensibilidad al excluir, de la funcion
de costo, un episodio completo de entrenamiento (pulso y descanso).
Se conserva toda la simulacion continua y la misma posicion inicial; no
se reinician estados ni se liberan velocidades por episodio. Se aplica
a original y asentamiento, base y v0 libre, con ambos rangos. Los episodios
incompletos al final de entrenamiento no se excluyen. Es un diagnostico
de influencia, no intervalos de confianza ni validacion independiente.

`evaluacion.mat` conserva curvas, referencias, mascaras, pesos y perfiles.
Un PNG por entrada muestra posiciones, errores y velocidades de las cuatro
variantes, con libre y restringida en columnas. La linea vertical separa
entrenamiento y validacion; gris y negro discontinuo en velocidad son las dos
derivadas. Cada ajuste conserva el mismo color en los tres paneles.
`graficar_variantes_asentamiento` regenera los PNG sin volver a ajustar.

`pendientes_tardias.csv` repite pendientes desde retorno+0.8 s, como los
antecedentes, para separar frenado inicial de deriva. `diagnostico.csv`
resume error absoluto medio de esas pendientes en validacion (igual peso
por tramo, incluidos descansos parciales) y ambos RMSE de velocidad.
`diagnosticar_variantes_asentamiento` genera estos archivos y verifica las
32 curvas completas de posicion/velocidad contra `lsim` con error <1e-6,
los pesos iguales por categoria y la reproduccion de los tres antecedentes
de RMSE. Se ejecuto y aprobo. Puede correrse sin volver a identificar.

## Resultados ejecutados el 27/10/2026

1591 muestras, entrenamiento hasta 23.84 s y validacion desde 23.86 s.
La restriccion no altera materialmente base, v0 libre ni equilibrado sobre
la nueva entrada: sus optimos libres ya estan en [2,5].

| Nueva entrada / ajuste | Busqueda | a | K | c | v0 | RMSE train | RMSE val | Sesgo val |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Base | libre | 3.6555 | 14.7370 | -2.7479 | 0 | 1.5014 | 3.9044 | +3.6867 |
| Base | [2,5] | 3.6555 | 14.7370 | -2.7479 | 0 | 1.5014 | 3.9044 | +3.6867 |
| c=0 | libre | 0 | -0.2106 | 0 | 0 | 4.4418 | 6.8348 | -4.7925 |
| c=0 | [2,5] | 2 | -0.8933 | 0 | 0 | 5.4840 | 6.8711 | +4.4081 |
| v0 libre | libre | 3.5171 | 14.1585 | -3.1723 | +8.3214 | 0.9526 | 2.1515 | +1.8857 |
| v0 libre | [2,5] | 3.5171 | 14.1585 | -3.1723 | +8.3214 | 0.9526 | 2.1515 | +1.8857 |
| Equilibrado | libre | 3.3381 | 13.1575 | -2.3952 | 0 | 1.5434 | 4.4302 | +4.2217 |
| Equilibrado | [2,5] | 3.3381 | 13.1574 | -2.3952 | 0 | 1.5434 | 4.4302 | +4.2217 |

Ambos c=0 terminan en el limite inferior y con K negativo. No son modelos
fisicamente aceptables. El c=0 restringido tiene poca deriva tardia, pero
eso no compensa invertir la respuesta a los pulsos y errar la posicion.
El equilibrio de pesos empeora la posicion reservada. Liberar v0 mejora
entrenamiento y validacion respecto de la base nueva, pero aumenta |c| y
la deriva de reposo: c/a pasa de -0.7517 a -0.9020 cm/s.
La variante equilibrada tampoco se detiene: c/a=-0.7175 cm/s.

v0=8.32 cm/s no tiene apoyo en una velocidad medida. La mediana de las
derivadas aproximadas en el primer segundo es +0.0155 y +0.0532 cm/s,
segun ventana; no equivale a medir v(0), pero contrasta con el fuerte
transitorio inicial que exige el ajuste. La contribucion de esa v0 a
posicion tiende a v0/a=2.37 cm: puede absorber desplazamientos y errores
del modelo en lugar de describir movimiento inicial real. No se adopta
esa compensacion sin evidencia fisica. Tampoco se identifica el origen
fisico de c con este ensayo.

### Referencias recalculadas con el mismo protocolo

| Entrada / ajuste | a libre | c | v0 | RMSE val libre | RMSE val [2,5] |
|---|---:|---:|---:|---:|---:|
| Original base | 6.8150 | -0.5692 | 0 | 1.1524 | 1.1508 |
| Original c=0 | 7.2518 | 0 | 0 | 2.6138 | 2.5924 |
| Original v0 libre | 6.7451 | -0.2384 | -4.6601 | 1.4641 | 1.4573 |
| Original equilibrado | 5.9143 | -0.3964 | 0 | 1.4271 | 1.4269 |
| Constantes base | 3.2683 | -1.7861 | 0 | 2.7055 | 2.7055 |
| Constantes c=0 | 0 | 0 | 0 | 6.0126 | 9.1329 |
| Constantes v0 libre | 3.2681 | -2.0126 | +3.8678 | 1.8190 | 1.8190 |
| Constantes equilibrado | 3.1307 | -1.6444 | 0 | 3.2968 | 3.2968 |
| Cero inmediato base | 3.1049 | -2.5044 | 0 | 4.6588 | 4.6588 |
| Cero inmediato c=0 | 0 | 0 | 0 | 6.9804 | 6.5520 |
| Cero inmediato v0 libre | 2.9788 | -2.9057 | +7.6938 | 2.7697 | 2.7697 |
| Cero inmediato equilibrado | 2.7609 | -2.1189 | 0 | 5.1960 | 5.1960 |

En original los cuatro restringidos terminan en a=5. En constantes c=0,
el libre tiene K<0 y a=0; el restringido a=5 con K=3.319 predice muy mal.
En cero inmediato c=0, ambos tienen K<0 y a en el borde inferior.
Los parametros restringidos completos estan en `resumen.csv`; c/v0 de la
tabla anterior corresponden a los libres.

La copia con asentamiento supera al cero inmediato en posicion y velocidad,
pero no al original en posicion. El resultado del cero inmediato no se uso
para descartarla de antemano: fue ensayada independientemente.

### Posicion por grupo de validacion

Cada celda es RMSE / sesgo, en cm. No hay reposo inicial en validacion.
Las versiones libres se muestran aqui; sobre asentamiento, las restringidas
coinciden salvo c=0. Todos los resultados, incluido registro completo,
entrenamiento y restringidos, estan en `posicion.csv`.

| Candidato | Pulso + | Pulso - | Descanso + | Descanso - |
|---|---:|---:|---:|---:|
| Original base libre | 1.074 / -0.526 | 0.955 / -0.937 | 0.657 / +0.375 | 1.502 / +0.767 |
| Original base [2,5] | 1.200 / -0.974 | 0.778 / -0.760 | 0.573 / +0.282 | 1.519 / +0.810 |
| Constantes v0 libre | 0.536 / +0.190 | 0.351 / +0.309 | 2.103 / +1.945 | 1.945 / +1.635 |
| Asentamiento base | 2.063 / +1.979 | 3.002 / +2.998 | 3.840 / +3.808 | 4.480 / +4.210 |
| Asentamiento c=0 | 2.455 / +0.443 | 10.423 / -10.376 | 9.368 / -9.275 | 3.334 / -1.352 |
| Asentamiento v0 libre | 0.726 / +0.531 | 1.063 / +1.053 | 2.145 / +2.075 | 2.549 / +2.269 |
| Asentamiento equilibrado | 2.473 / +2.435 | 3.564 / +3.559 | 4.193 / +4.161 | 5.168 / +4.926 |

La mejora de v0 libre se concentra en pulsos y desplazamiento global;
los descansos conservan sesgos importantes. En el original restringido
la pequena mejora global tampoco es uniforme entre signos y segmentos.

### Pendientes tardias y velocidad

En los dos descansos contenidos integramente en validacion, desde
retorno+0.8 s, las pendientes son (cm/s):

| Candidato | Descanso +, 26.50-28.68 s | Descanso -, 30.10-31.80 s |
|---|---:|---:|
| Posicion medida | +0.0123 | -0.0680 |
| Original base libre | -0.8113 | -1.5821 |
| Asentamiento base | -0.6745 | -0.8402 |
| Asentamiento v0 libre | -0.8117 | -1.0039 |
| Asentamiento equilibrado | -0.6090 | -0.8377 |

La nueva base reduce deriva frente al original pero no reproduce reposo.
Liberar v0 empeora ambas pendientes pese al menor RMSE de posicion.
La version equilibrada mejora modestamente estas pendientes, con un costo
grande en posicion. El descanso 6 cruza entrenamiento/validacion y se
informa por separado, sin contarlo como episodio independiente de validacion.

| Candidato | RMSE velocidad 0.20 s | RMSE velocidad 0.40 s |
|---|---:|---:|
| Original base libre | 2.162 | 2.100 |
| Original base [2,5] | 1.981 | 1.891 |
| Constantes base | 1.781 | 1.684 |
| Constantes v0 libre | 1.745 | 1.639 |
| Cero inmediato v0 libre | 1.894 | 1.801 |
| Asentamiento base | 1.688 | 1.592 |
| Asentamiento c=0 libre | 5.123 | 5.074 |
| Asentamiento v0 libre | 1.651 | 1.545 |
| Asentamiento equilibrado | 1.660 | 1.546 |

RMSE de velocidad en cm/s sobre muestras validas de validacion. Los dos
suavizados mantienen la misma conclusion general: la nueva entrada mejora
esta referencia aproximada frente al original, sin resolver posicion ni
deriva. La diferencia de 0.001 cm/s entre v0 libre y equilibrado con ventana
larga no permite preferir uno. El suavizado atenúa transitorios y no prueba
fidelidad de la velocidad instantanea. Por grupo, asentamiento v0 libre
tiene RMSE 0.20/0.40 s: pulso + 1.730/1.772, pulso - 0.886/0.716,
descanso + 1.437/1.249 y descanso - 1.907/1.831 cm/s.

### Sensibilidad entre episodios de entrenamiento

Se excluyen por turno los cinco episodios completos de entrenamiento.
En la nueva base libre, a varia entre 3.007 y 4.550, K entre 12.291 y
19.366, y c entre -3.606 y -2.342. El polo no queda fijado con precision.
Con v0 libre, a varia entre 3.135 y 3.835 y v0 entre 6.833 y 8.801 cm/s;
la compensacion inicial grande persiste, no desaparece al retirar un episodio.
Los restringidos nuevos dan esencialmente los mismos rangos.
La base original libre varia entre a=5.557 y 7.210; todas las exclusiones
de la base original restringida siguen dando a=5. Ese borde refleja
la restriccion activa, no evidencia de un polo fisico exactamente en -5.
Las exclusiones comparten datos y no miden incertidumbre estadistica formal.

## Decision y limites

Ninguna nueva variante resuelve conjuntamente pulsos, reposo y parametros
faciles de justificar. No se recomienda reemplazar automaticamente la planta.
El original base [2,5] sigue siendo un candidato restringido a contrastar:
conserva RMSE global cercano a 1.15 cm y mejora la referencia aproximada de
velocidad frente al libre, pero mantiene deriva y un optimo pegado al borde.
La nueva entrada con v0 libre obtiene mejor velocidad aproximada, a costa
de peor posicion, mayor c y una velocidad inicial sin respaldo experimental.

La unica particion temporal ya fue explorada repetidas veces; elegir por
ella no equivale a una prueba nueva. Solo hay un pulso de cada signo en
validacion y dos descansos completos; los RMSE por grupo tienen poco soporte
experimental. No se identifican friccion estatica, asimetrias ni offsets
de sensores por separado de c con estos datos. Para decidir una planta
fisica haria falta contrastar otro registro, el cero angular y el reposo
inicial. No se toco el informe ni la planta de control/observacion.
