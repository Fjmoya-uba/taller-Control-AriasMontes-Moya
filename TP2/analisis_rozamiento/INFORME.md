# Uso conjunto de las dos capturas del carrito

Se puede continuar en MATLAB: el procedimiento solo usa MATLAB base y no necesita System Identification Toolbox ni instalar Python. Se ejecuta con `comparar_rozamiento` desde TP2, o agregando TP2 al path.

**Resultado:** la captura dinamica identifica un modelo efectivo provisional. Las doce corridas son utiles como comprobacion externa y muestran que ese modelo todavia no explica ambos ensayos de forma consistente. Ajustar con ambos archivos no resolvio la diferencia; por ahora no conviene presentar parametros conjuntos como coeficientes fisicos definitivos.

## Datos y procedimiento

Se uso `TP2/mediciones_dinamica_carrito.mat`, no el archivo homonimo de la raiz. Contiene 2492 muestras, `tout` espaciado 0.02 s, sin ecos -1. Se excluyeron el arranque y el reposo posterior a 45 s. Se ajusto con 3..30 s; dentro de ese intervalo, 3..21 s sirve para estimar y 23..30 s para elegir la alineacion temporal. La evaluacion temporal se hizo en 32..44 s.

De `mediciones_dinamica_carrito_1.mat` se usaron las doce corridas ya recortadas. Para esta comparacion se toma `usar_ajuste` y se excluyen otros 0.08 s al inicio y 0.12 s al final, como proteccion ante liberacion y contacto. No se unieron corridas ni se interpolaron huecos. Los tramos rapidos quedan muy cortos: algunos interiores tienen apenas 10..15 muestras, y sus errores dependen bastante de la condicion inicial estimada.

El usuario confirmo que modifico los tiempos del ensayo. Se conserva `tout` en ambos archivos; no se aplica ninguna reescala a partir del firmware actual. Todas las magnitudes temporales y parametros dependen de que ese tiempo registrado represente el de adquisicion.

La ecuacion ensayada es:

```text
x'' = k sin(theta(t-retardo)) + sesgo - c tanh(v/0.005) - p v
```

Se comparan ausencia de rozamiento, solo viscoso, solo Coulomb y ambos. Posicion en metros, angulo en radianes. `k` es una ganancia estimada, `c` una aceleracion equivalente de rozamiento constante y `p` una tasa de amortiguamiento. Bajo un modelo traslacional simple, c=Fc/m y p=b/m. En un carrito con ruedas pueden representar parametros de una masa efectiva que incluya inercias de rotacion. No se conoce aqui esa masa efectiva: no se convierten a fuerzas o coeficientes dimensionales absolutos. Tampoco se identifica rozamiento estatico; tanh regulariza numericamente el cambio de signo y no es un modelo de adherencia.

El ajuste usa integrales con funciones de prueba suaves: se trasladan las derivadas de posicion a una funcion conocida, evitando calcular x'' a partir del ultrasonico. Una aproximacion polinomica cubica local estima solamente la velocidad necesaria para el signo del rozamiento. Los parametros disipativos se restringen a valores no negativos y se usa un ajuste robusto. Las ventanas se superponen: su cantidad no representa ensayos estadisticamente independientes.

## Resultado dentro de la captura dinamica

| Modelo | c [m/s2] | p [1/s] | RMSE posicion [cm] |
| --- | ---: | ---: | ---: |
| Sin rozamiento | 0 | 0 | 2.187 |
| Viscoso | 0 | 3.701 | 0.783 |
| Coulomb | 0.317 | 0 | 0.637 |
| Combinado | 0.280 | 0.489 | 0.641 |

**Estos errores son de predicciones de 0.8 s con reinicio cada 1 s**, usando posicion y velocidad iniciales estimadas con las nueve muestras anteriores al comienzo de cada prediccion. No son errores de una unica simulacion libre de 12 segundos. El angulo medido durante la prediccion es la entrada del modelo; no se simula el servo ni el controlador.

El modelo Coulomb es una referencia sencilla: k=10.602 m/s2, c=0.3168 m/s2 y sesgo=-0.0853 m/s2. La alineacion seleccionada es retardo=-0.02 s: se usa el angulo 20 ms adelantado. Es un ajuste empirico de alineacion entre sensores/filtros, no un retardo fisico negativo ni un modelo causal listo para implementar.

Al variar el semiancho de la ventana entre 0.18 y 0.30 s, c permanece entre 0.314 y 0.321 m/s2 y el error entre 0.633 y 0.641 cm. Esa estabilidad es alentadora para esta captura, pero no constituye un intervalo de confianza ni validacion fisica externa. El modelo combinado no mejora las predicciones respecto del Coulomb, por lo que no hay evidencia suficiente para declarar una estimacion separada y firme de c y p.

Ver `validacion_dinamica.png`, `comparacion_modelos.csv` y `sensibilidad.csv`.

## Comprobacion con las corridas libres

Los programas no usan la misma referencia de IMU: `dinamica-carrito.ino` resta un offset inicial y `ensayo_rozamiento.ino` conserva el angulo absoluto configurado. Se compararon las señales originales y una alineacion independiente del ajuste: para cada corrida se mide la mediana del angulo entre 0.8 y 0.2 s antes de inclinar, y se lleva esa referencia a la mediana del ensayo dinamico en 46..49 s. Esto supone que el servo centrado representa la misma orientacion fisica; no sustituye una calibracion real de horizontal.

La correccion se aplica solo a la entrada del modelo; los recortes originales siguen intactos. Los offsets empleados y los errores con/sin alineacion quedan en `validacion_corridas.csv`. Alinear de esta manera no resuelve la discrepancia: algunas corridas incluso empeoran. Es una hipotesis de referencia que debe verificarse, no una correccion certificada.

Con el Coulomb ajustado solo sobre la captura dinamica, los errores de posicion de las corridas 1, 5 y 9, que son las largas, son aproximadamente 15.7, 42.9 y 13.6 cm despues de esa alineacion. En la corrida 6 es 1.06 cm y en la 11 es 1.18 cm. El mismo modelo no explica uniformemente todos los recorridos. Las simulaciones de las corridas tienen condiciones iniciales estimadas del comienzo de cada tramo, sin reinicios durante el tramo; sus horizontes difieren y no son directamente comparables al error de 0.8 s de la captura dinamica.

Ver `validacion_corridas.png`: negro es medicion y rojo es prediccion. Las trayectorias que exceden el recorrido de la barra se muestran sin saturarlas para hacer visible el fallo del modelo.

## Prueba de ajuste con ambos archivos

Tambien se estimaron los cuatro modelos usando la captura dinamica (3..30 s) y las corridas 1..8, reservando las corridas 9..12. La suma de pesos nominal de las ventanas libres se fijo en la mitad de la de las ventanas dinamicas; el ajuste robusto puede reducir mas el peso de filas incompatibles. Para aprovechar las corridas rapidas se usaron ventanas libres de semiancho 0.08 s, mas sensibles al ruido que las dinamicas de 0.24 s. Es una prueba de viabilidad del ajuste conjunto bajo estas elecciones, no una demostracion de que cualquier modelo conjunto sea imposible.

El ajuste combinado termina en c=0.337 m/s2 y p=0: vuelve a un modelo Coulomb. Su error dinamico es 0.641 cm y la mediana del RMSE de las corridas 9..12 es 3.63 cm, aproximadamente 20% del rango recorrido al calcular primero los porcentajes por corrida. No mejora la referencia obtenida solo con la captura dinamica. Los modelos sin rozamiento y viscoso tampoco logran simultaneamente los menores errores en ambas capturas.

Ver `comparacion_ambos_archivos.csv` y `errores_por_corrida_todos_modelos.csv`. No se selecciono ni publico un coeficiente fisico definitivo a partir de estas pruebas.

## Como seguir

Usaria la captura dinamica para un primer modelo efectivo de trabajo y las corridas libres para detectar las condiciones que ese modelo no representa. Antes de refinar coeficientes, verificaria el cero real de angulo y su consistencia entre ensayos, el tiempo fisico de muestreo y si los recorridos lentos estuvieron completamente libres de manos/contactos. Los datos actuales muestran una discrepancia, pero no permiten atribuirla a una unica causa. La limpieza del ultrasonico por si sola no la resuelve.

No hace falta migrar a Python para hacer este analisis. Los resultados y tablas estan en `resultados_rozamiento.mat`; el procedimiento reproducible es `../comparar_rozamiento.m`. Se comprobo la ecuacion integral contra una trayectoria sintetica de aceleracion constante y el integrador contra la solucion analitica sin rozamiento; ambas comprobaciones pasan antes del ajuste real.
