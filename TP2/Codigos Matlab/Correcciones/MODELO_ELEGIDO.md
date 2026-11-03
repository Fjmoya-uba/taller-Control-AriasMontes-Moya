# Modelo elegido para continuar con el TP3

Decision del equipo, asentada el 27/10/2026: continuar con la primera
identificacion sobre el angulo corregido por resta de constantes en
descansos negativos, variante base. Esta decision reemplaza cualquier
ambiguedad sobre cual de los experimentos se toma como referencia.

## Parametros de referencia

Modelo del carrito, con posicion en cm y angulo en grados:

```text
p'' = K*angulo - a*p' + c
K = 13.487667 cm/(grado*s^2)
a = 3.2683171 s^-1
c = -1.7860731 cm/s^2
G_carrito(s) = 13.487667 / [s*(s+3.2683171)]
p(0) = 0.2995 cm
v(0) = 0 cm/s
```

El polo estable del carrito es -3.2683171 s^-1. El termino constante c
pertenece a la ecuacion de aceleracion y no esta incluido en G_carrito(s).
Su uso en el futuro observador debera explicitarse al disenar ese observador.

## Origen y resultado del ajuste

- Medicion utilizada: `mediciones_dinamica_carrito_1_corregida.mat`, en esta
  carpeta. Se genera con `corregir_retornos_negativos.m` a partir del original
  `../Mediciones/mediciones_dinamica_carrito_1.mat`, que se conserva intacto.
- Es la correccion por resta de constantes en descansos negativos. No es
  el cero inmediato ni el cero despues del asentamiento del servo.
- Variante base: posicion inicial fija, velocidad inicial cero, estimacion
  de K, a y c, pesos uniformes. No es la variante con v0 libre.
- Registro hasta 31.8 s; 1193 muestras de entrenamiento y 398 de validacion.
  Simulacion continua, sin reiniciar estados al comenzar validacion.
- RMSE de validacion: 2.70546 cm. FIT de validacion: 43.97 %.
- Resultados de esa variante: fila `Residual_base` de
  `Antecedentes/Variantes/resumen.csv` y curvas/parametros en
  `Antecedentes/Variantes/resultados.mat`.

Se adopta como modelo nominal de trabajo por decision del equipo, teniendo
en cuenta la recomendacion del profesor de revisar la rapidez del carrito.
No se afirma que sea el mejor en todas las metricas: el original obtiene
menor RMSE de posicion y este candidato conserva deriva en descansos.
Liberar v0 mejora el RMSE, pero empeora las pendientes y aumenta |c|;
por eso esa variante no es la seleccionada.

## Continuidad del trabajo

Para continuar con el TP3, usar estos parametros como referencia del
carrito junto con el modelo servo-barra correspondiente. No sustituirlos
por otros resultados de esta carpeta sin una nueva decision explicita.
Todavia no se diseno el observador de Luenberger.

Esta nota registra la eleccion; no modifica el informe, la planta usada
por los controladores ni los scripts de los experimentos anteriores.
Todas las alternativas quedan conservadas para poder revisar la decision:
[RESULTADOS.md](Antecedentes/RESULTADOS.md),
[VARIANTES.md](Antecedentes/VARIANTES.md),
[ASENTAMIENTO.md](Antecedentes/ASENTAMIENTO.md) y
[VARIANTES_ASENTAMIENTO.md](Antecedentes/VARIANTES_ASENTAMIENTO.md),
con sus respectivos archivos y graficos dentro de `Antecedentes/`.
