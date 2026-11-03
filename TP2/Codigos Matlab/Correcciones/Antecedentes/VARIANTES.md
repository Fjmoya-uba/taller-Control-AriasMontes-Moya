# Variantes sobre el angulo corregido (27/10/2026)

Ejecutar `probar_variantes_corregidas` desde MATLAB con esta carpeta en el path.
Requiere Control System Toolbox. Resultados aislados en `Variantes/`; no cambia
el informe, los comparadores anteriores ni las mediciones de origen.

Modelo: p'' = K*angulo - a*p' + c. Posicion inicial fija en 0.2995 cm.
Primer 75 % hasta 31.8 s para estimar, ultimo 25 % para validar mediante
simulacion continua, sin reiniciar estados. Se busca a entre 0 y 100 1/s
con grilla y refinamiento local; K, c y v0 se obtienen por minimos cuadrados.
K no se restringe: una ganancia negativa se considera diagnostico de fallo.

| Entrada / ajuste | a [1/s] | K | c [cm/s2] | v0 [cm/s] | RMSE validacion [cm] | FIT [%] |
|---|---:|---:|---:|---:|---:|---:|
| Residual, base | 3.2683 | 13.4877 | -1.7861 | 0 | 2.7055 | 43.97 |
| Residual, c=0 | 0 | -0.2994 | 0 | 0 | 6.0126 | -24.53 |
| Residual, v0 libre | 3.2681 | 13.2139 | -2.0126 | 3.8678 | 1.8190 | 62.33 |
| Residual, equilibrado | 3.1307 | 12.9726 | -1.6444 | 0 | 3.2968 | 31.72 |
| Cero, base | 3.1049 | 14.3380 | -2.5044 | 0 | 4.6588 | 3.51 |
| Cero, c=0 | 0 | -0.2218 | 0 | 0 | 6.9804 | -44.57 |
| Cero, v0 libre | 2.9788 | 13.8454 | -2.9057 | 7.6937 | 2.7697 | 42.64 |
| Cero, equilibrado | 2.7609 | 12.4204 | -2.1189 | 0 | 5.1960 | -7.62 |

## Interpretacion

- c=0 empeora claramente y lleva a al borde inferior de la busqueda con K
  negativo. No es una identificacion fisicamente aceptable del carrito.
- v0 libre mejora el error global, pero no elimina c: aumenta su magnitud.
  En los descansos exclusivamente de validacion, las pendientes simuladas
  pasan de -1.212 y -0.660 cm/s a -1.268 y -0.727 cm/s; las reales son
  +0.012 y -0.068 cm/s. La deriva empeora pese al mejor RMSE. El v0 estimado
  tampoco confirma que el carrito estuviera moviendose: puede absorber error
  de posicion inicial o una deficiencia del modelo. Requiere contraste fisico.
- El peso equilibrado empeora validacion. Se da igual peso total a pulsos
  positivos, negativos, descansos positivos y negativos, y al reposo inicial
  como quinta categoria. Cada muestra pesa inversamente al numero de muestras
  de su categoria en entrenamiento; validacion nunca interviene en los pesos.
  No se iguala el peso de cada episodio individual.
- Al no haber una mejora que resuelva la deriva, se ensaya tambien cero exacto
  en todos los intervalos con comando central, de ambos signos, incluido el
  reposo inicial y la cola final. La puesta a cero es inmediata desde el
  retorno del comando: tambien elimina el transitorio angular fisico del
  servo. Es una hipotesis experimental fuerte, no una correccion confirmada
  del sensor. Ninguna de sus cuatro variantes supera la base residual.

La base original sin correccion tenia RMSE 1.1524 cm y FIT 76.13 % en la misma
validacion (ver RESULTADOS.md). Ninguna variante de esta tanda la supera.
Estos resultados no justifican reemplazar por ahora la planta del informe.
La seleccion usa una sola particion temporal; no es una prueba independiente
adicional despues de comparar variantes.

## Archivos

- `resumen.csv`: parametros y metricas globales sin ponderar, mas objetivo del
  ajuste (solo comparable entre candidatos con el mismo criterio de pesos).
- `por_tramos.csv`: particion 1 entrenamiento / 2 validacion; grupo 0 reposo
  inicial, 1 pulso positivo, 2 negativo, 3 descanso positivo, 4 negativo.
  FIT puede ser muy negativo en descansos de poca variacion: revisar RMSE.
- `pendientes.csv`: pendientes desde 0.8 s despues del retorno; se etiqueta
  si el tramo completo pertenece a validacion. Error = simulada - real.
- `resultados.mat`: curvas, perfiles de error, segmentacion y tablas.
- `comparacion.png`: entradas, posiciones y errores de las ocho variantes.
- `mediciones_descansos_cero.mat`: copia completa, cambia solo out.d1 en
  comando central. El script verifica campos restantes y pulsos intactos.
