# Comparación de candidatos — 27/10/2026

Ejecutar `evaluar_modelos` desde la carpeta superior para regenerar tablas
y gráficos. Agregar próximas variantes en su tabla `modelos` y especificar
su señal angular en `entradas` si difiere de las dos existentes.

## Resultados principales

RMSE en cm; menor es mejor. Abierto usa su sesgo identificado y el último
25 % del registro. Los controles de esta tabla son nominales, con c=0.

| Ensayo | Original TP | Residual corregido |
|---|---:|---:|
| Abierto, validación | 1.1524 | 2.7055 |
| P, escalón | 2.0983 | 1.8501 |
| PI, escalón | 1.6322 | 1.7097 |
| PD, escalón | 5.1677 | 5.2896 |
| P, golpe | 1.9080 | 1.7925 |
| PI, golpe | 1.8403 | 1.8062 |
| PD, golpe | 8.9829 | 8.9472 |

Error absoluto medio entre pendientes simuladas y reales de los cuatro
descansos de cada signo (incluye entrenamiento y validación):

| Descansos | Original [cm/s] | Corregido [cm/s] |
|---|---:|---:|
| Positivos | 0.5634 | 0.9620 |
| Negativos | 1.6783 | 0.6906 |

Se reduce cerca de un 59 % el error de pendiente negativo, pero se perjudica
el positivo. El nuevo ajuste no elimina la deriva: sigue incluyendo una
aceleración constante negativa, incluso con ángulo cercano a cero. Las
pendientes y su separación temporal están en `pendientes_descansos.csv`.

La mejora nominal de RMSE con P es de aproximadamente 11.8 %. Con PI empeora
4.7 % y con PD 2.4 %. Por ahora no hay una mejora uniforme que permita elegir
el nuevo candidato solo por tener un polo más lento. El detalle de tiempos
de subida y sobrepasos ayuda a separar fidelidad del transitorio y error final.

Con P el tiempo de subida simulado es 0.46 s para ambos candidatos, frente
a 0.38 s medido: coincide con la observación de que las dos curvas suben a
velocidad parecida. Lo que cambia más es el sobrepaso: 7.04 % en el original
y 24.54 % en el corregido, frente a 76.45 % real. En PI el sobrepaso pasa de
22.99 % a 39.33 %, frente a 21.14 % real, empeorando esa comparación.

## Límites de la comparación

Los golpes usan un salto supuesto de ±5 cm/s y comienzo detectado por
movimiento; no se conoce la perturbación real. Esos números no son una
validación independiente de la amplitud del modelo. En particular, el ensayo
PD golpe queda muy desplazado respecto de cero mientras ambas simulaciones
nominales vuelven al origen: un RMSE cercano a 9 cm no representa solamente
un problema de rapidez del polo.

La simulación usa los controladores discretos del informe, condiciones de
reposo y memorias cero. Confirmar que esos coeficientes corresponden a los
ensayos registrados antes de sacar conclusiones definitivas. No se vuelve a
diseñar el control ni se ajustan offsets o amplitudes para mejorar cada curva.

`metricas_detalladas.csv` incluye el escenario con c identificado además del
nominal. No mezclar escenarios al seleccionar un ganador. FIT puede ser muy
negativo cuando la variación de la posición real es pequeña y existe un
desplazamiento persistente; se mantiene su definición convencional.

Se detectó y corrigió en `../../comparar_escalon.m` (archivo en Codigos Matlab)
una sustitución de la posición simulada por otra curva continua que usaba P
y la planta nueva. Las métricas se calculaban antes de esa sustitución, con
la planta original. Ahora ese comparador grafica la misma simulación que
evalúa. El nuevo evaluador no depende de ese archivo.
