# Primera comparación

Ejecutada en MATLAB el 27/10/2026 sobre la medición 1. Sin editar señales.

| Método | Polo del carrito [1/s] | K | c [cm/s²] | FIT validación | RMSE [cm] |
|---|---:|---:|---:|---:|---:|
| Ángulo medido, base | -6.8150 | 24.428 | -0.5692 | 76.13 % | 1.1524 |
| Episodios positivos | -4.9082 | 15.006 | 0.9369 | -78.99 % | 8.6420 |
| Servo fijo, desde comando | -3.0424 | 13.128 | -3.4924 | 39.06 % | 2.9423 |

La base reproduce el resultado existente. El segundo método usa tres
episodios positivos completos de entrenamiento, que empiezan a 3, 10.2 y
17.3 s. Su mal resultado corresponde a la simulación continua común,
incluidos los negativos; no al ajuste local de esos tres episodios.

La estructura con servo fijo produce un polo dentro del intervalo sugerido
por el profesor, sin imponer ese intervalo. Sin embargo, predice peor la
posición reservada que el modelo base. También requiere una aceleración
constante más grande, cuyo origen no puede deducirse de este ensayo.

Conclusión: ninguno de estos dos experimentos justifica por ahora reemplazar
el modelo utilizado para control u observación. Un polo más lento por sí
solo no garantiza una simulación más fiel. El siguiente experimento posible
es una copia con retornos negativos simetrizados, documentando exactamente
qué muestras se cambian y contrastando la posición original. Esa corrección
todavía no se implementó en esta primera comparación.

Los números completos están en `resumen.csv` y las curvas en
`comparacion.png`; los modelos y señales están en `resultados.mat`.
