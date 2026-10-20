# Corridas recortadas del carrito

Origen: `mediciones_dinamica_carrito_1.mat`. El original no se modifica.

- `corridas_rozamiento_limpias.mat`: `corridas(1:12)`, `resumen`, `todo`, `Ts`, `notas` y ruta de origen.
- `corridas_rozamiento_limpias.csv`: los mismos recortes en una tabla, separados por la columna `corrida`.
- `resumen_corridas_rozamiento.csv`: limites, cantidad de muestras y causa del corte.
- `corridas_rozamiento_limpias.png`: inspeccion de las doce corridas; gris = crudo, azul = limpio, verde = candidato para ajuste.
- `limpiar_corridas_rozamiento.m`: procedimiento reproducible y parametros de seleccion.

Cada recorte incluye angulo absoluto sin corregir, posicion original, posicion limpia, indice y tiempo originales, tiempo relativo al comienzo detectado del movimiento y marcas de calidad. Los margenes son de hasta 0.16 s. `usar_ajuste` excluye esos margenes. La posicion limpia conserva las muestras medidas: no se interpola ni suaviza. Ecos fuera de 2..40 cm y picos de mas de 1.5 cm respecto de una mediana local de cinco muestras se reemplazan por NaN. El primer invalido/pico posterior al comienzo corta el tramo candidato; no se unen fragmentos separados por huecos.

Para detectar movimiento se exige desplazamiento sostenido de mas de 0.6 cm en 0.12 s. Para detectar reposo se busca una ventana de 0.30 s con rango <=0.6 cm en una mediana movil usada solamente para deteccion, despues de avanzar al menos 2 cm. La primera muestra de esa ventana marca el fin. El reposo puede corresponder a un tope/contacto: no demuestra equilibrio por rozamiento.

SOBRE EL TIEMPO: el usuario confirmo que modifico los tiempos antes de medir. El Arduino actualmente guardado define ciclos de 10 s (4 s pausa + 2 s asentamiento + 4 s recorrido), mientras que los cambios de angulo de esta captura aparecen cada 6 s en `tout`. Los recortes se localizaron por los cambios de angulo cercanos a 2, 8, ..., 68 s. Todos los segundos y Ts se conservan tal como fueron registrados en Simulink: no se reescala el tiempo para forzarlo a coincidir con el firmware actual. El analisis toma `tout` como tiempo de adquisicion.

`d3` se excluye porque contiene la cabecera abcd interpretada como float32. No se asignan pulsos nominales a los recortes dado que la version del ensayo no esta confirmada.

Uso en MATLAB:

```matlab
S = load('corridas_rozamiento_limpias.mat');
D = S.corridas(1).datos;       % Elegir corrida de 1 a 12
Dutil = D(D.usar_ajuste,:);    % Solo el tramo candidato
plot(Dutil.tiempo_desde_movimiento_s,Dutil.distancia_cm,'.-');
```

Son candidatos para identificar: revisar especialmente arranques, cambios de inclinacion y contactos antes de interpretar los resultados como friccion libre. No concatenar distintas corridas en una trayectoria continua.

La comparacion con la otra captura y las estimaciones provisionales estan en [analisis_rozamiento/INFORME.md](analisis_rozamiento/INFORME.md). Se reproducen ejecutando `comparar_rozamiento` en MATLAB. La alineacion de ceros angulares para esa comparacion se guarda por separado; este archivo de recortes no se modifica.
