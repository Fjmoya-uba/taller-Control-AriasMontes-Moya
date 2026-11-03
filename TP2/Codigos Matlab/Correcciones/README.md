# Modelo elegido y antecedentes

**Para continuar con el TP3 usamos la primera correccion por constantes
en descansos negativos, variante base**, segun la decision del 27/10/2026:

```text
G_carrito(s) = 13.487667 / [s*(s+3.2683171)]
c = -1.7860731 cm/s^2
p0 = 0.2995 cm; v0 = 0 cm/s
Validacion: RMSE = 2.70546 cm; FIT = 43.97 %
```

Parametros, procedencia y limites en [MODELO_ELEGIDO.md](MODELO_ELEGIDO.md).
El termino c no forma parte de la transferencia.

## Archivos de trabajo

- `mediciones_dinamica_carrito_1_corregida.mat`: mediciones elegidas, cambia
  solo el angulo respecto del original; es una hipotesis experimental.
- `corregir_retornos_negativos.m`: genera esa copia desde el original en
  `../Mediciones/mediciones_dinamica_carrito_1.mat`. No identifica el carrito.
- `correcciones_aplicadas.csv` y `auditoria_correccion.mat`: auditoria.
- `angulo_corregido.png` y `detalle_retornos_corregidos.png`: graficos.

Para regenerar la correccion, agregar esta carpeta al path de MATLAB y
ejecutar `corregir_retornos_negativos`. Las rutas dependen del archivo,
no del directorio actual. La ruta del MAT elegido se mantiene para no
romper los scripts del TP2 que ya lo usan.

## Experimentos conservados

Todo lo exploratorio esta en [Antecedentes/](Antecedentes/README.md).
Se preservaron scripts, documentos, MAT, CSV y graficos, sin borrar pruebas.
Sus resultados no reemplazan el modelo elegido.

| Experimento | Documento | Resultados |
|---|---|---|
| Original, episodios positivos y servo fijo | [RESULTADOS.md](Antecedentes/RESULTADOS.md) | `Antecedentes/resumen.csv`, `resultados.mat` |
| Constantes y cero inmediato, cuatro ajustes | [VARIANTES.md](Antecedentes/VARIANTES.md) | `Antecedentes/Variantes/` |
| Cero tras asentamiento | [ASENTAMIENTO.md](Antecedentes/ASENTAMIENTO.md) | `Antecedentes/Asentamiento/` |
| Comparacion ampliada de 32 candidatos | [VARIANTES_ASENTAMIENTO.md](Antecedentes/VARIANTES_ASENTAMIENTO.md) | `Antecedentes/Asentamiento/Variantes/` |
| Evaluacion de lazo abierto y P/PI/PD | [Guia historica](Antecedentes/README.md) | `Antecedentes/Metricas/` |

Para ejecutar esos scripts, agregar **ambas carpetas** al path de MATLAB:

```matlab
% Desde TP2/Codigos Matlab:
addpath('Correcciones');
addpath('Correcciones/Antecedentes');
```

Orden para reproducir las tandas, si fuera necesario:

1. `corregir_retornos_negativos` (copia elegida y auditoria).
2. `comparar_identificaciones` (primera comparacion).
3. `probar_variantes_corregidas` (incluye copia con cero inmediato).
4. `corregir_angulo_tras_asentamiento` y `comparar_asentamiento`.
5. `evaluar_variantes_asentamiento` (incluye graficos y diagnosticos).
6. `evaluar_modelos` (evaluacion adicional de controles existentes).

Requieren MATLAB y Control System Toolbox para las identificaciones y
simulaciones. Cada experimento vuelve a guardar sus salidas en Antecedentes;
regenerar resultados es opcional. Las rutas textuales guardadas dentro de
MAT historicos reflejan la ubicacion al ejecutar el experimento; no se
reescribieron esos archivos binarios. Los scripts resuelven las rutas actuales.
`Antecedentes/inventario_traslado.csv` conserva las rutas anteriores y hashes
SHA256 verificados al mover los 59 archivos. Los scripts y documentos se
actualizaron despues para reflejar la nueva ubicacion.
