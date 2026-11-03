# Experimentos de identificación del carrito

> Archivo de antecedentes: los scripts y resultados exploratorios fueron
> trasladados aqui el 27/10/2026. El modelo vigente esta en
> [../MODELO_ELEGIDO.md](../MODELO_ELEGIDO.md) y la guia actual de ejecucion
> en [../README.md](../README.md). Agregar esta carpeta y su carpeta padre
> al path de MATLAB. Las salidas de los experimentos quedan aqui; la copia
> elegida, su script y su auditoria permanecen en la carpeta padre.

## Modelo elegido para continuar

**Decision del 27/10/2026:** se adopta la primera correccion por constantes
en descansos negativos, variante base: `K=13.487667`, `a=3.2683171`,
`c=-1.7860731`, `p0=0.2995`, `v0=0`. Medicion:
`../mediciones_dinamica_carrito_1_corregida.mat`. RMSE de validacion:
2.70546 cm; FIT: 43.97 %. Parametros, procedencia y limites en
[MODELO_ELEGIDO.md](../MODELO_ELEGIDO.md). Las otras pruebas se conservan
como antecedentes; no reemplazan esta eleccion.

## Cero despues del asentamiento del servo

La comparacion ampliada de cuatro ajustes y dos rangos, con referencias
anteriores, posicion, pendientes, velocidad aproximada y sensibilidad entre
episodios se ejecuta con `evaluar_variantes_asentamiento`. Ver
[VARIANTES_ASENTAMIENTO.md](VARIANTES_ASENTAMIENTO.md).

Ejecutar `corregir_angulo_tras_asentamiento` y `comparar_asentamiento`.
Nueva copia independiente del original, con pulsos y retornos preservados
durante 0.177001159 s (criterio nominal conservador del 2 %). Salidas en
`Asentamiento/`; metodo, auditoria y comparacion libre / a en [2,5] en
[ASENTAMIENTO.md](ASENTAMIENTO.md). Es distinta de la resta de constantes
y del cero inmediato. No se disena aun el observador del TP3.

## Métricas de lazo abierto y P / PI / PD

Ejecutar `evaluar_modelos`. Los candidatos se definen en la tabla `modelos`
al comienzo del archivo, con parámetros explícitos para poder reproducir
la comparación sin depender del workspace. Salidas en `Metricas/`:

- `parametros.csv`: a, K, c, tau y K/a.
- `resumen_RMSE.csv`: comparación compacta de los siete ensayos.
- `metricas_detalladas.csv`: RMSE, FIT, error medio (simulada menos real),
  tiempos de subida, sobrepasos, posiciones finales, establecimiento y picos,
  con valores reales, simulados y diferencias con signo.
- `pendientes_descansos.csv`: rectas ajustadas a cada descanso desde 0.8 s
  después del retorno a horizontal; incluye signo y si pertenece íntegramente
  a validación. No se obliga a que la pendiente real sea cero.
- `configuracion_golpes.csv`: instante detectado y salto de velocidad supuesto.
- Un PNG por ensayo, `evaluacion.mat` con las curvas y `configuracion.mat`.

Abierto: último 25 % del tramo hasta 31.8 s para las métricas globales;
simulación continua desde y0=0.2995 cm, v0=0, con c identificado. Original usa
ángulo original y corregido usa su copia, contra la misma posición. Las
pendientes se presentan también para descansos de entrenamiento, etiquetadas.

Control: mismas recurrencias digitales del informe (P, PI, PD), ZOH de planta
y Ts=0.02 s, redondeo del pulso y límites 1100–2000 us. No se resintoniza.
Se inicia en reposo, posición cero y memorias cero, como los comparadores
existentes. Se simulan por separado c=0 y c identificado; la tabla compacta
usa c=0 para controles y el detalle incluye ambos. El sesgo identificado en
un experimento puede no ser transferible a otro: no se elige por ensayo.

Escalones: subida por primer cruce de 10 y 90 % de la referencia nominal
(sin interpolación entre muestras), sobrepaso respecto de esa referencia,
posición final media de los últimos 2 s y establecimiento dentro de ±2 %.
Golpes: pico absoluto y recuperación dentro de ±0.2 cm. NaN en tiempos indica
que no se alcanzó el umbral o no se sostuvo la banda hasta el fin del registro.
FIT se calcula sobre las mismas muestras que RMSE, sin retirar offsets.

**Los golpes son comparaciones exploratorias:** la intensidad del golpe real
no está medida. Se usa ±5 cm/s para ambas plantas, con signo según el
movimiento y comienzo automático tras tres muestras que exceden el umbral.
No se ajusta el impulso para favorecer un modelo. RMSE y pico de estos ensayos
no prueban por sí solos cuál planta es más fiel; revisar la alineación y la
hipótesis de impulso. El estado del controlador real antes del golpe también
puede diferir del inicio nominal de la simulación.

El evaluador es independiente de `comparar_escalon.m`. Se eliminó de ese
comparador una sustitución de la curva por un escalón continuo con P y planta
corregida: esa sustitución hacía que gráfica y métricas no coincidieran.

## Copia con corrección angular

Ejecutar `corregir_retornos_negativos` (en la carpeta padre) para generar
`../mediciones_dinamica_carrito_1_corregida.mat`, `angulo_corregido.png` y
`detalle_retornos_corregidos.png`. Este script no vuelve a identificar.

Se detecta el descanso por el retorno de `d3` al comando central y el signo
mediano de `d1` durante el pulso anterior. En descansos negativos se resta
el residual mediano de los últimos 0.6 s de los primeros 3 s del descanso.
La corrección entra con una curva cúbica suave durante 0.6 s y se mantiene
hasta el próximo pulso. Al cambiar el comando se retoma el ángulo original:
esto puede modificar el salto entre muestras en ese instante. No se fuerza
un descenso ficticio antes del pulso para enlazar las señales.
Los parámetros están al inicio del script. No se utiliza la posición ni un
polo objetivo para calcular la corrección. Se conserva el ruido angular;
se lleva a cero el nivel mediano, no cada muestra. En el descanso final
largo puede quedar deriva posterior a la ventana de referencia.

La copia preserva las variables originales y solo cambia `out.d1`.
`correcciones_aplicadas.csv` y `auditoria_correccion.mat` documentan los
intervalos y desplazamientos aplicados. El script verifica que el resto de
los datos y los pulsos activos permanecen intactos. Esta es una señal
editada experimental, no una nueva medición física.

Ejecutar `comparar_identificaciones` en MATLAB agregando esta carpeta al path.
Requiere Control System Toolbox. Las rutas se resuelven desde el archivo, sin
depender del directorio actual. No se modifica ninguna medición ni el informe.

Se usa `Mediciones/mediciones_dinamica_carrito_1.mat`, hasta 31.8 s y antes
de la primera muestra inválida. Primer 75 % para estimar, último 25 % para
validación temporal. Modelo del carrito: `p'' = K*angulo - a*p' + c`.

Se comparan:

1. **Base:** ángulo medido, misma posición inicial y velocidad cero que el
   script existente. Se buscan `a`, `K` y aceleración constante `c`.
2. **Episodios positivos:** pulsos con ángulo mediano positivo y su descanso
   completo posterior, contenidos íntegramente en entrenamiento. Se ajustan
   `K`, `c` y `a` comunes, y una velocidad inicial por episodio. Se conserva
   el tiempo de cada episodio; no se concatenan señales como si fueran
   continuas. La selección usa el signo del ángulo, no del comando.
3. **Servo fijo:** desde el comando `d3`, usando exactamente
   `36.82/(s^2+50.12*s+967.5)` de `disenar_control.m`, sin retocar sus polos
   ni ganancia ni añadir retardo. Se estima solo el carrito y `c`. El comando
   se centra en la mediana del primer segundo. El servo parte de estado cero.

En todos los casos `a` se busca libremente entre 0 y 100 1/s, con grilla y
refinamiento local. **No se obliga al polo a quedar entre -3 y -4.** Para cada
`a`, los otros parámetros se resuelven por mínimos cuadrados. El perfil de
error ayuda a ver si el polo está bien determinado o si hay un valle plano.

La validación común simula todo el registro desde las condiciones iniciales
originales, sin reiniciar al llegar al último 25 %. El experimento positivo
se valida también con el ángulo medido completo, incluidos los negativos.
Su FIT de entrenamiento común no es su objetivo local por episodios.
Los perfiles de ECM corresponden a objetivos distintos y no deben usarse
para comparar directamente la calidad de los tres métodos.

Salidas en esta misma carpeta: `resumen.csv`, `episodios.csv`,
`resultados.mat` y `comparacion.png`. `K` queda en cm/(grado*s²), `c` en
cm/s² y el polo del carrito es `-a`. Los modelos guardados no incluyen `c`.

El ajuste desde el comando introduce la hipótesis de un servo lineal
simétrico: sustituye el ángulo observado por el predicho, pero conserva la
posición real. Por eso es una hipótesis a contrastar, no una corrección
confirmada del sensor. Una mejora del FIT por sí sola no prueba que el
desplazamiento angular fuera un error de medición.

## Nueva tanda: c=0, v0 libre y pesos equilibrados

Ejecutar `probar_variantes_corregidas`. Compara las tres alternativas con
la base residual y, como diagnostico adicional, repite con angulo exactamente
cero en descansos. Salidas en `Variantes/`. Metodo, limites de interpretacion
y resultados numericos en [VARIANTES.md](VARIANTES.md).
