# Ensayo estático de la IMU

Abrir `t_med_imu/t_med_imu.ino` en Arduino IDE. Está en una subcarpeta propia para que Arduino no combine sus `setup()` y `loop()` con los de `t_med_pote.ino`.

1. Instalar las bibliotecas **Adafruit MPU6050**, **Adafruit Unified Sensor** y **Adafruit BusIO** desde el gestor de bibliotecas.
2. Seleccionar la placa y puerto del Nano/Uno utilizado. Se conserva el cableado del proyecto: SDA=A4, SCL=A5, GND común, dirección `0x68` (cambiar a `0x69` si AD0 está alto). Alimentar según el módulo utilizado.
3. Cargar el sketch y abrir el monitor serie a **115200 baud**.
4. Fijar la barra, preferentemente cerca de horizontal, y enviar **m**. Espera 3 segundos, calibra el offset X del giróscopo con 500 lecturas (pausa de 3 ms, como v5), ejecuta 300 actualizaciones de calentamiento y toma 2000 lecturas a 100 Hz: aproximadamente 28 segundos en total. Mantener la IMU quieta durante toda la secuencia. No acciona el servo; sujetar la barra para que no se mueva.
5. Copiar el resumen del monitor. Enviar **m** para repetir en la misma posición o en otro ángulo fijo.

## Relación con el repositorio

- `Practica1`: ensayos de potenciómetro, ultrasonido y servo; `TP1/codigo/t_med_pote.ino` cronometra el potenciómetro.
- `Practica2`: lectura MPU6050 con Adafruit y envío binario `abcd` + seis `float` a 115200 baud; recepción MATLAB en `recibir_guardar_imu.m`.
- `Practica3`: versiones iniciales con `Wire` directo y ráfagas de 14 bytes; v4/v5 usan `mpu.getEvent()`, filtro complementario y servo. La fórmula de inclinación es `atan2(ay, sqrt(ax*ax + az*az)) * 180/PI`. Los scripts de estimación procesan mediciones de la planta.
- El repositorio también contiene modelos Simulink de comunicación, PCB KiCad, hojas de datos, pruebas del servo y el informe en `TP1/latex/main.tex`.

Este ensayo usa el ángulo complementario de v5: `theta = 0.96*(theta_anterior + (gyroX-offset)*180/pi*dt) + 0.04*theta_acc`. El `dt` se obtiene entre inicios de lecturas con `micros()`, en segundos. Se reinicia el estado en cero en cada ensayo y se excluyen los primeros 3 segundos de actualizaciones de las estadísticas. Cronometra lectura, cálculo del ángulo del acelerómetro, corrección del giróscopo y actualización complementaria. No incluye el control del servo ni la integración de gyro independiente que v5 transmite como diagnóstico. La salida es texto, incompatible con los receptores binarios de las prácticas.

## Configuración y resolución

Por defecto: acelerómetro **±8 g** (igual que v5), giróscopo ±500 grados/s, DLPF nominal 44 Hz, I²C **400 kHz**. Se aplica `Wire.setClock()` después de `mpu.begin()` para que una reinicialización del bus no cambie la velocidad elegida.

En `RANGO` se puede seleccionar `MPU6050_RANGE_2_G` para comparar con el cálculo ideal del informe; por defecto se reproduce el rango de v5. La sensibilidad y la resolución impresa se ajustan automáticamente. En `I2C_CLOCK_HZ` se puede elegir `100000` para comparar con el cálculo del informe. Repetir el ensayo tras cada cambio y registrar la configuración.

Cerca de horizontal, para gravedad proyectada sobre Y:

`Delta_theta ≈ (180/pi) / sensibilidad_LSB_por_g`

| Rango | Sensibilidad [LSB/g] | Resolución ideal cerca de horizontal [grados/LSB] |
| --- | ---: | ---: |
| ±2 g | 16384 | 0,003497 |
| ±8 g | 4096 | 0,013988 |

La salida la etiqueta como referencia del ADC del acelerómetro: no es la resolución del filtro complementario, que combina historia, giróscopo y acelerómetro. Es una aproximación de cuantización, no una exactitud garantizada ni una resolución angular uniforme para cualquier orientación. La transformación usa tres ejes; el efecto de cada LSB depende de la orientación. No corresponde dividir 180 grados por 65536.

Sobre el ángulo complementario, el ensayo informa la desviación estándar muestral `sigma`, el ruido pico a pico y `3*sigma` como **umbral orientativo respecto de la media en reposo**. No llama resolución al menor salto entre muestras: el ruido puede producir saltos arbitrariamente pequeños. `3*sigma` no prueba que dos ángulos puedan distinguirse, ni incluye errores de offset, escala o montaje. Para verificar detectabilidad hay que comparar posiciones conocidas. Tampoco se supone que promediar N muestras mejora automáticamente por raíz de N: el filtrado introduce correlación.

La fórmula tiene rango de −90 a +90 grados y presupone reposo dominado por gravedad. El control de norma entre 8 y 12 m/s² descarta datos claramente incompatibles, pero no detecta todo movimiento ni todo error de comunicación.

## Qué tiempo usar en el informe

La fila **`Medicion = getEvent + complementario [us]`** mide desde antes de adquirir los eventos hasta obtener el ángulo en grados. Reportar media y rango mínimo–máximo, junto con placa, biblioteca, rango e I²C. No confundir la desviación estándar con una incertidumbre instrumental certificada.

Se muestran también:

- `getEvent [us]`: comunicación y conversiones de la biblioteca para acelerómetro, temperatura y giróscopo.
- `Calculo complementario [us]`: cálculo de dt, trigonometría, corrección y conversión del giróscopo y actualización del filtro.
- `Intervalo entre lecturas [us]`: período real de adquisición por software, nominalmente 10000 µs. No es el tiempo que tarda la operación.

La calibración inicial, calentamiento, estadísticas, esperas y salida serie quedan fuera del tiempo de medición. Este tiempo de ejecución no mide la latencia ni el tiempo de establecimiento del filtro complementario. Las llamadas a `micros()` y las interrupciones sí aportan sobrecarga. En Nano/Uno AVR a 16 MHz, `micros()` tiene pasos de 4 µs; los decimales de una media no mejoran esa resolución individual. El resultado se almacena en una variable `volatile` antes del último timestamp para evitar que el compilador elimine o posponga el cálculo.

El sensor actualiza internamente en forma continua: con DLPF habilitado y divisor 0 la tasa nominal es 1 kHz. El programa lee a 100 Hz sin sincronización mediante DATA_RDY/FIFO y no conserva todas las muestras internas. El tiempo medido no incluye la espera hasta una actualización ni caracteriza la latencia física del filtro interno. Las muestras pueden seguir correlacionadas.

### Estimación del bus frente a la biblioteca

Para una lectura por registros de B bytes, contar aproximadamente `9*(B+3)` pulsos SCL: dirección de escritura, registro inicial, dirección de lectura y B bytes, todos con ACK/NACK. START/STOP agregan restricciones temporales, no un pulso de datos fijo.

- 6 bytes: 81 pulsos → 810 µs a 100 kHz o 202,5 µs a 400 kHz.
- 14 bytes: 153 pulsos → 1530 µs a 100 kHz o 382,5 µs a 400 kHz.

En el [código de Adafruit consultado](https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp), `_read()` lee 14 bytes y además consulta `ACCEL_CONFIG` y `GYRO_CONFIG` (un byte cada uno). Eso suma aproximadamente 225 pulsos: **2250 µs a 100 kHz o 562,5 µs a 400 kHz**, antes de sobrecargas, pausas entre transacciones y cálculo. Verificar la versión instalada si se quiere comparar exactamente; son estimaciones de bus, no resultados experimentales.

Por esto, el cálculo de 6 bytes a 100 kHz de `main.tex` corresponde a otra implementación. Para describir este ensayo, usar el tiempo medido con Adafruit y explicitar 400 kHz, o cambiar la constante y volver a medir.

## Límites de validación

Una lectura detectada como inválida aborta el ensayo: no se continúa con un estado recursivo contaminado. El resumen avisa si se pierden períodos (incluye el calentamiento). Cuando el core lo soporta se habilita timeout de Wire. La implementación consultada de `getEvent()` devuelve siempre `true` y no propaga todos los errores I²C; el chequeo del retorno, timeout y norma no garantiza detectar cada lectura corrupta. No usar resultados de una conexión inestable.

Se verificó la sintaxis con `avr-g++` para ATmega328P, core AVR 1.8.8 y Adafruit MPU6050 2.2.9 instalada localmente. También se corroboraron las lecturas adicionales en esa biblioteca. Esta comprobación no incluye enlazado, tamaño final del firmware ni ejecución en hardware. Los tiempos y el ruido experimental deben obtenerse con la placa y la IMU.
