# Ensayo del observador de sesgo por Tustin

1. Desde Practica7, ejecutar `exportar_sesgo_tustin` en MATLAB. Usa
   `sumamos_el_sesog.m` y genera `coeficientes_sesgo_tustin.h` con precision
   completa. Regenerar si se cambian los polos o el modelo.
2. Abrir y cargar `validar_sesgo_tustin.ino`. Usa las mismas bibliotecas,
   conexiones, calibracion, secuencia del servo y filtro angular del sketch
   `validar_observador_tustin`. No requiere BasicLinearAlgebra.
3. Mantener la barra quieta durante la calibracion inicial. El centro sigue
   siendo 1544 us, el periodo 20 ms y el retardo de entrada dos muestras.

La calibracion resta el sesgo inicial real del giroscopo. Despues se agregan
10 grados/s exclusivamente a la medicion de velocidad del nuevo observador.
El filtro complementario sigue usando el giro calibrado sin esa adicion.
La estimacion se inicializa en cero, sin entregarle el valor del sesgo.
El polo adicional -5 es una eleccion inicial pendiente del ensayo real.

## Trama para MATLAB

115200 baud. Cabecera ASCII `abcd` y seis float32 little endian, 28 bytes:

| Posicion | Senal | Unidad |
| --- | --- | --- |
| 1 | Velocidad estimada | grados/s |
| 2 | Velocidad medida con sesgo artificial | grados/s |
| 3 | Angulo estimado | grados |
| 4 | Angulo del filtro relativo al centro | grados |
| 5 | Sesgo artificial aplicado (10) | grados/s |
| 6 | Sesgo estimado | grados/s |

Se conserva el orden de las cuatro senales anteriores, pero ahora todas se
transmiten en grados o grados/s. El receptor anterior de cuatro floats no
es compatible: adaptar la lectura a seis floats y las etiquetas/unidades.
La quinta senal es el valor artificial conocido, no otra medicion del sensor.

Con la barra quieta se espera velocidad estimada cercana a cero y sesgo
estimado cercano a 10 grados/s. Durante el movimiento, la velocidad medida
y la estimada deberian diferir aproximadamente en el sesgo. El ensayo usa
una referencia angular que no recibe el sesgo artificial; no prueba aun un
sesgo que afecte tambien al filtro complementario. El error de modelo y el
sesgo real residual pueden afectar la convergencia experimental.

Si se enciende el LED de fallo, descartar la medicion: hubo un plazo perdido
o una trama omitida y no vale reconstruir el tiempo suponiendo Ts fijo.
