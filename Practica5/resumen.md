# Sintonización del control de posición del carrito

## Configuración

- Referencia de posición: **17 cm**, medida con el sensor ultrasónico HC-SR04.
- Lazo de posición a **50 Hz**: calcula la inclinación deseada de la barra.
- Lazo interno proporcional: controla el ángulo de la barra mediante el servo.
- Cero mecánico del servo: corrección de **7°** respecto del comando nominal de 90°.

## Búsqueda de la ganancia crítica

Se buscó aplicar Ziegler-Nichols por ganancia límite, usando solo control proporcional. Se tomaron oscilaciones de amplitud moderada alrededor de la referencia para evitar los topes y la saturación del servo.

- **Kp = 7,0:** las oscilaciones se amortiguaron.
- **Kp = 7,5:** se observó una respuesta estable con mucha oscilación.
- **Kp = 7,65:** se registraron oscilaciones casi sostenidas y se tomó este valor como ganancia crítica provisional (**Ko**).

El promedio de ocho intervalos entre máximos consecutivos dio un período crítico de **To ≈ 1,11 s**.

**Pendiente de confirmar:** la respuesta registrada con Kp = 7,5 no es consistente con tomar 7,65 como límite de estabilidad bajo las mismas condiciones.

## Resultados de las pruebas

| Controlador | Resultado observado |
| --- | --- |
| **P** (Kp = Ko/2 ≈ 3,83) | Se estabilizó cerca de **16,37 cm**, con un error estacionario de **0,63 cm**. |
| **PI** | Alcanzó aproximadamente los **17 cm**, con un transitorio más largo y mayores sobrepasos y oscilaciones ante perturbaciones. |
| **PID** | Al incorporar la acción derivativa, la respuesta se volvió inestable. No se obtuvo un ajuste satisfactorio. |

El **PI fue el ajuste que permitió alcanzar la referencia** en las pruebas registradas. Falta documentar las ganancias finales utilizadas y confirmar la ganancia crítica.
