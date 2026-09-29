# Calibracion manual del servo

Abrir `calibrar_servo.ino` en Arduino IDE. Usa la biblioteca Servo y no necesita la IMU.

1. Conectar la señal al pin 9 (editable en `PIN_SERVO`). Alimentar el servo con una fuente adecuada para el modelo y unir su GND con el GND del Arduino.
2. Dejar libre el mecanismo para el movimiento inicial: al arrancar comanda **1500 us**.
3. Abrir el monitor serie a **115200 baudios**, con **Nueva linea** o **Ambos NL y CR**.
4. Escribir solo un entero, por ejemplo `1500`, y enviarlo. Conserva ese pulso hasta recibir otro comando valido. Solo acepta **1000 a 2000 us**.
5. Probar primero `1475`, `1500`, `1525` y avanzar de a poco. Si el mecanismo llega a un tope o el servo hace fuerza, volver hacia el centro; el rango configurado no garantiza que el mecanismo pueda recorrerlo completo.

## Encontrar el cero y medir

**1500 us es el centro de comando; no garantiza que la barra este horizontal.** Para centrar el montaje, comandar 1500 us, cortar la alimentacion y ajustar la union mecanica o recolocar el brazo del servo para dejar la barra aproximadamente horizontal. Volver a alimentar y verificar; no girar a la fuerza el eje energizado.

Para medir la barra, colocar un nivel sobre ella para ubicar el cero. Luego usar un transportador con referencia horizontal o un inclinometro apoyado en la barra, esperando que se detenga en cada punto. Si se usa un telefono, su peso puede modificar la posicion del mecanismo.

Otra opcion es fijar la MPU6050 a la barra: el sketch existente `../tope-subeybaja/tope-subeybaja.ino` contiene lectura de inclinacion, pero usa otro rango y arranca en 900 us; no cargarlo como reemplazo de esta prueba sin adaptarlo. La IMU mide la inclinacion de la barra, no directamente el giro del eje del servo.

Registrar varias parejas reales, por ejemplo:

| Pulso (us) | Angulo medido de la barra (grados) |
| --- | --- |
| 1400 | |
| 1450 | |
| 1500 | |
| 1550 | |
| 1600 | |

Elegir un signo para cada sentido y mantenerlo. Estos puntos permiten construir despues una conversion pulso-angulo. No asumir que 1000 y 2000 us equivalen fisicamente a -90 y +90 grados, ni reutilizar la calibracion anterior de 550-1600 us tras modificar el montaje.
