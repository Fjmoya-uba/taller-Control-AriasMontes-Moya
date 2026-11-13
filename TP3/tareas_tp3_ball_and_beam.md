# Trabajo Práctico 3 - Control digital avanzado
**Facultad de Ingeniería, Universidad de Buenos Aires**  
*Laboratorio de Control Automático (86.22) - Taller de Automatización y Control (TA135)*  
*Dr. Ing. Claudio D. Pose*

--- 

## 2 Tareas

### 2.1 Modelado
* Se considerará el sistema sensado y controlado a $50\text{ Hz}$.
* Se define la entrada del sistema como la señal de comando $u$, y las salidas como el ángulo de la barra $\theta$ y la posición del carro $p$, las cuales también pueden considerarse variables de estado del sistema.
* Utilice exactamente las mismas convenciones que para el trabajo anterior, en cuanto a la definición de las ternas, la dirección positiva de cada uno y la definición del cero.
* Definir una representación en variables de estado, linealizada en torno al punto de equilibrio donde el carro está en posición cero y la barra en ángulo cero, que se denominará el punto de equilibrio.
* Se puede asumir que las variables de estado serán las mencionadas, junto con sus derivadas: $[p, \dot{p}, \theta, \dot{\theta}]^T$. Si bien se puede obtener cualquier otra representación en variables de estado, se sugiere utilizar esas cuatro variables para facilitar la etapa de diseño.

---

### 2.2 Observador
* Implementar un observador de Luenberger completo, considerando las cuatro variables de estado $[p, \dot{p}, \theta, \dot{\theta}]^T$.
* Como medición se consideran las salidas $y = [p, \theta]^T$.
* **Análisis de observabilidad:**
  * Compruebe la observabilidad del sistema bajo estas condiciones.
  * ¿Sigue siendo observable si sólo se mide la posición del carro?
  * ¿Y si sólo se mide el ángulo de la barra?
* **Prueba y validación:**
  * Para probar y demostrar el correcto funcionamiento, genere una secuencia de comandos tipo escalón para $u$ (por ejemplo, una referencia de $10^\circ$ durante unos instantes, luego otra referencia de $-15^\circ$ durante otro instante, y así sucesivamente) para mover el sistema de diferentes maneras (cuidando que el carro no llegue a los extremos).
  * Grafique, comparando, las mediciones reales de las variables de estado que pueda medir, contra la estimación de los estados correspondiente mediante simulación.
  * Indique por qué diseñó el observador de la manera en la que lo haya hecho, y qué consideraciones tuvo al respecto.

---

### 2.3 Diseño de controlador
* **Regulador sin referencia (estabilización en el punto de equilibrio):**
  * Diseñar un controlador por realimentación de estados que estabilice al sistema en torno al punto de equilibrio, sin la posibilidad de darle una referencia de ningún tipo. Esto implica poner a funcionar un bloque de realimentación de estados que opere correctamente, sin considerar la matriz de *feedforward*.
  * Para este sistema, grafique la respuesta temporal de todas las variables de estado del sistema ante una perturbación tipo impulso al carro (golpee el carro lo más instantáneamente posible hacia un lado y hacia el otro).
  * Compare contra la respuesta simulada; para ello, inyecte una perturbación tipo impulso que logre un apartamiento máximo similar en el desplazamiento del carro.

* **Seguimiento de referencias:**
  * Agregue al controlador la capacidad de seguimiento de referencias, para poder comandar el ángulo $\theta$ y la posición $p$ al mismo tiempo.
  * Grafique la respuesta temporal de todas las variables de estado del sistema ante una secuencia de comandos tipo escalón para $p_{\text{ref}}$, con $\theta_{\text{ref}} = 0$.

* **Control con acción integral:**
  * Reformule el control por realimentación de estados para agregar también acción integral, pero en este caso, sólo se desea controlar la posición $p$ del carrito (aún se mide el ángulo para el observador, pero no se generan comandos de referencia para la barra).
  * **Requerimientos de desempeño:**
    * Tiempo de establecimiento: $t_s < 2.5\text{ s}$
    * Sobrepico: $M_p < 15\%$
  * Repita los últimos experimentos para este caso.

* **Nota general:** Si en cualquier momento se requiere reformular el observador de estados, hágalo, e indique por qué es necesario y bajo qué consideraciones se propone el nuevo observador.