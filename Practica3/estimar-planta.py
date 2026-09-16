import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

# ============================================================
# 1. CARGAR DATOS DESDE MATLAB
# ============================================================

datos = loadmat("mediciones_y_u.mat")

# Para ver qué variables contiene el archivo:
print(datos.keys())

y = np.squeeze(datos[0]).astype(float)
u = np.squeeze(datos[1]).astype(float)
t = np.squeeze(datos[2]).astype(float)

Ts = np.mean(np.diff(t))

print(f"Ts = {Ts:.6f} s")
print(f"Muestras de y: {len(y)}")
print(f"Muestras de u: {len(u)}")

# ============================================================
# 2. ARMAR LA REGRESION
#
# Modelo:
#
# y[k] = a1*y[k-1] + a2*y[k-2] + b0*u[k]
#
# Entonces:
#
# Y = X theta
#
# theta = [a1, a2, b0]^T
# ============================================================

Y = y[2:]

X = np.column_stack((
    y[1:-1],   # y[k-1]
    y[:-2],    # y[k-2]
    u[2:]      # u[k]
))


# ============================================================
# 3. MINIMOS CUADRADOS
# ============================================================

theta, residuals, rank, singular_values = np.linalg.lstsq(
    X, Y, rcond=None
)

a1, a2, b0 = theta

print("\n--- Modelo discreto estimado ---")
print(f"a1 = {a1}")
print(f"a2 = {a2}")
print(f"b0 = {b0}")

print(
    f"\ny[k] = ({a1:.6f}) y[k-1] "
    f"+ ({a2:.6f}) y[k-2] "
    f"+ ({b0:.6f}) u[k]"
)


# ============================================================
# 4. PASAR DE a1,a2,b0 A A,B,C
#
# Planta continua:
#
#            C
# G(s) = -----------
#         s² + As + B
#
# donde:
# A = p1 + p2
# B = p1*p2
# C = p1*p2*c
# ============================================================

D = -1 / a2

A = (a1 * D - 2) / Ts

B = (D - 1 - A * Ts) / Ts**2

C = b0 * D / Ts**2

print("\n--- Coeficientes continuos ---")
print(f"A = p1 + p2   = {A}")
print(f"B = p1*p2     = {B}")
print(f"C = p1*p2*c   = {C}")


# ============================================================
# 5. OBTENER p1, p2 y c
#
# p1 y p2 son las raices de:
#
# p² - A*p + B = 0
# ============================================================

discriminante = A**2 - 4*B

if discriminante < 0:
    print("\nATENCION:")
    print("El discriminante es negativo.")
    print("Los polos obtenidos no corresponden a dos p reales.")
    print("Revisar modelo, datos o discretizacion.")

p1 = (A + np.sqrt(complex(discriminante))) / 2
p2 = (A - np.sqrt(complex(discriminante))) / 2

c = C / B

print("\n--- Parametros de la planta ---")
print(f"p1 = {p1}")
print(f"p2 = {p2}")
print(f"c  = {c}")


# ============================================================
# 6. PREDICCION DE UN PASO
#
# Usamos los y medidos anteriores para predecir y[k].
# Esto sirve para verificar la regresion.
# ============================================================

y_pred = a1*y[1:-1] + a2*y[:-2] + b0*u[2:]


# ============================================================
# 7. GRAFICAR MEDICION VS PREDICCION
# ============================================================

plt.figure()

plt.plot(t[2:], y[2:], label="Medido")
plt.plot(t[2:], y_pred, "--", label="Predicción")

plt.xlabel("Tiempo [s]")
plt.ylabel("Ángulo")
plt.grid()
plt.legend()
plt.title("Salida medida vs modelo discreto")

plt.show()