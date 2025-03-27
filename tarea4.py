import numpy as np
import matplotlib.pyplot as plt

def y(t):
    """Calcula el valor de y(t)."""
    return (5/4) * (1 - np.exp(-4*t) * (1 + 4*t))

# Generar valores de t
t_values = np.linspace(0, 5, 500)  # De 0 a 5 segundos, 500 puntos

# Calcular valores de y(t)
y_values = y(t_values)

# Graficar
plt.figure(figsize=(10, 6))
plt.plot(t_values, y_values)
plt.title('Gráfica de y(t) = (5/4)(1 - e^(-4t)(1 + 4t))')
plt.xlabel('Tiempo (t)')
plt.ylabel('y(t)')
plt.grid(True)
plt.show()