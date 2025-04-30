import numpy as np
import matplotlib.pyplot as plt
# Solicitar la frecuencia de corte al usuario
fc = 200  # Frecuencia de corte en Hz
# Calcular tau a partir de la frecuencia de corte
tau = 1 / (2 * np.pi * fc)
# Mostrar la frecuencia de corte calculada
print(f"Frecuencia de corte definida: {fc:.2f} Hz")
# Parámetros
yi = 0   # Condición inicial
ti = 0   # Tiempo inicial
tf = 0.1 # Tiempo final para mostrar varias ondas
h = 0.0001  # Paso de tiempo
# Definir la señal de entrada como una combinación de senoidales
u = lambda t: np.sin(2 * np.pi * 50 * t) + 0.5 * np.sin(2 * np.pi * 300 * t)
# Crear el vector de tiempo
t = np.arange(ti, tf, h)
# Inicializar el vector de salida
y = np.zeros_like(t)
# Resolución por diferencias finitas hacia atrás (Filtro pasa-bajas)
for i in range(1, len(t)):
    y[i] = (h * u(t[i]) + tau * y[i-1]) / (tau + h)
# Graficar la señal de entrada y la salida filtrada
plt.figure(figsize=(10, 6))
# Graficar la entrada u(t)
plt.subplot(2, 1, 1)
plt.plot(t, u(t), label='Entrada u(t)', linestyle='--', color='b')
plt.title('Entrada y Salida Filtrada')
plt.ylabel('Amplitud')
plt.legend()
plt.grid()
# Graficar la salida y(t)
plt.subplot(2, 1, 2)
plt.plot(t, y, label='Salida y(t)', color='r')
plt.xlabel('Tiempo (s)')
plt.ylabel('Amplitud')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()
# --- Diagrama de Bode ---
# Definir la frecuencia angular (en radianes/segundo)
frecuencias = np.logspace(1, 5, 500)  # Rango de frecuencias de 10 Hz a 100 kHz
omega = 2 * np.pi * frecuencias  # Convertir a radianes/segundo
# Función de transferencia en el dominio de la frecuencia para un filtro pasa-bajas
H = 1 / (1 + 1j * omega * tau)
# Magnitud en dB
magnitude_dB = 20 * np.log10(np.abs(H))
# Fase en grados
fase = np.angle(H, deg=True)
# Graficar el diagrama de Bode (magnitud y fase)
plt.figure(figsize=(10, 6))
# Gráfico de la magnitud (diagrama de Bode)
plt.subplot(2, 1, 1)
plt.semilogx(frecuencias, magnitude_dB, label='Magnitud', color='b')
plt.axvline(fc, color='r', linestyle='--', label=f'Frecuencia de corte = {fc:.2f} Hz')
plt.title('Diagrama de Bode')
plt.ylabel('Magnitud (dB)')
plt.grid(True)
plt.legend()
# Gráfico de la fase
plt.subplot(2, 1, 2)
plt.semilogx(frecuencias, fase, label='Fase', color='g')
plt.axvline(fc, color='r', linestyle='--', label=f'Frecuencia de corte = {fc:.2f} Hz')
plt.xlabel('Frecuencia (Hz)')
plt.ylabel('Fase (grados)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()