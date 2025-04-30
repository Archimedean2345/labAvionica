import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import bode, TransferFunction

# Parámetros del sistema
zeta = 1            # Factor de amortiguamiento
omega_n_hz = 200       # Frecuencia natural en Hertz
omega_n = 2 * np.pi * omega_n_hz  # Convertir a rad/s
h = 0.00001              # Paso de tiempo
ti = 0                   # Tiempo inicial
tf = 0.1                 # Tiempo final

# Definir la señal de entrada u(t)
u = lambda t: np.sin(2 * np.pi * 50 * t) + 0.5 * np.sin(2 * np.pi * 300 * t)  # Escalar entrada

# Crear el vector de tiempo
t = np.arange(ti, tf, h)

# Inicializar los vectores de salida y entrada
y = np.zeros_like(t)
u_t = u(t)

# Inicializar las condiciones iniciales
y[0] = 0
y[1] = 0

# Resolver la ecuación con diferencias finitas hacia atrás
for i in range(2, len(t)):
    y[i] = ((2 + 2 * zeta * omega_n * h) * y[i-1] - y[i-2] + u_t[i] - 2 * u_t[i-1] + u_t[i-2]) / (1 + 2 * zeta * omega_n * h + omega_n**2 * h**2)

# Graficar la señal de entrada y la salida
plt.figure(figsize=(12, 8))

# Graficar la entrada u(t)
plt.subplot(3, 1, 1)
plt.plot(t, u_t, label='Entrada u(t)', linestyle='--', color='b')
plt.title('Entrada y Salida del Sistema')
plt.ylabel('Amplitud')
plt.legend()
plt.grid()
plt.autoscale()  # Ajustar automáticamente los ejes

# Graficar la salida y(t)
plt.subplot(3, 1, 2)
plt.plot(t, y, label='Salida y(t)', color='r')
plt.xlabel('Tiempo (s)')
plt.ylabel('Amplitud')
plt.legend()
plt.grid()
plt.ylim([-0.6, 0.6])  # Ajustar límites de y para mejor visualización
plt.autoscale()  # Ajustar automáticamente los ejes

# Definir la función de transferencia del sistema
num = [1, 0]  # Numerador: s^2
den = [1, 2*zeta*omega_n, omega_n**2]  # Denominador: s^2 + 2ζω_n s + ω_n^2
system = TransferFunction(num, den)

# Generar el diagrama de Bode
w, mag, phase = bode(system)

# Convertir la frecuencia a Hertz
w_hz = w / (2 * np.pi)

# Considerar la frecuencia de corte como la frecuencia natural
cutoff_freq_hz = omega_n_hz  # Frecuencia de corte en Hz

# Graficar el diagrama de Bode - Magnitud
plt.subplot(3, 1, 3)
plt.semilogx(w_hz, mag, label='Magnitud (dB)', color='g')
plt.axvline(x=cutoff_freq_hz, color='orange', linestyle='--', label=f'Frecuencia de Corte: {cutoff_freq_hz:.2f} Hz')
plt.title('Diagrama de Bode')
plt.ylabel('Magnitud (dB)')
plt.legend()
plt.grid()
plt.autoscale()  # Ajustar automáticamente los ejes

# Graficar el diagrama de Bode - Fase
plt.figure(figsize=(10, 4))
plt.semilogx(w_hz, phase, label='Fase (grados)', color='m')
plt.axvline(x=cutoff_freq_hz, color='orange', linestyle='--', label=f'Frecuencia de Corte: {cutoff_freq_hz:.2f} Hz')
plt.xlabel('Frecuencia (Hz)')  # Cambiar etiqueta a Hz
plt.ylabel('Fase (grados)')
plt.legend()
plt.grid()
plt.autoscale()  # Ajustar automáticamente los ejes

plt.tight_layout()
plt.show()