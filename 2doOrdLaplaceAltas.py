import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

# Parámetros del sistema
fs = 1000  # Frecuencia de muestreo
ti = 0  # Tiempo inicial
tf = 0.1  # Tiempo final
t = np.linspace(ti, tf, int(fs), endpoint=False)  # Espacio de datos por llenar del tiempo inicial al final en espacios iguales de la frecuencia de muestreo
u = np.sin(2 * np.pi * 50 * t) + 0.5 * np.sin(2 * np.pi * 300 * t)  # Señal de entrada

# Características del filtro
fc = 200  # Frecuencia de corte en Hz
omega_n = 2 * np.pi * fc  # Frecuencia de corte a rad/s
zeta = 0.5  # Factor de amortiguamiento (damping)
num = [1,0,0]  # Numerador de la FT
den = [1, 2*zeta*omega_n, omega_n**2]  # Denominador de la FT
FT = signal.TransferFunction(num, den)  # Función de transferencia
t_out, y_out, _ = signal.lsim(FT, U=u, T=t)  # Simulación de FT

# Gráfico de la señal de entrada u(t)
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(t, u, label='Entrada $u(t)$')
plt.title('Señal de Entrada')
plt.xlabel('Tiempo [s]')
plt.ylabel('Amplitud')
plt.grid(True)
plt.legend()

# Gráfico de la señal de salida y(t) con el filtro
plt.subplot(2, 1, 2)
plt.plot(t_out, y_out, label='Salida $y(t)$ (Filtrada)', color='orange')
plt.title('Salida del Sistema (Señal Filtrada)')
plt.xlabel('Tiempo [s]')
plt.ylabel('Amplitud')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# Diagrama de Bode
w, mag, phase = signal.bode(FT)
w_hz = w / (2 * np.pi)  # Convertir frecuencia de radianes/s a Hz

plt.figure(figsize=(12, 6))

# Magnitud
plt.subplot(2, 1, 1)
plt.semilogx(w_hz, mag)
plt.axvline(fc, color='r', linestyle='--', label=f'Frecuencia de corte: {fc} Hz')  # Línea vertical en la frecuencia de corte
plt.title('Diagrama de Bode - Magnitud')
plt.xlabel('Frecuencia [Hz]')
plt.ylabel('Magnitud [dB]')
plt.grid(True)
plt.legend()

# Fase
plt.subplot(2, 1, 2)
plt.semilogx(w_hz, phase)
plt.axvline(fc, color='r', linestyle='--', label=f'Frecuencia de corte: {fc} Hz')  # Línea vertical en la frecuencia de corte
plt.title('Diagrama de Bode - Fase')
plt.xlabel('Frecuencia [Hz]')
plt.ylabel('Fase [grados]')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()