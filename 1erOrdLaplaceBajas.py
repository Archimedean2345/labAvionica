import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

fs = 10000  # Define la frecuencia de muestreo
ti = 0  # Define el tiempo inicial
tf = 0.1  # Define el tiempo final
fc = 200  # Frecuencia de corte en Hertz

# Convertimos la frecuencia de corte de Hertz a radianes por segundo
wb = 2 * np.pi * fc  # Frecuencia de corte en rad/s

Tf = 1 / wb  # Tiempo característico del filtro
t = np.linspace(ti, tf, int(fs * (tf - ti)), endpoint=False)  # Vector de tiempo

# Señal de entrada: combinación de dos senoidales
u = np.sin(2 * np.pi * 50 * t) + 0.5 * np.sin(2 * np.pi * 300 * t)

# Define el numerador y denominador de la FT
num = np.array([1])
den = np.array([Tf, 1])

# Define la función de transferencia
FT = signal.TransferFunction(num, den)

# Simulación de la respuesta del sistema
t_out, y_out, _ = signal.lsim(FT, U=u, T=t)

# Gráfica de entrada vs salida
plt.figure(figsize=(10, 10))

# Gráfica señal de entrada u(t)
plt.subplot(3, 1, 1)
plt.plot(t, u, label='Entrada $u(t)$')
plt.title('Señal de Entrada')
plt.xlabel('Tiempo [s]')
plt.ylabel('Amplitud')
plt.grid(True)
plt.legend()

# Gráfica señal de salida filtrada
plt.subplot(3, 1, 2)
plt.plot(t_out, y_out, label='Salida $y(t)$ (Filtrada)', color='orange')
plt.title('Salida del Sistema (Señal Filtrada)')
plt.xlabel('Tiempo [s]')
plt.ylabel('Amplitud')
plt.grid(True)
plt.legend()

# Diagrama de Bode (Magnitud y Fase)
w, mag, phase = signal.bode(FT)

# Convertir de radianes/s a Hertz
frecuencia_hz = w / (2 * np.pi)

# Magnitud
plt.subplot(3, 1, 3)
plt.semilogx(frecuencia_hz, mag, label='Magnitud')
plt.axvline(fc, color='red', linestyle='--', label=f'Frecuencia de corte = {fc} Hz')  # Línea en la frecuencia de corte
plt.title('Diagrama de Bode - Magnitud')
plt.xlabel('Frecuencia [Hz]')
plt.ylabel('Magnitud [dB]')
plt.grid(True)
plt.legend()

# Fase
plt.figure(figsize=(10, 6))
plt.semilogx(frecuencia_hz, phase, label='Fase', color='green')
plt.axvline(fc, color='red', linestyle='--', label=f'Frecuencia de corte = {fc} Hz')  # Línea en la frecuencia de corte
plt.title('Diagrama de Bode - Fase')
plt.xlabel('Frecuencia [Hz]')
plt.ylabel('Fase [grados]')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()