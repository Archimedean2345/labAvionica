import numpy as np
import matplotlib.pyplot as plt

# Parámetros del filtro
fc = 50 # Frecuencia de corte del filtro en Hz
fs = 1000  # Frecuencia de muestreo en Hz
Ts = 1 / fs  # Período de muestreo en segundos
wn = 2 * np.pi * fc  # Frecuencia angular en rad/s
zeta = 0.5  # Coeficiente de amortiguación
tau = 1 / (2 * np.pi * fc)  # Constante de tiempo del filtro

# Rango del giroscopio (°/s) y acelerómetro (g)
gyro_range = 250.0
accel_range = 4.0

# Cargar señales desde el archivo .txt
imu_signals = np.loadtxt('D:/00 Leonardo/avionica/tareas/imusignals.txt')

# Columna 2 - Datos del giroscopio (Datos brutos)
gyro_raw = imu_signals[:, 1]
# Columna 5 y 6 - Datos del acelerómetro (Datos brutos)
accel_rawy = imu_signals[:, 4]
accel_rawz = imu_signals[:, 5]

# Convertir los datos brutos a unidades útiles
gyro_data = ((gyro_raw - 518) / 518) * gyro_range  # Convertir a °/s
accel_datay = ((accel_rawy - 477) / 477) * accel_range  # Convertir a g
accel_dataz = ((accel_rawz - 477) / 477) * accel_range  # Convertir a g

# Calcular el ángulo a partir del giroscopio mediante integración
gyro_angle = np.cumsum(gyro_data) * Ts  # Integral aproximada para obtener el ángulo
# Calcular el ángulo a partir del acelerómetro usando sin⁻¹
accel_angle = np.arcsin(accel_dataz / 1.0) * (180 / np.pi)  # Convertir a grados

# Inicializar las salidas del filtro
gyro_hpf = np.zeros(len(gyro_angle))  # Salida del filtro paso alto para el giroscopio
accel_lpf = np.zeros(len(accel_angle))  # Salida del filtro paso bajo para el acelerómetro

# Aplicar filtro paso alto al ángulo calculado del giroscopio
for k in range(2, len(gyro_angle)):
    gyro_hpf[k] = ((gyro_angle[k] - (2 * gyro_angle[k - 1]) + gyro_angle[k - 2]) +
                    (2 * gyro_hpf[k - 1]) - gyro_hpf[k - 2] +
                    (2 * wn * zeta * Ts * gyro_hpf[k - 1])) / (1 + (2 * Ts * wn * zeta) + (Ts * Ts * wn * wn))

# Aplicar filtro paso bajo al ángulo calculado del acelerómetro
accel_lpf[0] = accel_angle[0]  # Inicializar con el primer valor
for k in range(1, len(accel_angle)):
    accel_lpf[k] = ((Ts * Ts * wn * wn * accel_angle[k]) +
                     (2 * accel_lpf[k - 1]) - accel_lpf[k - 2] +
                     (2 * wn * zeta * Ts * accel_lpf[k - 1])) / (1 + (2 * Ts * wn * zeta) + (Ts * Ts * wn * wn))

# Filtro complementario
alpha_comp = 0.5  # Ajustar la contribución del LPF y HPF
complementary_output = alpha_comp * gyro_hpf + (1 - alpha_comp) * accel_lpf

# Graficar las señales combinadas
plt.figure(figsize=(14, 8))

# Señales del Giroscopio y Acelerómetro Convertidas
plt.subplot(3, 1, 1)
plt.plot(gyro_angle, label='Gyroscope Angle (integrated)', color='black')
plt.plot(accel_angle, label='Accelerometer Angle (sin⁻¹)', color='blue')
plt.title('Ángulos Calculados del Giroscopio y Acelerómetro')
plt.xlabel('Tiempo (muestras)')
plt.ylabel('Ángulo (grados)')
plt.grid(True)
plt.legend()

# Señales Filtradas por LPF y HPF
plt.subplot(3, 1, 2)
plt.plot(accel_lpf, color='black', label='LPF (Acelerómetro)')
plt.plot(gyro_hpf, color='blue', label='HPF (Giroscopio)')
plt.title('Señales Filtradas por LPF y HPF')
plt.xlabel('Tiempo (muestras)')
plt.ylabel('Ángulo (grados)')
plt.grid(True)
plt.legend()

# Señal Filtrada Complementaria
plt.subplot(3, 1, 3)
plt.plot(complementary_output, color='black', label='Filtro Complementario')
plt.title('Señal Filtrada por el Filtro Complementario')
plt.xlabel('Tiempo (muestras)')
plt.ylabel('Ángulo (grados)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()