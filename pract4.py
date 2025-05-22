import smbus
from time import sleep, time
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# Registros MPU6050
PWR_MGMT_1     = 0x6B
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_XOUT     = 0x3B
ACCEL_YOUT     = 0x3D
ACCEL_ZOUT     = 0x3F
GYRO_XOUT      = 0x43

# Inicialización I2C
bus = smbus.SMBus(1)
Device_Address = 0x68

# Inicializa MPU6050
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0x18)

# Lectura cruda de registros
def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

# Ángulo estimado por acelerómetro (pitch)
def get_accel_phi():
    ay = read_raw_data(ACCEL_YOUT)
    az = read_raw_data(ACCEL_ZOUT)
    phi = np.arctan2(ay, az) * (180.0 / np.pi)
    return phi

# Calibración de offset inicial del acelerómetro
def calibrate_phi(samples=100):
    values = []
    for _ in range(samples):
        values.append(get_accel_phi())
        sleep(0.01)
    return np.mean(values)

# --- Inicialización ---
MPU_Init()
phi_init = calibrate_phi()

x = np.array([[phi_init], [0.0]])  # Estado inicial [phi, dot_phi]
P = np.eye(2)

dt = 0.125  # paso de tiempo (s)
A = np.array([[1, dt],
              [0, 1]])
B = np.array([[0],
              [1]])
H = np.array([[1, 0]])
Q = np.array([[0.001, 0],
              [0, 0.1]])
R = np.array([[0.03]])

# --- Variables para graficar ---
N = 100
time_values = deque(maxlen=N)
phi_values = deque(maxlen=N)
accel_phi_values = deque(maxlen=N)
dot_phi_values = deque(maxlen=N)
gyro_x_values = deque(maxlen=N)

start_time = time()
duration = 10  # segundos

# --- Ciclo principal de estimación ---
while time() - start_time < duration:
    t = time() - start_time

    gyro_x = read_raw_data(GYRO_XOUT) / 16.4
    accel_phi = get_accel_phi()

    # Predicción
    u = np.array([[gyro_x]])
    x = A @ x + B @ u
    P = A @ P @ A.T + Q

    # Actualización
    z = np.array([[accel_phi]])
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(2) - K @ H) @ P

    # Registro de resultados
    phi = x[0, 0]
    dot_phi = x[1, 0]

    time_values.append(t)
    phi_values.append(phi)
    accel_phi_values.append(accel_phi)
    dot_phi_values.append(dot_phi)
    gyro_x_values.append(gyro_x)

    sleep(dt)

# --- Gráficas ---
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time_values, phi_values, label='Ángulo estimado (Kalman)', color='blue')
plt.plot(time_values, accel_phi_values, label='Ángulo del acelerómetro', color='orange', linestyle='--')
plt.title('Comparativa de Ángulo Estimado vs. Acelerómetro')
plt.ylabel('Ángulo (°)')
plt.grid()
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_values, dot_phi_values, label='Velocidad estimada (dot_phi)', color='green')
plt.plot(time_values, gyro_x_values, label='Giroscopio (gyro_x)', color='red', linestyle='--')
plt.title('Comparativa de Velocidad Angular Estimada')
plt.ylabel('Velocidad (°/s)')
plt.xlabel('Tiempo (s)')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
