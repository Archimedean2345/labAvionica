import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete

# Parámetros
m = 1
M = 5
L = 2
g = -10
d = 1
b = -1

# Sistema continuo
A = np.array([
    [0, 1, 0, 0],
    [0, -d/M, b*m*g/M, 0],
    [0, 0, 0, 1],
    [0, -b*d/(M*L), -b*(m+M)*g/(M*L), 0]
])
B = np.array([[0], [1/M], [0], [b*1/(M*L)]])
C = np.array([[1, 0, 0, 0]])
D = np.array([[0]])

# Discretización
dt = 0.01
t = np.arange(0, 50+dt, dt)
N = len(t)

Ad, Bd, Cd, Dd = cont2discrete((A, B, C, D), dt)[:4]
H = Cd

# Ruido
Q = np.eye(4) * 0.01
R = 1

# Entrada
u = np.zeros(N)
u[100] = 50 / dt
u[1500] = -50 / dt

# Ruido de proceso y medición
w = np.random.multivariate_normal(np.zeros(4), Q, N).T
v = np.sqrt(R) * np.random.randn(N)

# Simulación del sistema real
x_real = np.zeros((4, N))
y_meas = np.zeros(N)

for k in range(1, N):
    x_real[:, k] = Ad @ x_real[:, k-1] + Bd.flatten() * u[k-1] + w[:, k-1]
    y_meas[k] = H @ x_real[:, k] + v[k]

# Filtro de Kalman
x_hat = np.zeros((4, N))
P_hist = np.zeros((4, 4, N))
P_pred_hist = np.zeros((4, 4, N))
z_hist = np.zeros(N)

P = np.eye(4)

for k in range(1, N):
    x_pred = Ad @ x_hat[:, k-1] + Bd.flatten() * u[k-1]
    P_pred = Ad @ P @ Ad.T + Q

    z = y_meas[k]
    K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
    x_hat[:, k] = x_pred + (K @ (z - H @ x_pred)).flatten()
    P = (np.eye(4) - K @ H) @ P_pred

    P_hist[:, :, k] = P
    P_pred_hist[:, :, k] = P_pred
    z_hist[k] = z

# === GRAFICADO ===

# 1. Estados reales vs estimados
plt.figure(figsize=(12, 8))
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.plot(t, x_real[i, :], label=f'x_real[{i}]')
    plt.plot(t, x_hat[i, :], '--', label=f'x_hat[{i}]')
    plt.xlabel('Tiempo (s)')
    plt.ylabel(f'Estado {i}')
    plt.title(f'Estado {i} real vs estimado')
    plt.legend()
    plt.grid(True)

plt.tight_layout()
plt.show()

# 2. Medición y estimación (salida observada)
plt.figure(figsize=(10, 4))
plt.plot(t, y_meas, 'k:', label='y_meas (medición)')
plt.plot(t, z_hist, 'r--', label='z (usado en el filtro)')
plt.xlabel('Tiempo (s)')
plt.ylabel('Medición / z')
plt.title('Medición y entrada al filtro')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
