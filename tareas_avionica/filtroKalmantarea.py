import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete

# ==========================
# Parámetros físicos
# ==========================
m = 1     # masa del péndulo
M = 5     # masa del carro
L = 2     # longitud del péndulo
g = -10   # gravedad negativa
d = 1     # fricción
b = -1    # factor de dirección

# ==========================
# Modelo continuo del sistema
# ==========================
A = np.array([
    [0, 1, 0, 0],
    [0, -d/M, b*m*g/M, 0],
    [0, 0, 0, 1],
    [0, -b*d/(M*L), -b*(m+M)*g/(M*L), 0]
])
B = np.array([[0], [1/M], [0], [b*1/(M*L)]])
C = np.eye(4)  # observamos todos los estados
D = np.zeros((4, 1))

# ==========================
# Discretización
# ==========================
dt = 0.01
t = np.arange(0, 50+dt, dt)
N = len(t)
Ad, Bd, Cd, Dd = cont2discrete((A, B, C, D), dt)[:4]
H = Cd  # observamos todos los estados

# ==========================
# Ruido de proceso y medición
# ==========================
Q = np.eye(4) * 0.01   # incertidumbre del modelo
R = np.eye(4) * 0.1    # incertidumbre de medición (para todos los estados)

# ==========================
# Entradas
# ==========================
u = np.zeros(N)
u[100] = 50 / dt
u[1500] = -50 / dt

# ==========================
# Simulación del sistema real
# ==========================
x_real = np.zeros((4, N))
y_meas = np.zeros((4, N))

# Ruido
w = np.random.multivariate_normal(np.zeros(4), Q, N).T
v = np.random.multivariate_normal(np.zeros(4), R, N).T

for k in range(1, N):
    x_real[:, k] = Ad @ x_real[:, k-1] + Bd.flatten() * u[k-1] + w[:, k-1]
    y_meas[:, k] = H @ x_real[:, k] + v[:, k]

# ==========================
# Filtro de Kalman
# ==========================
x_hat = np.zeros((4, N))
P = np.eye(4)
x_estimates = []

for k in range(1, N):
    # Predicción
    x_pred = Ad @ x_hat[:, k-1] + Bd.flatten() * u[k-1]
    P_pred = Ad @ P @ Ad.T + Q

    # Corrección
    K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
    x_hat[:, k] = x_pred + (K @ (y_meas[:, k] - H @ x_pred)).flatten()
    P = (np.eye(4) - K @ H) @ P_pred

    x_estimates.append(x_hat[:, k])

x_estimates = np.array(x_estimates)

# ==========================
# GRAFICADO
# ==========================

fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

for i in range(4):
    axs[i].plot(t, x_real[i, :], label="Estado real", color="blue")
    axs[i].plot(t[1:], x_estimates[:, i], label="Estadi estimado", color="red", linestyle="--")
    axs[i].set_ylabel(f'Estado {i+1}')
    axs[i].legend()
    axs[i].grid(True)

axs[3].set_xlabel('Tiempo (segundos)')
plt.suptitle("Comparativa entre estado real vs el estimado por filtro de Kalman")
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()