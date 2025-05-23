import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import StateSpace, step

# Parametros del sistema 
m =1
M= 5
L = 2
g = -10
d = 1
b = -1

# Definimos las matrices del sistema
A = np.array([
    [0, 1, 0, 0],
    [0, -d/M, b*m*g/M, 0],
    [0, 0, 0, 1],
    [0, -b*d/(M*L), -b*(m+M)*g/(M*L), 0]
])

B = np.array([ [0], [1/M], [0], [b*1/(M*L)]])

# El sistema tiene todos los estados como salidas
C = np.array([[1,0,0,0]])
print(C)
D = np.array([[0]])

# Configuramos el sistema para espacio de estados
system = StateSpace(A, B, C, D)

# Simulamos la respuesta al escalón
time = np.linspace(0, 200, 5000)  # Definimos el tiempo de simulación
t, y = step(system, T=time)

# Matriz de observación H: mide solo u y w (estados 1 y 2)
H = np.array([
    [1, 0, 0, 0],  # u
    [0, 0, 0, 0],   # w
])  # H: 2x4

# Ruido de proceso (modelo) y de medición (sensor)
Q = np.eye(4) * 0.001       # Incertidumbre del modelo
R = np.eye(2) * 0.01        # Incertidumbre de medición (2 variables medidas)

# Estado estimado inicial y covarianza
x_est = np.zeros((4, 1))
P_est = np.eye(4)

x_estimates = []

# -------------------------------
# 3. Filtro de Kalman
# -------------------------------
for i in range(len(time)):
    u = np.array([[1]]) if i > 0 else np.array([[0]])

    # Predicción
    x_pred = A @ x_est + B @ u
    P_pred = A @ P_est @ A.T + Q

    # Medición simulada con ruido en u y w
    z = np.array([
        [y[i, 0] + np.random.normal(0, np.sqrt(R[0, 0]))],  # u
        [y[i, 1] + np.random.normal(0, np.sqrt(R[1, 1]))]   # w
    ])  # z: 2x1

    # Corrección
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_est = x_pred + K @ (z - H @ x_pred)
    P_est = (np.eye(4) - K @ H) @ P_pred

    x_estimates.append(x_est.flatten())

x_estimates = np.array(x_estimates)

# Graficamos la comparación de los estados verdaderos y estimados con los colores solicitados
fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

for i in range(4):
    axs[i].plot(t, y[:, i], label="True State", color="blue")
    axs[i].plot(t, x_estimates[:, i], label="Estimated State", color="red", linestyle="--")
    axs[i].set_ylabel(f'State {i+1}')
    axs[i].legend()

axs[3].set_xlabel('Time (seconds)')
plt.suptitle("Comparison of True and Estimated States with Kalman Filter")
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
