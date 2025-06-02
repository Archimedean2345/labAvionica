import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Transformer

# === Leer archivo ===
df = pd.read_csv("datosFusionados_IMU_GPS_con_ruido.csv")

# === Configurar origen para coordenadas ENU ===
lat0 = df['latitude_interp'].iloc[0]
lon0 = df['longitude_interp'].iloc[0]
transformer = Transformer.from_crs("epsg:4326", f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +ellps=WGS84", always_xy=True)

# Convertir coordenadas geográficas a locales ENU
east, north = transformer.transform(df['longitude_noisy'].values, df['latitude_noisy'].values)
east_ref, north_ref = transformer.transform(df['longitude_interp'].values, df['latitude_interp'].values)

# === Filtro de Kalman 1D ===
class KalmanFilter1D:
    def __init__(self, initial_pos, initial_vel, pos_var, accel_var):
        self.x = np.array([[initial_pos], [initial_vel]])
        self.P = np.eye(2)
        self.F = np.eye(2)
        self.H = np.array([[1, 0]])
        self.R = np.array([[pos_var]])
        self.Q = np.array([[0.25*accel_var, 0.5*accel_var],
                           [0.5*accel_var, accel_var]])
        self.I = np.eye(2)

    def predict(self, dt):
        self.F = np.array([[1, dt], [0, 1]])
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0, 0]

# === Parámetros del filtro ===
pos_var = 4       # ruido GPS en metros^2
accel_var = 0.5   # aceleración asumida

kf_e = KalmanFilter1D(east[0], 0, pos_var, accel_var)
kf_n = KalmanFilter1D(north[0], 0, pos_var, accel_var)

filtered_east = []
filtered_north = []

for i in range(len(df)):
    dt = 0.4 if i == 0 else df['tiempo'].iloc[i] - df['tiempo'].iloc[i-1]

    kf_e.predict(dt)
    kf_e.update(np.array([[east[i]]]))
    filtered_east.append(kf_e.get_position())

    kf_n.predict(dt)
    kf_n.update(np.array([[north[i]]]))
    filtered_north.append(kf_n.get_position())

# === Gráfica 1: Trayectoria GPS ===
plt.figure(figsize=(10, 6))
plt.plot(east_ref, north_ref, color='blue', linewidth=2.5, label='GPS filtrado con Kalman', zorder=2)
plt.scatter(east, north, color='red', alpha=0.6, s=10, label='GPS señal medida', zorder=1)
plt.xlabel("Este (m)")
plt.ylabel("Norte (m)")
plt.title("Trayectoria GPS: Medida vs Filtrada")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()

# === Gráfica 2: Aceleraciones IMU ===
plt.figure(figsize=(10, 6))
plt.plot(df['tiempo'], df['acc_x'], label='Acc X')
plt.plot(df['tiempo'], df['acc_y'], label='Acc Y')
plt.plot(df['tiempo'], df['acc_z'], label='Acc Z')
plt.xlabel("Tiempo (s)")
plt.ylabel("Aceleración (m/s²)")
plt.title("Aceleraciones medidas por el IMU")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


