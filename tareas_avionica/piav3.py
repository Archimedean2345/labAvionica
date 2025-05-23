import smbus
import time
import numpy as np
import serial
import matplotlib.pyplot as plt
from math import sin, cos, pi

# -------------------------------
# CONFIGURACIÓN MPU6050
# -------------------------------
bus = smbus.SMBus(1)
mpu_addr = 0x68
bus.write_byte_data(mpu_addr, 0x6B, 0)

def read_word(reg):
    high = bus.read_byte_data(mpu_addr, reg)
    low = bus.read_byte_data(mpu_addr, reg+1)
    val = (high << 8) + low
    return val - 65536 if val > 32768 else val

def get_acc():
    acc_x = read_word(0x3B) / 16384.0
    acc_y = read_word(0x3D) / 16384.0
    return acc_x * 9.81, acc_y * 9.81

# -------------------------------
# CONFIGURACIÓN GPS
# -------------------------------
gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

def parse_latlon(nmea):
    try:
        parts = nmea.split(",")

        # $GPRMC
        if parts[0].endswith("RMC") and parts[2] == "A":
            lat = float(parts[3])
            lon = float(parts[5])
            lat = int(lat/100) + (lat % 100)/60
            lon = int(lon/100) + (lon % 100)/60
            if parts[4] == "S": lat *= -1
            if parts[6] == "W": lon *= -1
            return lat, lon

        # $GPGLL
        elif parts[0].endswith("GLL") and parts[6] == "A":
            lat = float(parts[1])
            lon = float(parts[3])
            lat = int(lat/100) + (lat % 100)/60
            lon = int(lon/100) + (lon % 100)/60
            if parts[2] == "S": lat *= -1
            if parts[4] == "W": lon *= -1
            return lat, lon
    except:
        pass
    return None, None

def latlon_to_xy(lat, lon, lat0, lon0):
    R = 6371000  # Radio de la Tierra en metros
    dlat = (lat - lat0) * pi / 180
    dlon = (lon - lon0) * pi / 180
    x = R * dlon * cos(lat0 * pi / 180)
    y = R * dlat
    return x, y

# -------------------------------
# FILTRO KALMAN 2D
# -------------------------------
dt = 0.1
A = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
B = np.array([[0.5*dt**2, 0],
              [0, 0.5*dt**2],
              [dt, 0],
              [0, dt]])
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])
Q = np.eye(4) * 0.1
R = np.eye(2) * 4.0
x_k = np.zeros((4,1))
P = np.eye(4)

# -------------------------------
# LOOP PRINCIPAL
# -------------------------------
print("Iniciando fusión de sensores con Kalman...")
data = []
t0 = time.time()

lat0, lon0 = None, None

while time.time() - t0 < 30:
    acc_x, acc_y = get_acc()
    u = np.array([[acc_x], [acc_y]])

    # Predicción
    x_k = A @ x_k + B @ u
    P = A @ P @ A.T + Q

    gps_x, gps_y = None, None

    try:
        line = gps.readline().decode(errors='ignore').strip()
        lat, lon = parse_latlon(line)

        if lat and lon:
            if lat0 is None:
                lat0, lon0 = lat, lon
            gps_x, gps_y = latlon_to_xy(lat, lon, lat0, lon0)
            z = np.array([[gps_x], [gps_y]])

            # Corrección
            y_k = z - H @ x_k
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x_k = x_k + K @ y_k
            P = (np.eye(4) - K @ H) @ P

    except Exception as e:
        print("Error GPS:", e)

    # Guardar datos
    data.append([time.time()-t0, x_k[0,0], x_k[1,0], acc_x, acc_y, gps_x, gps_y])
    time.sleep(dt)

# -------------------------------
# GUARDAR CSV
# -------------------------------
with open("kalman_posicion.csv", "w") as f:
    f.write("tiempo,x_est,y_est,acc_x,acc_y,x_gps,y_gps\n")
    for row in data:
        f.write(",".join([f"{v:.4f}" if v is not None else "NaN" for v in row]) + "\n")

print("¡Datos guardados en kalman_posicion.csv!")

# -------------------------------
# GRAFICAR
# -------------------------------
x_kalman = [row[1] for row in data]
y_kalman = [row[2] for row in data]
x_gps = [row[5] for row in data if row[5] is not None]
y_gps = [row[6] for row in data if row[6] is not None]

plt.figure(figsize=(8,6))
plt.plot(x_kalman, y_kalman, label="Estimación Kalman", linewidth=2)
plt.scatter(x_gps, y_gps, color='red', label="GPS", alpha=0.7)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Trayectoria: GPS + IMU (Kalman)")
plt.grid(True)
plt.legend()
plt.axis("equal")
plt.tight_layout()
plt.show()
