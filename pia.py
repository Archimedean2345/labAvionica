import serial
import pynmea2
import smbus
import time
import numpy as np
from filterpy.kalman import KalmanFilter

# --- Configuración I2C del MPU-6050 ---
MPU_ADDR = 0x68
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up sensor

# --- Filtro de Kalman (posición 1D) ---
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.x = np.array([[0.], [0.]])  # [posición, velocidad]
kf.F = np.array([[1., 1.],
                 [0., 1.]])
kf.H = np.array([[1., 0.]])
kf.P *= 1000.  # incertidumbre inicial
kf.R = 5       # ruido de medición
kf.Q = np.array([[0.1, 0.1],
                 [0.1, 0.1]])  # ruido del proceso

# --- Funciones para el MPU-6050 ---
def read_word_2c(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

def read_mpu6050_accel_x():
    raw = read_word_2c(0x3B)  # aceleración X
    accel = raw / 16384.0     # de cuenta a g
    return accel * 9.81       # de g a m/s²

# --- Función para leer GPS ---
def read_gps_line(serial_port='/dev/serial0'):
    with serial.Serial(serial_port, baudrate=9600, timeout=1) as ser:
        while True:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    if msg.altitude:  # validación simple
                        return float(msg.latitude), float(msg.longitude), float(msg.altitude)
                except pynmea2.ParseError:
                    continue

# --- Loop principal ---
while True:
    try:
        # Datos GPS
        lat, lon, alt = read_gps_line()
        print(f"GPS: Lat {lat}, Lon {lon}, Alt {alt} m")

        # Aceleración
        acc_x = read_mpu6050_accel_x()
        print(f"Accel X: {acc_x:.2f} m/s^2")

        # Kalman: estimación de altitud filtrada
        kf.predict()
        kf.update(alt)
        print(f"Kalman Altitud Estimada: {kf.x[0][0]:.2f} m\n")

        time.sleep(1)

    except KeyboardInterrupt:
        print("Finalizado por el usuario.")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)
