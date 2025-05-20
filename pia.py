# Codigo actualizado rev2
import serial
import pynmea2
import smbus
import time
import numpy as np
from filterpy.kalman import KalmanFilter

# --- Configuración del MPU-6050 ---
MPU_ADDR = 0x68
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Activar el sensor

# --- Configuración del filtro de Kalman ---
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.x = np.array([[0.], [0.]])  # estado inicial: [posición, velocidad]
kf.F = np.array([[1., 1.], [0., 1.]])  # modelo de transición
kf.H = np.array([[1., 0.]])           # modelo de medición
kf.P *= 1000.  # incertidumbre inicial
kf.R = 5       # ruido de medición (GPS)
kf.Q = np.array([[0.1, 0.1], [0.1, 0.1]])  # ruido del proceso

# --- Abrimos el puerto serie del GPS una sola vez ---
try:
    ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
except Exception as e:
    print(f"Error abriendo el puerto serial: {e}")
    exit(1)

# --- Funciones MPU-6050 ---
def read_word_2c(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

def read_mpu6050_accel_x():
    raw = read_word_2c(0x3B)  # registro aceleración en X
    accel = raw / 16384.0     # a G
    return accel * 9.81       # a m/s²

# --- Función para leer datos GPS GPGGA ---
def read_gps_line():
    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                if msg.altitude:
                    return float(msg.latitude), float(msg.longitude), float(msg.altitude)
        except pynmea2.ParseError:
            continue
        except Exception as e:
            print(f"Error GPS: {e}")
            return None, None, None

# --- Loop principal ---
print("Iniciando lectura GPS + IMU con filtro de Kalman...\n")
while True:
    try:
        lat, lon, alt = read_gps_line()
        if lat is None:
            continue  # salto si hubo error de lectura

        print(f"GPS: Lat {lat}, Lon {lon}, Alt {alt:.1f} m")

        acc_x = read_mpu6050_accel_x()
        print(f"Accel X: {acc_x:.2f} m/s^2")

        kf.predict()
        kf.update(alt)
        print(f"Kalman Altitud Estimada: {kf.x[0][0]:.2f} m\n")

        time.sleep(1)

    except KeyboardInterrupt:
        print("\nFinalizado por el usuario.")
        break
    except Exception as e:
        print(f"Error general: {e}")
        time.sleep(1)
