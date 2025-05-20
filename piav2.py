import serial
import pynmea2
import smbus
import time
import numpy as np
from filterpy.kalman import KalmanFilter

# --- Configuración del MPU-6050 ---
MPU_ADDR = 0x68
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU

# --- Filtro de Kalman para altitud (posición 1D) ---
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.x = np.array([[0.], [0.]])
kf.F = np.array([[1., 1.], [0., 1.]])
kf.H = np.array([[1., 0.]])
kf.P *= 1000.
kf.R = 5
kf.Q = np.array([[0.1, 0.1], [0.1, 0.1]])

# --- Puerto serie (global para recuperación) ---
ser = None

def open_gps_port():
    try:
        return serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
    except Exception as e:
        print(f"❌ Error abriendo /dev/serial0: {e}")
        return None

ser = open_gps_port()

# --- Funciones del MPU-6050 ---
def read_word_2c(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def read_mpu6050_accel_x():
    raw = read_word_2c(0x3B)
    accel = raw / 16384.0  # en g
    return accel * 9.81    # en m/s²

# --- Lectura GPS con recuperación ---
def read_gps_line():
    global ser
    if ser is None:
        ser = open_gps_port()
        if ser is None:
            time.sleep(2)
            return None, None, None

    try:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$GPGGA'):
            msg = pynmea2.parse(line)
            if msg.altitude:
                return float(msg.latitude), float(msg.longitude), float(msg.altitude)
    except pynmea2.ParseError:
        return None, None, None
    except Exception as e:
        print(f"Error GPS: {e}")
        try:
            ser.close()
        except:
            pass
        ser = None  # Forzar reapertura en siguiente ciclo
    return None, None, None

# --- Bucle principal ---
print("📡 Iniciando lectura GPS + IMU con filtro de Kalman...\n")
while True:
    try:
        lat, lon, alt = read_gps_line()
        if lat is None:
            print("Esperando datos GPS...")
            time.sleep(1)
            continue

        print(f"GPS: Lat {lat}, Lon {lon}, Alt {alt:.1f} m")

        acc_x = read_mpu6050_accel_x()
        print(f"Accel X: {acc_x:.2f} m/s^2")

        kf.predict()
        kf.update(alt)
        print(f"Kalman Altitud Estimada: {kf.x[0][0]:.2f} m\n")

        time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 Finalizado por el usuario.")
        break
    except Exception as e:
        print(f"⚠️ Error general: {e}")
        time.sleep(1)
