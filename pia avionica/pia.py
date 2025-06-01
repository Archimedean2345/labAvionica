import smbus
import math
import time
import numpy as np
import serial
import pynmea2
import csv
class MPU6050:
    def _init_(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.initialize()
    def initialize(self):
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low
        return value - 65536 if value > 32768 else value
    def get_gyro_data(self):
        x = self.read_raw_data(self.GYRO_XOUT_H) / 131.0
        y = self.read_raw_data(self.GYRO_XOUT_H + 2) / 131.0
        z = self.read_raw_data(self.GYRO_XOUT_H + 4) / 131.0
        return np.radians([x, y, z])
    def get_accel_data(self):
        x = self.read_raw_data(self.ACCEL_XOUT_H) / 16384.0
        y = self.read_raw_data(self.ACCEL_XOUT_H + 2) / 16384.0
        z = self.read_raw_data(self.ACCEL_XOUT_H + 4) / 16384.0
        return x, y, z
class KalmanFilter1D:
    def _init_(self):
        self.x = np.array([[0], [0]])  # [angle, rate]
        self.P = np.eye(2)
        self.Q = np.array([[1e-5, 0], [0, 1e-3]])
        self.R = np.array([[0.03]])
        self.H = np.array([[1, 0]])
        self.I = np.eye(2)
    def predict(self, u, dt):
        A = np.array([[1, -dt], [0, 1]])
        B = np.array([[dt], [0]])
        self.x = A @ self.x + B * u
        self.P = A @ self.P @ A.T + self.Q
    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
    def get_angle(self):
        return self.x[0, 0]
class MPUAngleCalculator:
    def _init_(self):
        self.mpu = MPU6050()
        self.last_time = time.time()
        self.kalman_pitch = KalmanFilter1D()
        self.kalman_roll = KalmanFilter1D()
    def update_kalman(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        p, q, r = self.mpu.get_gyro_data()
        ax, ay, az = self.mpu.get_accel_data()
        acc_pitch = math.atan2(-ax, math.sqrt(ay*2 + az*2))
        acc_roll = math.atan2(ay, az)
        self.kalman_pitch.predict(q, dt)
        self.kalman_pitch.update(acc_pitch)
        self.kalman_roll.predict(p, dt)
        self.kalman_roll.update(acc_roll)
        return (
            math.degrees(self.kalman_pitch.get_angle()),
            math.degrees(acc_pitch),
            math.degrees(q),
            math.degrees(self.kalman_roll.get_angle()),
            math.degrees(acc_roll),
            math.degrees(p)
        )
def read_gps(serial_port):
    try:
        line = serial_port.readline().decode("utf-8", errors="ignore")
        if line.startswith('$GPGGA'):
            msg = pynmea2.parse(line)
            return msg.latitude, msg.longitude
    except Exception as e:
        print("GPS read error:", e)
    return None, None
def main():
    try:
        gps_port = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
        calculator = MPUAngleCalculator()
        pitch_kalman_list = []
        pitch_accel_list = []
        pitch_rate_list = []
        roll_kalman_list = []
        roll_accel_list = []
        roll_rate_list = []
        gps_lat_list = []
        gps_lon_list = []
        time_list = []
        t0 = time.time()
        print("Starting Kalman filter with GPS tracking...")
        while time.time() - t0 < 60:  # run for 1 minute
            pitch_kalman, pitch_acc, pitch_rate, roll_kalman, roll_acc, roll_rate = calculator.update_kalman()
            lat, lon = read_gps(gps_port)
            pitch_kalman_list.append(pitch_kalman)
            pitch_accel_list.append(pitch_acc)
            pitch_rate_list.append(pitch_rate)
            roll_kalman_list.append(roll_kalman)
            roll_accel_list.append(roll_acc)
            roll_rate_list.append(roll_rate)
            gps_lat_list.append(lat)
            gps_lon_list.append(lon)
            time_list.append(time.time() - t0)
            gps_info = f"Lat: {lat:.5f}, Lon: {lon:.5f}" if lat and lon else "GPS: No fix"
            print(f"Pitch_K: {pitch_kalman:.2f}°, Pitch_A: {pitch_acc:.2f}°, q: {pitch_rate:.2f}°/s | Roll_K: {roll_kalman:.2f}°, Roll_A: {roll_acc:.2f}°, p: {roll_rate:.2f}°/s | {gps_info}")
            time.sleep(0.1)
        # Guardar datos GPS
        with open("trayectoria_gps.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Tiempo (s)", "Latitud", "Longitud", "Pitch_K", "Roll_K"])
            for i in range(len(time_list)):
                writer.writerow([
                    time_list[i],
                    gps_lat_list[i] if gps_lat_list[i] else "",
                    gps_lon_list[i] if gps_lon_list[i] else "",
                    pitch_kalman_list[i],
                    roll_kalman_list[i]
                ])
        # Guardar datos de pitch
        with open("datos_pitch.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Tiempo (s)", "Pitch Accel (°)", "Pitch Kalman (°)", "q (°/s)"])
            for i in range(len(time_list)):
                writer.writerow([
                    time_list[i],
                    pitch_accel_list[i],
                    pitch_kalman_list[i],
                    pitch_rate_list[i]
                ])
        # Guardar datos de roll
        with open("datos_roll.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Tiempo (s)", "Roll Accel (°)", "Roll Kalman (°)", "p (°/s)"])
            for i in range(len(time_list)):
                writer.writerow([
                    time_list[i],
                    roll_accel_list[i],
                    roll_kalman_list[i],
                    roll_rate_list[i]
                ])
        print("Todos los datos fueron guardados correctamente.")
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario")
if _name_ == "_main_":
    main()