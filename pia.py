import serial
import pynmea2
import smbus
import time
from filterpy.kalman import KalmanFilter
import math

# MPU6050 config
MPU_ADDR = 0x68
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up

# Kalman Filter setup (1D position from GPS + acceleration from IMU)
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.x = [0., 0.]  # initial state (position and velocity)
kf.F = [[1., 1.], [0., 1.]]  # state transition
kf.H = [[1., 0.]]  # measurement function
kf.P *= 1000.  # covariance
kf.R = 5  # measurement noise
kf.Q = [[0.1, 0.1], [0.1, 0.1]]  # process noise

def read_mpu6050():
    accel_x = read_word_2c(0x3B) / 16384.0  # convert to g
    return accel_x * 9.81  # convert to m/s^2

def read_word_2c(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def read_gps_line(serial_port='/dev/serial0'):
    with serial.Serial(serial_port, baudrate=9600, timeout=1) as ser:
        while True:
            line = ser.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    return float(msg.latitude), float(msg.longitude), float(msg.altitude)
                except:
                    pass

# Main loop
while True:
    try:
        # Get GPS
        lat, lon, alt = read_gps_line()
        print(f"GPS: Lat {lat}, Lon {lon}, Alt {alt} m")

        # Get IMU
        acc = read_mpu6050()
        print(f"Accel X: {acc:.2f} m/s^2")

        # Kalman filter update
        kf.predict()
        kf.update(alt)
        print(f"Kalman Z: {kf.x[0]:.2f} m\n")

        time.sleep(1)

    except KeyboardInterrupt:
        break
