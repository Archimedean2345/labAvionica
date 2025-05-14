import smbus2
import time
import math
import numpy as np

# Dirección I2C del MPU6050
MPU6050_ADDR = 0x68

# Registros del MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Inicializar la conexión I2C
bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_word_2c(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value

def get_accel_data():
    accel_x = read_word_2c(ACCEL_XOUT_H)
    accel_y = read_word_2c(ACCEL_XOUT_H + 2)
    accel_z = read_word_2c(ACCEL_XOUT_H + 4)
    accel_x /= 16384.0
    accel_y /= 16384.0
    accel_z /= 16384.0
    return accel_x, accel_y, accel_z

def get_gyro_data():
    gyro_x = read_word_2c(GYRO_XOUT_H)
    gyro_y = read_word_2c(GYRO_XOUT_H + 2)
    gyro_z = read_word_2c(GYRO_XOUT_H + 4)
    gyro_x /= 131.0
    gyro_y /= 131.0
    gyro_z /= 131.0
    return gyro_x, gyro_y, gyro_z

def get_euler_angles():
    accel_x, accel_y, accel_z = get_accel_data()
    pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    roll = math.atan2(-accel_x, accel_z) * 180 / math.pi
    return roll, pitch

def calculate_euler_rates(roll, pitch, gyro_data):
    gyro_x, gyro_y, gyro_z = gyro_data
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    transform_matrix = np.array([
        [1, math.sin(roll) * math.tan(pitch), math.cos(roll) * math.tan(pitch)],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll) / math.cos(pitch), math.cos(roll) / math.cos(pitch)]
    ])
    gyro_vector = np.array([gyro_x, gyro_y, gyro_z])
    euler_rates = np.dot(transform_matrix, gyro_vector)
    return euler_rates

if __name__ == "__main__":
    mpu6050_init()
    yaw = 0.0
    last_time = time.time()

    while True:
        try:
            roll, pitch = get_euler_angles()
            gyro_data = get_gyro_data()
            gyro_x, gyro_y, gyro_z = gyro_data

            # Calcular tiempo transcurrido
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # Integrar el yaw
            yaw += gyro_z * dt

            # Calcular tasas de Euler
            euler_rates = calculate_euler_rates(roll, pitch, gyro_data)

            # Mostrar resultados
            print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            print(f"Gyro p: {gyro_x:.2f}°/s, q: {gyro_y:.2f}°/s, r: {gyro_z:.2f}°/s")
            print(f"Euler Rates: {euler_rates}")
            print("-" * 50)
            time.sleep(0.1)

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)
