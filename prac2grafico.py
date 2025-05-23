import smbus2
import time
import math
import numpy as np
import matplotlib.pyplot as plt

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
    return value - 65536 if value >= 0x8000 else value

def get_accel_data():
    return tuple(read_word_2c(ACCEL_XOUT_H + i) / 16384.0 for i in [0, 2, 4])

def get_gyro_data():
    return tuple(read_word_2c(GYRO_XOUT_H + i) / 131.0 for i in [0, 2, 4])

def get_euler_angles():
    ax, ay, az = get_accel_data()
    pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
    roll = math.atan2(-ax, az) * 180 / math.pi
    return roll, pitch

def calculate_euler_rates(roll, pitch, gyro_data):
    gx, gy, gz = gyro_data
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    matrix = np.array([
        [1, math.sin(roll)*math.tan(pitch), math.cos(roll)*math.tan(pitch)],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll)/math.cos(pitch), math.cos(roll)/math.cos(pitch)]
    ])
    return np.dot(matrix, np.array([gx, gy, gz]))

if __name__ == "__main__":
    mpu6050_init()
    yaw = 0.0
    last_time = time.time()

    times = []
    rolls = []
    pitchs = []
    yaws = []
    ps, qs, rs = [], [], []
    euler_p, euler_q, euler_r = [], [], []

    start_time = time.time()
    while time.time() - start_time <= 20:
        try:
            roll, pitch = get_euler_angles()
            gx, gy, gz = get_gyro_data()

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            yaw += gz * dt

            euler_rates = calculate_euler_rates(roll, pitch, (gx, gy, gz))

            # Guardar datos
            times.append(current_time - start_time)
            rolls.append(roll)
            pitchs.append(pitch)
            yaws.append(yaw)
            ps.append(gx)
            qs.append(gy)
            rs.append(gz)
            euler_p.append(euler_rates[0])
            euler_q.append(euler_rates[1])
            euler_r.append(euler_rates[2])

            time.sleep(0.1)

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)

    # Graficar resultados
    plt.figure()
    plt.plot(times, rolls, label="Roll (°)")
    plt.plot(times, pitchs, label="Pitch (°)")
    plt.plot(times, yaws, label="Yaw (°)")
    plt.title("Euler Angles")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(times, ps, label="p (°/s)")
    plt.plot(times, qs, label="q (°/s)")
    plt.plot(times, rs, label="r (°/s)")
    plt.title("Gyro Rates (p, q, r)")
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (°/s)")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(times, euler_p, label="Euler rate p")
    plt.plot(times, euler_q, label="Euler rate q")
    plt.plot(times, euler_r, label="Euler rate r")
    plt.title("Euler Rates")
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (°/s)")
    plt.legend()
    plt.grid(True)
    plt.show()
