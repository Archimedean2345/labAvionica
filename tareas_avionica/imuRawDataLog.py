# mpu_grabacion.py
import smbus
import time
import csv

# Dirección I2C y registros
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)  # Despierta MPU6050

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg+1)
    val = (high << 8) + low
    return val - 65536 if val > 32768 else val

# Graba por 10 segundos
with open("mpu_datos.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])
    
    start = time.time()
    while time.time() - start < 10:
        t = time.time() - start
        acc_x = read_word(ACCEL_XOUT_H) / 16384.0
        acc_y = read_word(ACCEL_XOUT_H+2) / 16384.0
        acc_z = read_word(ACCEL_XOUT_H+4) / 16384.0
        gyro_x = read_word(GYRO_XOUT_H) / 131.0
        gyro_y = read_word(GYRO_XOUT_H+2) / 131.0
        gyro_z = read_word(GYRO_XOUT_H+4) / 131.0
        writer.writerow([round(t, 2), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])
        time.sleep(0.1)  # 10 Hz
