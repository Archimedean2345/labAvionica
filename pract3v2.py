import smbus
import math
import time
import numpy as np
import matplotlib.pyplot as plt

class MPU:
    def __init__(self, gyro, acc, tau):
        self.gx = None; self.gy = None; self.gz = None;
        self.ax = None; self.ay = None; self.az = None;

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.dtTimer = 0
        self.tau = tau

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor, self.accHex = self.accelerometerSensitivity(acc)

        self.bus = smbus.SMBus(1)
        self.address = 0x68

        # Listas para guardar los ángulos de cada eje
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.accel_roll_data = []
        self.accel_pitch_data = []
        self.gyro_roll_data = []
        self.gyro_pitch_data = []
        self.gyro_yaw_data = []

    def gyroSensitivity(self, x):
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x):
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0, 0x18]
        }.get(x,[8192.0,  0x08])

    def setUp(self):
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        self.bus.write_byte_data(self.address, 0x1C, self.accHex)
        self.bus.write_byte_data(self.address, 0x1B, self.gyroHex)

        print("MPU set up:")
        print('\tAccelerometer: ' + str(self.accHex) + ' ' + str(self.accScaleFactor))
        print('\tGyro: ' + str(self.gyroHex) + ' ' + str(self.gyroScaleFactor) + "\n")
        time.sleep(2)

    def eightBit2sixteenBit(self, reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        val = (h << 8) + l
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def getRawData(self):
        self.gx = self.eightBit2sixteenBit(0x43)
        self.gy = self.eightBit2sixteenBit(0x45)
        self.gz = self.eightBit2sixteenBit(0x47)

        self.ax = self.eightBit2sixteenBit(0x3B)
        self.ay = self.eightBit2sixteenBit(0x3D)
        self.az = self.eightBit2sixteenBit(0x3F)

    def calibrateGyro(self, N):
        print("Calibrating gyro with " + str(N) + " points. Do not move!")

        for ii in range(N):
            self.getRawData()
            self.gyroXcal += self.gx
            self.gyroYcal += self.gy
            self.gyroZcal += self.gz

        self.gyroXcal /= N
        self.gyroYcal /= N
        self.gyroZcal /= N

        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXcal,1)))
        print("\tY axis offset: " + str(round(self.gyroYcal,1)))
        print("\tZ axis offset: " + str(round(self.gyroZcal,1)) + "\n")
        time.sleep(2)
        self.dtTimer = time.time()

    def processIMUvalues(self):
        self.getRawData()

        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

    def compFilter(self):
        self.processIMUvalues()

        dt = time.time() - self.dtTimer
        self.dtTimer = time.time()

        accPitch = math.degrees(math.atan2(self.ay, self.az))
        accRoll = math.degrees(math.atan2(self.ax, self.az))

        self.gyroRoll -= self.gy * dt
        self.gyroPitch += self.gx * dt
        self.gyroYaw += self.gz * dt
        self.yaw = self.gyroYaw

        self.roll = (self.tau)*(self.roll - self.gy*dt) + (1-self.tau)*(accRoll)
        self.pitch = (self.tau)*(self.pitch + self.gx*dt) + (1-self.tau)*(accPitch)

        # Store data for plotting
        self.roll_data.append(self.roll)
        self.pitch_data.append(self.pitch)
        self.yaw_data.append(self.yaw)

        self.accel_roll_data.append(accRoll)
        self.accel_pitch_data.append(accPitch)

        self.gyro_roll_data.append(self.gyroRoll)
        self.gyro_pitch_data.append(self.gyroPitch)
        self.gyro_yaw_data.append(self.gyroYaw)

        print(f" R: {round(self.roll,1)} P: {round(self.pitch,1)} Y: {round(self.yaw,1)}")

def main():
    gyro = 250
    acc = 2
    tau = 0.98
    mpu = MPU(gyro, acc, tau)

    mpu.setUp()
    mpu.calibrateGyro(500)

    startTime = time.time()
    while(time.time() < (startTime + 20)):
        mpu.compFilter()

    print("Closing")

    # Plot the results after the loop
    plt.figure(figsize=(10, 6))

    # Plot the Roll, Pitch, Yaw from the complementary filter
    plt.subplot(3, 1, 1)
    plt.plot(mpu.roll_data, label="Roll (Filtered)", color='r')
    plt.plot(mpu.accel_roll_data, label="Roll (Accelerometer)", color='b', linestyle='--')
    plt.title("Roll Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.legend()

    # Plot the Pitch
    plt.subplot(3, 1, 2)
    plt.plot(mpu.pitch_data, label="Pitch (Filtered)", color='r')
    plt.plot(mpu.accel_pitch_data, label="Pitch (Accelerometer)", color='b', linestyle='--')
    plt.title("Pitch Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.legend()

    # Plot the Yaw
    plt.subplot(3, 1, 3)
    plt.plot(mpu.yaw_data, label="Yaw (Filtered)", color='r')
    plt.plot(mpu.gyro_yaw_data, label="Yaw (Gyroscope)", color='b', linestyle='--')
    plt.title("Yaw Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
