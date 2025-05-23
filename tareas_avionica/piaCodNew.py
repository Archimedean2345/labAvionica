#!/usr/bin/env python3
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
from kalmanFilter import kalmanFilter
from helperMethods import helperMethods
import smbus

# -------------------- CONFIGURACIÓN --------------------
ACTUAL_GRAVITY = 9.80665
DURATION = 30  # segundos

# MPU6050
bus = smbus.SMBus(1)
MPU_ADDR = 0x68
bus.write_byte_data(MPU_ADDR, 0x6B, 0)

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) + low
    return val - 65536 if val > 32768 else val

def get_acc():
    ax = read_word(0x3B) / 16384.0
    ay = read_word(0x3D) / 16384.0
    az = read_word(0x3F) / 16384.0
    return ax, ay, az  # en g

# GPS
gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

def parse_nmea(line):
    try:
        parts = line.split(",")
        if parts[0].endswith("RMC") and parts[2] == "A":
            lat = float(parts[3])
            lon = float(parts[5])
            lat = int(lat / 100) + (lat % 100) / 60
            lon = int(lon / 100) + (lon % 100) / 60
            if parts[4] == "S": lat *= -1
            if parts[6] == "W": lon *= -1
            return lat, lon
    except:
        pass
    return None, None

# -------------------- INICIALIZACIÓN --------------------
helperObj = helperMethods()

print("Esperando primer GPS fix...")
lat0 = lon0 = None
while lat0 is None:
    if gps.in_waiting:
        line = gps.readline().decode(errors='ignore').strip()
        lat0, lon0 = parse_nmea(line)
    time.sleep(0.1)

timestamp0 = time.time()
objEast = kalmanFilter(helperObj.lonToMtrs(lon0), 0.0, 2.0, ACTUAL_GRAVITY*0.03, timestamp0)
objNorth = kalmanFilter(helperObj.latToMtrs(lat0), 0.0, 2.0, ACTUAL_GRAVITY*0.05, timestamp0)
objUp = kalmanFilter(0.0, 0.0, 3.5, ACTUAL_GRAVITY*0.2, timestamp0)

# almacenamiento
pointsToPlotLat, pointsToPlotLon = [], []
orgLat, orgLon = [], []

# -------------------- LOOP PRINCIPAL --------------------
print("Iniciando fusión Kalman por {} segundos...".format(DURATION))
while time.time() - timestamp0 < DURATION:
    now = time.time()

    # Leer aceleración
    ax, ay, az = get_acc()
    acc_east = ay * ACTUAL_GRAVITY
    acc_north = ax * ACTUAL_GRAVITY
    acc_up = -az * ACTUAL_GRAVITY

    # Predicción
    objEast.predict(acc_east, now)
    objNorth.predict(acc_north, now)
    objUp.predict(acc_up, now)

    # Leer GPS
    lat, lon = None, None
    if gps.in_waiting:
        try:
            line = gps.readline().decode(errors='ignore').strip()
            lat, lon = parse_nmea(line)
        except:
            pass

    # Si hay datos GPS válidos
    if lat and lon:
        longitude = helperObj.lonToMtrs(lon)
        latitude = helperObj.latToMtrs(lat)

        objEast.update(longitude, 0.0, 0.0, 0.5)
        objNorth.update(latitude, 0.0, 0.0, 0.5)

        orgLat.append(lat)
        orgLon.append(lon)

    # Obtener predicción
    predLon = helperObj.mtrsToLon(objEast.getPredictedPos())
    predLat = helperObj.mtrsToLat(objNorth.getPredictedPos())

    pointsToPlotLat.append(predLat)
    pointsToPlotLon.append(predLon)

    time.sleep(0.1)

# -------------------- GRAFICADO --------------------
plt.subplot(2,1,1)
plt.title('Posición GPS original')
plt.plot(orgLat, orgLon, 'r.')

plt.subplot(2,1,2)
plt.title('Estimación fusionada (Kalman)')
plt.plot(pointsToPlotLat, pointsToPlotLon, 'b-')

plt.tight_layout()
plt.show()
