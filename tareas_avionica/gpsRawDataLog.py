# gps_grabacion.py
import serial
import time

# Ajusta el puerto a tu módulo GPS
puerto = "/dev/ttyUSB0"
gps = serial.Serial(puerto, baudrate=9600, timeout=1)

with open("gps_datos.txt", "w") as archivo:
    start = time.time()
    while time.time() - start < 10:
        try:
            linea = gps.readline().decode('utf-8', errors='ignore').strip()
            if linea.startswith("$GPGGA") or linea.startswith("$GPRMC"):
                archivo.write(linea + "\n")
        except Exception as e:
            print("Error:", e)
