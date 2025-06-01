import serial
import pynmea2
import time

def read_gps_data(gps_serial, last_lat=None, last_lon=None):
    try:
        line = gps_serial.readline().decode("utf-8", errors="ignore")
        if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
            msg = pynmea2.parse(line)
            lat = msg.latitude
            lon = msg.longitude
            if lat != 0.0 and lon != 0.0:
                return lat, lon
    except Exception:
        pass
    return last_lat, last_lon

def main():
    try:
        gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
        last_lat = None
        last_lon = None
        print("Leyendo datos del GPS... (Ctrl+C para salir)")

        while True:
            lat, lon = read_gps_data(gps, last_lat, last_lon)
            if lat and lon:
                last_lat = lat
                last_lon = lon
            if last_lat and last_lon:
                print(f"Latitud: {last_lat:.5f}°, Longitud: {last_lon:.5f}°")
            else:
                print("Esperando datos válidos del GPS...")
            time.sleep(0.5)

    except serial.SerialException as e:
        print("Error al acceder al puerto serial:", e)
    except KeyboardInterrupt:
        print("\nLectura detenida por el usuario.")

if __name__ == "__main__":
    main()
