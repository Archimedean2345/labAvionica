import serial
import pynmea2

def read_gps():
    try:
        gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
        print("Leyendo datos del GPS... (Ctrl+C para salir)")

        while True:
            line = gps.readline().decode("utf-8", errors="ignore")
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)
                    lat = msg.latitude
                    lon = msg.longitude
                    print(f"Latitud: {lat:.5f}°, Longitud: {lon:.5f}°")
                except pynmea2.ParseError:
                    continue

    except KeyboardInterrupt:
        print("\nLectura detenida por el usuario.")
    except Exception as e:
        print(f"Error al abrir el puerto: {e}")

if __name__ == "__main__":
    read_gps()
