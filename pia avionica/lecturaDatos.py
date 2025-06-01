import smbus
import time
import serial
import pynmea2
import csv

# --- MPU6050 Configuración ---
class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value = value - 65536
        return value

    def get_accel_data(self):
        ax = self.read_raw_data(0x3B) / 16384.0 * 9.80665
        ay = self.read_raw_data(0x3D) / 16384.0 * 9.80665
        az = self.read_raw_data(0x3F) / 16384.0 * 9.80665
        return round(ax, 3), round(ay, 3), round(az, 3)

# --- GPS Configuración ---
def read_gps(serial_port):
    try:
        line = serial_port.readline().decode("utf-8", errors="ignore")
        if line.startswith('$GPGGA'):
            msg = pynmea2.parse(line)
            if msg.lat and msg.lon:
                lat = msg.latitude
                lon = msg.longitude
                return lat, lon
    except Exception:
        pass
    return None, None

# --- PROGRAMA PRINCIPAL ---
def main():
    mpu = MPU6050()
    gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
    last_lat = None
    last_lon = None
    filename = "lecturas_avionica.csv"

    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Tiempo (s)", "Acc X (m/s²)", "Acc Y (m/s²)", "Acc Z (m/s²)", "Latitud", "Longitud"])
        print("Iniciando captura de datos (Ctrl+C para detener)...")

        start_time = time.time()
        try:
            while True:
                timestamp = round(time.time() - start_time, 2)
                ax, ay, az = mpu.get_accel_data()

                lat, lon = read_gps(gps)
                if lat is not None and lon is not None:
                    last_lat = lat
                    last_lon = lon

                writer.writerow([timestamp, ax, ay, az, last_lat, last_lon])
                print(f"[{timestamp}s] Ax={ax:.2f} Ay={ay:.2f} Az={az:.2f} | Lat={last_lat} Lon={last_lon}")
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\nFinalizando captura.")
            gps.close()

if __name__ == "__main__":
    main()
