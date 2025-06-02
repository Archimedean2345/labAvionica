#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// === Objetos y pines ===
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3); // RX, TX (ajusta según conexión)
File dataFile;

unsigned long startTime;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("No se detectó MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // SD
  if (!SD.begin(10)) {
    Serial.println("Fallo en SD");
    while (1);
  }

  // Crear archivo
  dataFile = SD.open("datos.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("tiempo,acc_z,acc_y,acc_x,latitude,longitude");
    dataFile.close();
  }

  startTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Leer GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Obtener tiempo relativo
  float tiempo = (millis() - startTime) / 1000.0;

  // Validar que hay fix GPS
  float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  float lon = gps.location.isValid() ? gps.location.lng() : 0.0;

  // Guardar datos
  dataFile = SD.open("datos.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(tiempo, 3); dataFile.print(",");
    dataFile.print(a.acceleration.z, 6); dataFile.print(",");
    dataFile.print(a.acceleration.y, 6); dataFile.print(",");
    dataFile.print(a.acceleration.x, 6); dataFile.print(",");
    dataFile.print(lat, 6); dataFile.print(",");
    dataFile.println(lon, 6);
    dataFile.close();
  }

  delay(500); // 0.5 segundos entre muestras
}
