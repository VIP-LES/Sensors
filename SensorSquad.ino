#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SparkFunLSM9DS1.h>

Adafruit_BMP280 bme;
LSM9DS1 imu;
 
void setup() {
  Serial.begin(9600);
  Serial.println(F("SENSOR SQUAD :)"));
  
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  imu.settings.device.commInterface = IMU_MODE_I2C;
  if (!imu.begin())
  {
      Serial.println("Failed to communicate with LSM9DS1.");
      while (1);
  }

  
}
  
void loop() {
    float temperature = bme.readTemperature();
    int pressure = bme.readPressure();

    //Serial.print(temperature);
    //Serial.print(" ");
    //Serial.print(pressure);
    //Serial.println();

    imu.readAccel(); // Update the accelerometer data
    Serial.print(imu.ax); // Print x-axis data
    Serial.print(", ");
    Serial.print(imu.ay); // print y-axis data
    Serial.print(", ");
    Serial.println(imu.az); // print z-axis data
    
    delay(2000);
}
