#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SparkFunLSM9DS1.h>
#include <SparkFunCCS811.h>

#define CCS811_ADDR 0x5B //Default I2C Address

Adafruit_BMP280 bme;
LSM9DS1 imu;
CCS811 mySensor(CCS811_ADDR);
int geiger_input = 2;
int count = 0;
 
void setup() {
  Serial.begin(9600);
  Serial.println("SENSOR SQUAD: time ax ay az mx my mz gx gy gz temp pressure humidity co2 voc geigercount");
  
  if (!bme.begin()) {  
    Serial.println("Failed to communicate with BMP280");
    //while (1);
  }

  imu.settings.device.commInterface = IMU_MODE_I2C;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1");
    //while (1);
  }

  CCS811Core::status returnCode = mySensor.begin();
  switch ( returnCode )
  {
  case CCS811Core::SENSOR_ID_ERROR:
    Serial.println("CCS811 beginCore exited with: ID_ERROR");
    break;
  case CCS811Core::SENSOR_I2C_ERROR:
    Serial.println("CCS811 beginCore exited with: I2C_ERROR");
    break;
  case CCS811Core::SENSOR_INTERNAL_ERROR:
    Serial.println("CCS811 beginCore exited with: INTERNAL_ERROR");
    break;
  case CCS811Core::SENSOR_GENERIC_ERROR:
    Serial.println("CCS811 beginCore exited with: GENERIC_ERROR");
    break;
  }

  pinMode(geiger_input, INPUT);
  digitalWrite(geiger_input,HIGH);
  attachInterrupt(digitalPinToInterrupt(geiger_input), countPulse, FALLING);
}
  
void loop() {
    float temperature = bme.readTemperature();
    int pressure = bme.readPressure();
    int humidity = analogRead(A1); // in range from 0 to 1023
    
    int co2 = 0;
    int voc = 0;
    if (mySensor.dataAvailable())
    {
      mySensor.readAlgorithmResults();
      co2 = mySensor.getCO2();
      voc = mySensor.getTVOC();
    }

    imu.readAccel(); 
    imu.readMag(); 
    imu.readGyro();
    
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(imu.ax); // Print x-axis data
    Serial.print(", ");
    Serial.print(imu.ay); // print y-axis data
    Serial.print(", ");
    Serial.print(imu.az); // print z-axis data
    Serial.print(", ");    
    Serial.print(imu.mx); // Print x-axis data
    Serial.print(", ");
    Serial.print(imu.my); // print y-axis data
    Serial.print(", ");
    Serial.print(imu.mz); // print z-axis data
    Serial.print(", ");
    Serial.print(imu.gx); // Print x-axis data
    Serial.print(", ");
    Serial.print(imu.gy); // print y-axis data
    Serial.print(", ");
    Serial.print(imu.gz); // print z-axis data
    Serial.print(", ");
    Serial.print(temperature);
    Serial.print(", ");
    Serial.print(pressure);
    Serial.print(", ");
    Serial.print(humidity);
    Serial.print(", ");
    Serial.print(co2);
    Serial.print(", ");
    Serial.print(voc);
    Serial.print(", ");
    Serial.print(count);

    Serial.println();

    delay(2000);
}

void countPulse(){
  detachInterrupt(digitalPinToInterrupt(geiger_input));
  count++;
  while(digitalRead(geiger_input)==0){
  }
  attachInterrupt(0,countPulse,FALLING);
}
