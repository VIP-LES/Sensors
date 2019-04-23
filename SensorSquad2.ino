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

int interval = 300; 
 
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
    float temperature = 0;
    float pressure = 0;
    int humidity = 0;
    int co2 = 0;
    int voc = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
        
    for (int i = 0; i < interval; i++) {
      temperature = temperature + bme.readTemperature();
      pressure = pressure + bme.readPressure();
      humidity = humidity + analogRead(A1); // in range from 0 to 1023
      
      if (mySensor.dataAvailable())
      {
        mySensor.readAlgorithmResults();
        co2 = mySensor.getCO2();
        voc = mySensor.getTVOC();
      }
  
      imu.readAccel(); 
      imu.readMag(); 
      imu.readGyro();
      
      mx = mx + imu.mx;
      my = my + imu.my;
      mz = mz + imu.mz;
      gx = gx + imu.gx;
      gy = gy + imu.gy;
      gz = gz + imu.gz;
      ax = ax + imu.ax;
      ay = ay + imu.ay;
      az = az + imu.az;
    }

    temperature = temperature / interval;
    pressure = pressure / interval;
    humidity = humidity / interval;
    mx = mx / interval;
    my = my / interval;
    mz = mz / interval;
    gx = gx / interval;
    gy = gy / interval;
    gz = gz / interval;
    ax = ax / interval;
    ay = ay / interval;
    az = az / interval;
    
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(ax); // Print x-axis data
    Serial.print(", ");
    Serial.print(ay); // print y-axis data
    Serial.print(", ");
    Serial.print(az); // print z-axis data
    Serial.print(", ");    
    Serial.print(mx); // Print x-axis data
    Serial.print(", ");
    Serial.print(my); // print y-axis data
    Serial.print(", ");
    Serial.print(mz); // print z-axis data
    Serial.print(", ");
    Serial.print(gx); // Print x-axis data
    Serial.print(", ");
    Serial.print(gy); // print y-axis data
    Serial.print(", ");
    Serial.print(gz); // print z-axis data
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
}

void countPulse(){
  detachInterrupt(digitalPinToInterrupt(geiger_input));
  count++;
  while(digitalRead(geiger_input)==0){
  }
  attachInterrupt(0,countPulse,FALLING);
}
