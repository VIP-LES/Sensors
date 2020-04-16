#include <SPI.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <SparkFunLSM9DS1.h>
#include <DHT.h>
#include <DHT_U.h>



#define DHTPIN 2 
#define DHTTYPE DHT22

SFE_BMP180 bmp180_sensor;  
LSM9DS1 imu;


int geiger_input = 2;
int count = 0;

int interval = 300; 
 
void setup() {
  Serial.begin(9600);
  Serial.println("SENSOR SQUAD: time ax ay az mx my mz gx gy gz temp pressure humidity co2 voc geigercount");

  
  
  if (bmp180_sensor.begin())
  {
    Serial.println("BMP180 initialized successfully"); 
  }
  else
  {
    Serial.println("BMP180 initialization failure"); 
    //while(1)
  }

  imu.settings.device.commInterface = IMU_MODE_I2C;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1");
    //while (1);
  }

  pinMode(geiger_input, INPUT);
  digitalWrite(geiger_input,HIGH);
  attachInterrupt(digitalPinToInterrupt(geiger_input), countPulse, FALLING);
}

//float baseline;
//baseline= getPressure();
//Serial.print("baseline pressure: ");
//Serial.print(baseline);
//Serial.println(" mb")




void loop() {
    char stat;
    double T; 
    float temper = 0;
    float pressure = 0;
    float alt = 0; 
    int humidity = 0;
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
      
      
      stat = bmp180_sensor.startTemperature();
      if (stat != 0)
      {
      delay(stat);
      bmp180_sensor.getTemperature(T);
      }
     
            
      imu.readAccel(); 
      imu.readMag(); 
      imu.readGyro();
      
      mx = mx + imu.mx;
      my = my + imu.my;
      mz = mz + imu.mz;
      gx = gz + imu.gx;
      gy = gy + imu.gy;
      gz = gz + imu.gz;
      ax = ax + imu.ax;
      ay = ay + imu.ay;
      az = az + imu.az;
    }

    
    
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
    Serial.print(temper);
    Serial.print(", ");
    Serial.print(pressure);
    Serial.print(", ");

    Serial.println();
    count = 0;
}

void countPulse(){
  detachInterrupt(digitalPinToInterrupt(geiger_input));
  count++;
  while(digitalRead(geiger_input)==0){
  }
  attachInterrupt(0,countPulse,FALLING);
}

double getP(double T){   
      double P; 
      char stat;
      stat = bmp180_sensor.startPressure(3);
      if(stat != 0)
      {
        delay(stat);
        stat = bmp180_sensor.getPressure(P,T);
        if (stat!=0) 
        {
          return(P);
             }else Serial.println("error retrieving pressure measurement\n");
          }else Serial.println("error starting pressure measurement\n");  
}
