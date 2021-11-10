#include <Wire.h>
#include <SFE_BMP180.h>
#include "Kalman.h"

SFE_BMP180 pressure;

Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68;

int pos;
/*
1 Температура (мб обогрев)
2 Акселерометр(да/нет)(значение)
3 Барометр(да/нет)(значение)
4 Зуммер(вкл/выкл)
5 Датчик вибрации(вкл/выкл)
6 Провод(да/нет)
*/

/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

double accXangle; // Angle calculate using the accelerometer
double accYangle;
double temp;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleY = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;

double P;

uint32_t timer;

void setup() {  
  pos=1;
  Serial.begin(9600);
  pressure.begin();
  Wire.begin();  
  i2cWrite(0x6B,0x00); // Disable sleep mode  
  if(i2cRead(0x75,1)[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("MPU-6050 with address 0x"));
    Serial.print(IMUAddress,HEX);
    Serial.println(F(" is not connected"));
    while(1);
  }      
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();
}

void loop() {
  
  /* Update all the values */
  uint8_t* data = i2cRead(0x3B,14);  
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);  
  tempRaw = ((data[6] << 8) | data[7]);  
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
  
  /* Calculate the angls based on the different sensors and algorithm */
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;    
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);  
  
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  timer = micros();
  
  temp = ((double)tempRaw + 12412.0) / 340.0;

  P = getPressure();
  
  /* Print Data */   

  Serial.print(kalAngleX);Serial.print("\t");
  Serial.print(kalAngleY);Serial.print("\t");
  Serial.print(temp);Serial.print("\t");   
  Serial.print("\n");
  delay(1); // The accelerometer's maximum samples rate is 1kHz
}

double getPressure(){
    char status;
    double T,P,p0,a;

    status = pressure.startTemperature();
    if (status != 0){
        // ожидание замера температуры
        delay(status);
        status = pressure.getTemperature(T);
        if (status != 0){
            status = pressure.startPressure(3);
            if (status != 0){
                // ожидание замера давления
                delay(status);
                status = pressure.getPressure(P,T);
                if (status != 0){
                    return(P);
                }
            }
        }
    }
}

void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];  
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data[i] = Wire.read();
  return data;
} 
