#include <Wire.h>
#include <SFE_BMP180.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "Kalman.h"

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
SFE_BMP180 pressure;

#define BTN_UP   1
#define BTN_DOWN 2
#define BTN_LEFT 3
#define BTN_RIGHT 4
#define BTN_SELECT 5
#define BTN_NONE 10

Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68;

int pos1;
int rezh=1;//режим 1-начальный 2-подкатолог
/*
1 Температура (мб обогрев)
2 Акселерометр(да/нет)(значение)
3 Барометр(да/нет)(значение)
4 Зуммер(вкл/выкл)
5 Датчик вибрации(вкл/выкл)
6 Провод(да/нет)
*/
byte acsel=1;//байт 1
byte barometr=1;//байт 2
byte zummer=1;//байт 3
byte vibro=1;//байт 4
byte provod=1;//байт 5

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
  updateData();
  Serial.begin(9600);
  acsel = EEPROM.read(1);
  barometr = EEPROM.read(2);
  zummer = EEPROM.read(3);
  vibro = EEPROM.read(4);
  provod = EEPROM.read(5);
  
  
  

  lcd.begin(16, 2);
  lcd.print("temperature: ");
  delay(300);
  lcd.setCursor(0, 0);
  lcd.print("temperature: ");
  lcd.setCursor(0, 1);
  lcd.print(temp);
  pos1=1;
  
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

void updateData(){
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
}

void ris(){
  if(pos1==1){
    lcd.begin(16, 2);
    lcd.print("temperature: ");
    delay(300);
    lcd.setCursor(0, 0);
    lcd.print("temperature: ");
    lcd.setCursor(0, 1);
    lcd.print(temp);
  }
  if (pos1==2){
    String znach;
    if ( acsel==1){znach="on";}else{ znach="off";}
    lcd.begin(16, 2);
    lcd.print("gyroscope: "+znach);
    delay(300);
    lcd.setCursor(0, 0);
    lcd.print("gyroscope: "+znach);
    lcd.setCursor(0, 1);
    lcd.print("X "+String(kalAngleX,1)+"  Y "+String(kalAngleY,1));
  }
  if (pos1==3){
    String znach;
    if (barometr==1){znach="on";}else{ znach="off";}
    lcd.begin(16, 2);
    lcd.print("barometer: "+znach);
    delay(300);
    lcd.setCursor(0, 0);
    lcd.print("barometer: "+znach);
    lcd.setCursor(0, 1);
    lcd.print(String(P,4));
  }
  if (pos1==4){
    String znach;
    if (zummer==1){znach="on";}else{ znach="off";}
    lcd.begin(16, 2);
    lcd.print("buzzer: "+znach);
    delay(300);
    lcd.setCursor(0, 0);
    lcd.print("buzzer: "+znach);
    lcd.setCursor(0, 1);
    lcd.print("");
  }
  if (pos1==5){
    String znach;
    if (vibro==1){znach="on";}else{ znach="off";}
    lcd.begin(16, 2);
    lcd.print("shock sensor:"+znach);
    delay(300);
    lcd.setCursor(0, 0);
    lcd.print("shock sensor:"+znach);
    lcd.setCursor(0, 1);
    lcd.print("");
  }
  if (pos1==6){
    String znach;
    if (provod==1){znach="on";}else{ znach="off";}
    lcd.begin(16, 2);
    lcd.print("wire break: "+znach);
    delay(300);
    lcd.setCursor(0, 0);
    lcd.print("wire break: "+znach);
    lcd.setCursor(0, 1);
    lcd.print("");
  }
}

void loop() {
  
  
  updateData();
  /* Print Data */   
 
  /*Serial.print(kalAngleX);Serial.print("\t");
  Serial.print(kalAngleY);Serial.print("\t");
  Serial.print(temp);Serial.print("\t");   
  Serial.print("\n");
  delay(1); // The accelerometer's maximum samples rate is 1kHz*/
  
  int button = detectButton();
 
  switch (button) {
    case BTN_UP:
      pos1-=1;
      if (pos1>6){pos1=1;}else if(pos1<1){pos1=6;}
      ris();
      break;
    case BTN_DOWN:
      pos1+=1;
      if (pos1>6){pos1=1;}else if(pos1<1){pos1=6;}
      ris();
      break;
    case BTN_LEFT:
      if (pos1==2){
        if (acsel==1){
          EEPROM.write(1,0);
        }else{
          EEPROM.write(1,1);
        }
      }else if(pos1==3){
        if(barometr==1){
          EEPROM.write(2,0);
        }else{
          EEPROM.write(2,1);
        }
      }else if(pos1==4){
        if( zummer==1){
          EEPROM.write(3,0);
        }else{
          EEPROM.write(3,1);
        }
      }else if(pos1==5){
        if(vibro==1){
          EEPROM.write(4,0);
        }else{
          EEPROM.write(4,1);
        }
      }else if(pos1==6){
        if(provod==1){
          EEPROM.write(5,0);
        }else{
          EEPROM.write(5,1);
        }
      }
      acsel = EEPROM.read(1);
      barometr = EEPROM.read(2);
      zummer = EEPROM.read(3);
      vibro = EEPROM.read(4);
      provod = EEPROM.read(5);
      ris();
      break;
    case BTN_RIGHT:
      pos1-=1;
      if (pos1>6){pos1=1;}else if(pos1<1){pos1=6;}
      ris();
      break;
    case BTN_SELECT:
      printDisplay("SELECT");
      break;
    default:
      //printDisplay("Press any key");
      break;
  }
  
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

int detectButton() {
  int keyAnalog =  analogRead(A0);
  if (keyAnalog < 100) {
    // Значение меньше 100 – нажата кнопка right
    return BTN_RIGHT;
  } else if (keyAnalog < 200) {
    // Значение больше 100 (иначе мы бы вошли в предыдущий блок результата сравнения, но меньше 200 – нажата кнопка UP
    return BTN_UP;
  } else if (keyAnalog < 400) {
    // Значение больше 200, но меньше 400 – нажата кнопка DOWN
    return BTN_DOWN;
  } else if (keyAnalog < 600) {
    // Значение больше 400, но меньше 600 – нажата кнопка LEFT
    return BTN_LEFT;
  } else if (keyAnalog < 800) {
    // Значение больше 600, но меньше 800 – нажата кнопка SELECT
    return BTN_SELECT;
  } else {
    // Все остальные значения (до 1023) будут означать, что нажатий не было
    return BTN_NONE;
  }
}
void clearLine(int line){
  lcd.setCursor(0, 1);
  lcd.print("                ");
}
 
void printDisplay(String message){
  Serial.println(message);
  lcd.setCursor(0, 1);
  lcd.print(message);
  delay(1000);
  clearLine(1);
}
