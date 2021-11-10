#include <Wire.h>
#include <SFE_BMP180.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include"timer-api.h"
#include "Kalman.h"

int _timer = TIMER_DEFAULT;
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
int gyroZnach=0;//6 7
int baroZnach=0;//8 9

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
//  EEPROM.write(6,highByte(100));
//  EEPROM.write(7,lowByte(100));
//  EEPROM.write(8,highByte(20));
//  EEPROM.write(9,lowByte(20));
  timer_init_ISR_1Hz(_timer);

  pinMode(22,OUTPUT);
  pinMode(52,OUTPUT);
  pinMode(53,INPUT);
  digitalWrite(52,HIGH);
  digitalWrite(22,HIGH);
  updateData();
  Serial.begin(9600);
  acsel = EEPROM.read(1);
  barometr = EEPROM.read(2);
  zummer = EEPROM.read(3);
  vibro = EEPROM.read(4);
  provod = EEPROM.read(5);
  byte hi= EEPROM.read(6);
  byte lo= EEPROM.read(7);
  gyroZnach=word(hi, lo);
  hi= EEPROM.read(8);
  lo= EEPROM.read(9);
  baroZnach=word(hi, lo);
  

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

void timer_handle_interrupts(int timer) {
    ris();
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
    if(rezh==1){
      String znach;
      if ( acsel==1){znach="on";}else{ znach="off";}
      lcd.begin(16, 2);
      lcd.print("gyroscope: "+znach);
      delay(300);
      lcd.setCursor(0, 0);
      lcd.print("gyroscope: "+znach);
      lcd.setCursor(0, 1);
      lcd.print("X "+String(kalAngleX,1)+"  Y "+String(kalAngleY,1));
    }else{
      lcd.begin(16, 2);
      lcd.print(gyroZnach);
      delay(300);
      lcd.setCursor(0, 0);
      lcd.print(gyroZnach);
    }
  }
  if (pos1==3){
    if(rezh==1){
      String znach;
      if (barometr==1){znach="on";}else{ znach="off";}
      lcd.begin(16, 2);
      lcd.print("barometer: "+znach);
      delay(300);
      lcd.setCursor(0, 0);
      lcd.print("barometer: "+znach);
      lcd.setCursor(0, 1);
      lcd.print(String(P,4));
    }else{
      lcd.begin(16, 2);
      lcd.print(baroZnach);
      delay(300);
      lcd.setCursor(0, 0);
      lcd.print(baroZnach);
    }
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
bool flag=true;
unsigned long time;
int gyroStartX;
int gyroStartY;

bool gyroZvon=false;
bool breakZvon=false;
bool zommerActive=false;

void loop() {  
  updateData();
  if((flag)&& (millis()>5000)){
    flag=false;
    gyroStartX=kalAngleX;
    gyroStartY=kalAngleY;
    Serial.println(gyroStartX);
    Serial.println(gyroStartY);
  }
  /* Print Data */   
 
  /*Serial.print(kalAngleX);Serial.print("\t");
  Serial.print(kalAngleY);Serial.print("\t");
  Serial.print(temp);Serial.print("\t");   
  Serial.print("\n");
  delay(1); // The accelerometer's maximum samples rate is 1kHz*/
  int button = detectButton();
 
  switch (button) {
    case BTN_UP: 
      if(rezh==1){
        pos1-=1;
        if (pos1>6){pos1=1;}else if(pos1<1){pos1=6;}
      }else{
        if(pos1==2){
          gyroZnach+=5;
          if(gyroZnach>360){
            gyroZnach=5;
          }else if(gyroZnach<0){
            gyroZnach=355;
          }
        }else if(pos1==3){
          baroZnach+=2;
          if(baroZnach>1000){
            baroZnach=2;
          }else if(baroZnach<0){
            baroZnach=1999;
          }
        }
        EEPROM.write(6,highByte(gyroZnach));
        EEPROM.write(7,lowByte(gyroZnach));
        EEPROM.write(8,highByte(baroZnach));
        EEPROM.write(9,lowByte(baroZnach));
      }
      ris();
      break;
    case BTN_DOWN:
      if(rezh==1){
        pos1+=1;
        if (pos1>6){pos1=1;}else if(pos1<1){pos1=6;}
      }else{
        if(pos1==2){
          gyroZnach-=5;
          if(gyroZnach>360){
            gyroZnach=5;
          }else if(gyroZnach<0){
            gyroZnach=355;
          }
        }else if(pos1==3){
          baroZnach-=2;
          if(baroZnach>1000){
            baroZnach=2;
          }else if(baroZnach<0){
            baroZnach=1999;
          }
        }
        EEPROM.write(6,highByte(gyroZnach));
        EEPROM.write(7,lowByte(gyroZnach));
        EEPROM.write(8,highByte(baroZnach));
        EEPROM.write(9,lowByte(baroZnach));
      }
      ris();
      break;
    case BTN_LEFT:
      if(rezh==1){
        if (pos1==2){
          if (acsel==1){
            EEPROM.write(1,0);
            gyroZvon=false;
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
            breakZvon=false;
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
      }
      break;
    case BTN_RIGHT:
      if(rezh==1){
        pos1-=1;
        if (pos1>6){pos1=1;}else if(pos1<1){pos1=6;}
      }else{
        if(pos1==2){
          gyroZnach+=5;
          if(gyroZnach>360){
            gyroZnach=5;
          }else if(gyroZnach<0){
            gyroZnach=355;
          }
        }else if(pos1==3){
          baroZnach+=2;
          if(baroZnach>1000){
            baroZnach=2;
          }else if(baroZnach<0){
            baroZnach=1999;
          }
        }
        EEPROM.write(6,highByte(gyroZnach));
        EEPROM.write(7,lowByte(gyroZnach));
        EEPROM.write(8,highByte(baroZnach));
        EEPROM.write(9,lowByte(baroZnach));
      }
      ris();
      break;
    case BTN_SELECT:
      if((pos1==2)||(pos1==3)){
        if(rezh==1){
          rezh=2;
        }else{
          rezh=1;
        }
        ris();
      }   
      break;
    default:
      //printDisplay("Press any key");
      break;
  }

  /*Проверка датчиков*/
  if (zummer==1){
        /*акселирометр*/
        if((flag==false)&&(acsel==1)){
          //gyroStartX kalAngleX gyroZnach
          bool flagZum=false;
          if(kalAngleX>gyroStartX){
            if((abs(kalAngleX-gyroStartX)>gyroZnach/2)&&(abs(gyroStartX+(360-kalAngleX))>gyroZnach/2)){
              Serial.println("1");
              flagZum=true;
            }
          }else{
            if((abs(gyroStartX-kalAngleX)>gyroZnach/2)&&(abs(kalAngleX+(360-gyroStartX))>gyroZnach/2)){
              Serial.println("2");
              flagZum=true;
            }
          }
          
          if(kalAngleY>gyroStartY){
            if((abs(kalAngleY-gyroStartY)>gyroZnach/2)&&(abs(gyroStartY+(360-kalAngleY))>gyroZnach/2)){
              Serial.println("3");
              flagZum=true;
            }
          }else{
            if((abs(gyroStartY-kalAngleY)>gyroZnach/2)&&(abs(kalAngleY+(360-gyroStartY))>gyroZnach/2)){
              Serial.println("4");
              flagZum=true;
            }
          }
          gyroZvon=flagZum;
//          if(flagZum){
//            digitalWrite(22,LOW);
//          }else{
//            digitalWrite(22,HIGH);
//          }
        }
        /*провод*/
        if(provod==1){
          if (digitalRead(53)==LOW){
            breakZvon=true;
          }else{
            breakZvon=false;
          }
        }
  }
  //zommerActive
  if((gyroZvon)||(breakZvon)){
    if (!zommerActive){
      zommerActive=true;
      digitalWrite(22,LOW);
    }
  }else{
    if (zommerActive){
      zommerActive=false;
      digitalWrite(22,HIGH);
    }
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
