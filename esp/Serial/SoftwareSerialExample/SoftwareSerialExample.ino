#include <SoftwareSerial.h>
SoftwareSerial mySerial(13, 15); // RX, TX

int mass[13];
void setup()  
{
 // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
}
void reciveData(){
  String str;
  while (mySerial.available() > 0){
        char recieved = mySerial.read();
        str += recieved;
        float x; float y;float theta;
        if (recieved == '\n'){
            Serial.println(str);
            String s="";
            int k=0;
            for (int i=0;i<str.length();i++){
              if(str[i]!=':'){
                s+=str[i];
              }else{
                Serial.println(s);
                mass[k]=s.toInt();
                k++;
                s="";
              }
            }
            str = "";
        }
        
   }
}

void loop() // run over and over
{
    reciveData();
    for(int i;i<13;i++){
        Serial.println(String(mass[i],0));
    }
    Serial.println();
}
