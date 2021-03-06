
/* Virtuino NodeMCU ESP8266 web server and WiFi access point example No1  
 * Created by Ilias Lamprou
 * Updated Feb 1 2016
 * Before  running this code config the settings below as the instructions on the right
 * 
 * 
 * Download latest Virtuino android app from the link: https://play.google.com/store/apps/details?id=com.virtuino_automations.virtuino&hl=el
 * Contact address for questions or comments: iliaslampr@gmail.com
 */


/*========= Virtuino General methods  
*
*  void vDigitalMemoryWrite(int digitalMemoryIndex, int value)       write a value to a Virtuino digital memory   (digitalMemoryIndex=0..31, value range = 0 or 1)
*  int  vDigitalMemoryRead(int digitalMemoryIndex)                   read  the value of a Virtuino digital memory (digitalMemoryIndex=0..31, returned value range = 0 or 1)
*  void vMemoryWrite(int memoryIndex, float value);                  write a value to Virtuino memory             (memoryIndex=0..31, value range as float value)
*  float vMemoryRead(int memoryIndex);                               read a value of  Virtuino memory             (memoryIndex=0..31, returned a float value
*  run();                                                            neccesary command to communicate with Virtuino android app  (on start of void loop)
*  int getPinValue(int pin);                                         read the value of a Pin. Usefull to read the value of a PWM pin
*  long lastCommunicationTime;                                       Stores the last communication with Virtuino time
*/ 


// 1.  On Arduino IDE software select the board NodeMCU. If the board isn't in boards list, open the boards manager, on the search line enter esp8266 and download all ESP8266 boards  
// 2.  Open the Virtuino library file: VirtuinoESP.h,  enable the line #define NODEMCU and disable the other boards
// 3.  Connect the NODEMCU ESP8266 module to usb and upload this code

#include <ESP8266WiFi.h>
#include "Virtuino_ESP_WifiServer.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(13, 15); // RX, TX
int mass[13];
String str;

const char* ssid = "WIFI NETWORK NAME";    
const char* password = "PASSWORD";
WiFiServer server(8000);                      // Server port


Virtuino_ESP_WifiServer virtuino(&server);

//NodeMCU ESP8266 pins= D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10                  

// Example variables
 int storedValue=0;
 int counter =0;
 long storedTime=0;

//============================================================== setup
//==============================================================
//==============================================================
void setup() {
   //----- Virtuino settings
  virtuino.DEBUG=true;                         // set this value TRUE to enable the serial monitor status
  virtuino.password="1234";                    // Set a password to your web server for more protection 
                                               // avoid special characters like ! $ = @ # % & * on your password. Use only numbers or text characters
  mySerial.begin(9600);
  Serial.begin(9600);                          // Enable this line only if DEBUG=true
  delay(10);

  //----- NodeMCU module settings
  //----- prepare GPIO2
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);
  
  //----  1. Settings as Station - Connect to a WiFi network
//  Serial.println("Connecting to "+String(ssid));
//  
//   // If you don't want to config IP manually disable the next four lines
//  IPAddress ip(192, 168, 1, 150);            // where 150 is the desired IP Address   
//  IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network
//  IPAddress subnet(255, 255, 255, 0);        // set subnet mask to match your network
//  WiFi.config(ip, gateway, subnet);          // If you don't want to config IP manually disable this line
//  
//  WiFi.mode(WIFI_STA);                       // Config module as station only.
//  WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//    }
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println(WiFi.localIP());
 
  //----   2. Settings as Access point - Create a private Wifi Network. Enable the next five lines to use module as Acces point 
   Serial.print("Setting soft-AP ... ");                   // Default IP: 192.168.4.1
   WiFi.mode(WIFI_AP);                                     // Config module as Acces point only.  Set WiFi.mode(WIFI_AP_STA); to config module as Acces point and station
   boolean result = WiFi.softAP("NodeMCU", "12345678");      // SSID: NodeMCU   Password:12345678
   if(result == true)  Serial.println("Server Ready");
   else Serial.println("Failed!");
  
  // ---- Start the server
  server.begin();
  Serial.println("Server started");

 //---- Enter your setup code below
  
  pinMode(D4,OUTPUT);             // On Virtuino panel add a switch to pin D4 to enable or disable the board led
  pinMode(D3,OUTPUT);            // connect a relay or a led to this pin.  On Virtuino panel add a switch to pin D3
  pinMode(D5,INPUT);            // connect a switch.  On Virtuino panel add a Led to pin D5
    

 
}


//============================================================== loop
//==============================================================
//==============================================================
void loop() {
  virtuino.run();
  str="";
  while (mySerial.available()>0){
        char recieved = mySerial.read();
        str += recieved;
        float x, y, theta;
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
            for(int i;i<13;i++){
              Serial.println(String(mass[i],0));
            }
            Serial.println();
            str = "";
        }    
    }
    virtuino.vMemoryWrite(1,mass[9]);
    if (mass[10]<=0){
      virtuino.vMemoryWrite(2,0);
    }else{
      virtuino.vMemoryWrite(2,mass[10]);
    }
    if(mass[11]<=0){
      virtuino.vMemoryWrite(3,0);
    }else{
      virtuino.vMemoryWrite(3,mass[11]);
    }
    virtuino.vMemoryWrite(4,mass[0]);
    virtuino.vMemoryWrite(5,word(mass[5],mass[6]));
    virtuino.vMemoryWrite(6,mass[12]);
    virtuino.vMemoryWrite(7,mass[1]);
    virtuino.vMemoryWrite(8,word(mass[7],mass[8]));
    virtuino.vMemoryWrite(9,mass[2]);
    virtuino.vMemoryWrite(10,mass[3]);
    virtuino.vMemoryWrite(11,mass[4]);
   // enter your loop code here.
   //------ avoid to use delay() function in your code

   
   //--- example 1  Enable or disable the board led using virtual pin
   //--- On Virtuino app panel add a Switch to Virtual Pin 0 to enable or disable the nodemcu board led
  /* int v=virtuino.vDigitalMemoryRead(0);              // Read virtual memory 0 from Virtuino app 
   if (v!=storedValue) {                          
    Serial.println("-------Virtual pin DV0 is changed to="+String(v));
    if (v==1) digitalWrite(D4,0);
    else digitalWrite(D4,1);
    storedValue=v;
   }*/


    //----  Example 2
    //----  Write a random value to virtual pin 2. On virtuino panel add an indicator to V2
    //virtuino.vMemoryWrite(2,random(1000));


    //---- Example 3 
    //---- Create a counter every 5 seconds 
    //---- Use this technique. Avoid to use the delay function
   /* long t= millis();       // read the time
    if (t>storedTime+5000){
      counter++;            // increase counter by 1
      if (counter>20) counter=0;    // limit = 20
      storedTime = t; 
      virtuino.vMemoryWrite(12,counter);      // write counter to virtual pin V12
    }*/
  }





 


