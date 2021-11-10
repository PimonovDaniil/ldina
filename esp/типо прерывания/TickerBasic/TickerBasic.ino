#include <Ticker.h>

Ticker flipper;


void flip() {
  Serial.println("lol");
}

void setup() {
  Serial.begin(9600);
  flipper.attach(3, flip);
}

void loop() {
  
}
