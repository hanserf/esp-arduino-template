//file: main.cpp
#include "Arduino.h"
extern "C"{
#include "idf_main.h"
};
void setup(){
  Serial.println("Starting Arduino Serial");
  Serial.begin(115200);
}

void loop(){
  int cntr = 0;
  Serial.println("In Loop, starting External Code");
  start_idf_main();
  Serial.println("Doing Arduino stuff");
  while(true){
    cntr++;
    Serial.println("looping ");
    delay(1000);
  }
  /*Maybe do not exit loop?*/
}
