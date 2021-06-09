//file: main.cpp 
#include "Arduino.h"
#include "ardu_encoder.h"
extern "C"{
#include "idf_main.h"
};

/* Local function prototypes */
static void my_encoder_callback(my_encoder_event_t event, int32_t current);

void setup(){
  Serial.println("Starting Arduino Serial");
  Serial.begin(115200);
  /*Start Up FreeRTOS background Core*/
  start_idf_main();
  
}

void loop(){
  int cntr = 0;
  ardu_encoder my_enc(5,14,13,my_encoder_callback);
  Serial.println("In Loop, starting External Code");
  Serial.println("Doing Arduino stuff");
  while(true){
    cntr++;
    Serial.println("looping ");
    delay(1000);
  }
  /*Maybe do not exit loop?*/
}
/*We need to supply the "my_encoder_callback_t" function to encoder class.
This function determines what action arduino code should take based on rotary encoder event */
static void my_encoder_callback(my_encoder_event_t event, int32_t current){
  switch (event)
  {
  case MYENC_BTN_CLICKED:
    Serial.println("BTN Clicked");
    break;
  case MYENC_BTN_LONG:
    Serial.println("BTN Held down");
    break;
  case MYENC_BTN_PUSHED:
    Serial.println("BTN Pushed down");
    break;
  case MYENC_BTN_RELEASED:
    Serial.println("BTN Released");
    break;
  case MYENC_POS_DEC:
    Serial.println("Encoder Rotation Increase");
    break;
  case MYENC_POS_INC:
    Serial.println("Encoder Rotation Decrease");
    break;
  default:
    break;
  }

}