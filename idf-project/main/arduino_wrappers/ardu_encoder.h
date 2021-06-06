#ifndef _ARDU_ENCODER_H
#define _ARDU_ENCODER_H

extern "C"{
#include "my_encoder.h"

};


class ardu_encoder
{
private:
    /* data */
public:
    ardu_encoder(int pina,int pinb, int pin_btn,my_encoder_callback_t event_callback);
    ~ardu_encoder();

};
/*
This is just a wrapper for calling C-Code and starting a Freertos Task handling HW interrups.

*/
    
ardu_encoder::ardu_encoder(int pina,int pinb, int pin_btn,my_encoder_callback_t event_callback ){
    init_encoder(pina,pinb, pin_btn,event_callback);
}

ardu_encoder::~ardu_encoder()
{
    destroy_encoder();
}
#endif