#include "Arduino.h"
#include "Button_Pressing.h"


static unsigned char button_direction_pin;
static unsigned char button_enable_pin;

void button_pressing_init( unsigned char pin1, unsigned char pin2){
    button_enable_pin = pin1;
    button_direction_pin = pin2;
};

void extend_button_presser(){
    // probably write a high signal to button 
};

void retract_button_presser(){

};