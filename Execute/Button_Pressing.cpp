#include "Arduino.h"
#include "Button_Pressing.h"
#include "Servo.h" // Arduino Server library

static unsigned char button_direction_pin; // digital
static unsigned char button_enable_pin; // pwm

void button_pressing_init( unsigned char button_enable, unsigned char button_direction){
    button_enable_pin = button_enable;
    button_direction_pin = button_direction;
    pinMode(button_enable_pin, OUTPUT);
    pinMode(button_direction_pin, OUTPUT);
};


void extend_button_presser(){
    // probably write a high signal to button 
};

void retract_button_presser(){

};

