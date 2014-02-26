// A little library that is getting me rehabituated with 
// the file sharing and classes of C++

#include "Arduino.h"

static unsigned char red_pin = 11;
static unsigned char green_pin = 11;
static unsigned char blue_pin = 11;

void init(unsigned char r,unsigned char g,unsigned char b){
    red_pin = r;
    green_pin = g;
    blue_pin = b;
    pinMode(red_pin, OUTPUT);
    pinMode(green_pin, OUTPUT);
    pinMode(blue_pin, OUTPUT);
    Serial.println("red_pin: ");
    Serial.println(red_pin);
    Serial.println("green_pin: ");
    Serial.println(red_pin);
    Serial.println("blue_pin: ");
    Serial.println(red_pin);
}


void clear_debug_leds(){

}

void debug_red_on(){

}

void debug_green_on(){

}

void debug_blue_on(){

}