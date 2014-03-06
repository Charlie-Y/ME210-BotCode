// A little library that is getting me rehabituated with 
// the file sharing and classes of C++

#include "Arduino.h"
#include "Debug_Leds.h"

// Constructor 
Debug_Led::Debug_Led(unsigned char pin){
    led_pin = pin;
    is_on = false;

    pinMode(led_pin, OUTPUT);
    // Serial.print("Setup pin: " );
    // Serial.println(led_pin);
}

void Debug_Led::led_on(){
    digitalWrite(led_pin, HIGH);
    is_on = true;

    // Serial.print("Debug led on: " );
    // Serial.println(led_pin);
}

void Debug_Led::led_off(){
    digitalWrite(led_pin, LOW);
    is_on = false;

    // Serial.print("Debug led off: " );
    // Serial.println(led_pin);
}

void Debug_Led::toggle(){
    if (is_on){
        led_off();
    } else {
        led_on();
    }
}

// void flash_like_you_mean_it(){

// }

// void stop_flashing___please(){
    
// }


// void init(unsigned char r,unsigned char g,unsigned char b){
//     red_pin = r;
//     green_pin = g;
//     blue_pin = b;
//     pinMode(red_pin, OUTPUT);
//     pinMode(green_pin, OUTPUT);
//     pinMode(blue_pin, OUTPUT);
//     Serial.println("red_pin: ");
//     Serial.println(red_pin);
//     Serial.println("green_pin: ");
//     Serial.println(red_pin);
//     Serial.println("blue_pin: ");
//     Serial.println(red_pin);
// }


// void clear_debug_leds(){

// }

// void debug_red_on(){

// }

// void debug_green_on(){

// }

// void debug_blue_on(){

// }