#include "Arduino.h"
#include "Tape_Sensing.h"


Tape_Sensor::Tape_Sensor(unsigned char pin){
    sensor_pin = pin;
    pinMode(sensor_pin, INPUT);
    Serial.print("Setup tape pin: " );
    Serial.println(sensor_pin);
}

// expects a comparator input

// todo - need to move band lower to 0 V. More like .5V
unsigned char Tape_Sensor::is_on_tape(){
    unsigned char val = digitalRead(sensor_pin);
    return (val == HIGH);
}

unsigned char Tape_Sensor::is_off_tape(){
    unsigned char val = digitalRead(sensor_pin);
    return (val == LOW);
}