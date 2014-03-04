#include "Arduino.h"
#include "Bump_Sensing.h"


Bump_Sensor::Bump_Sensor(unsigned char pin){
    sensor_pin = pin;
    pinMode(sensor_pin, INPUT);
    Serial.print("Setup Bumper pin: ");
    Serial.println(sensor_pin);
}

unsigned char Bump_Sensor::is_bumped(){
    unsigned char val = digitalRead(sensor_pin);
    return (val == HIGH);
}

unsigned char Bump_Sensor::is_not_bumped(){
    unsigned char val = digitalRead(sensor_pin);
    return (val == LOW);
}
