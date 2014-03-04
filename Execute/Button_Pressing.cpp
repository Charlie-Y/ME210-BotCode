#include "Arduino.h"
#include "Button_Pressing.h"
#include "Servo.h" // Arduino Servo library

#define SERVO_MIN           0 // the axis of the servo
#define SERVO_MAX           90 // CCW from top
#define SERVO_INTERVAL      90 // if this is too large does it explode? // why did it explode?

#define SERVO_EXTENDING     0
#define SERVO_RETRACTING    1


static unsigned char button_presser_pin; // pwm
static Servo servo;
static int position = SERVO_MIN;
static unsigned char current_servo_direction;


void button_pressing_init( unsigned char button_presser){
    button_presser_pin = button_presser;
    servo.attach(button_presser_pin);
    servo.write(SERVO_MIN);
    current_servo_direction = SERVO_EXTENDING;
};

// extends the servo a bit
void extend_button_presser(){
    servo.write(position);
    // Serial.print("Extended to pos: ");
    // Serial.println(position);
    position += SERVO_INTERVAL;
    current_servo_direction = SERVO_EXTENDING;
}


// retracts the servo a bit
void retract_button_presser(){
    servo.write(position);
    position -= SERVO_INTERVAL;
    current_servo_direction = SERVO_RETRACTING;
}

// should return true if finished moving in any direction
unsigned char button_presser_finished(){
    if (current_servo_direction == SERVO_EXTENDING){
        return (position >= SERVO_MAX);
    } else if (current_servo_direction == SERVO_RETRACTING) {
        return (position <= SERVO_MIN);
    }
    return true; 
}

// I'll need to work out the basics. 
// it looks like we will need a state diagram of some sort. 

// void extend_button_presser(){
//     // probably write a high signal to button 
// };

// void retract_button_presser(){

// };

