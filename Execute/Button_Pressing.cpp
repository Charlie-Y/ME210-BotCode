#include "Arduino.h"
#include "Button_Pressing.h"
#include "Servo.h" // Arduino Servo library

// Right now the servo is bw, so the positions are a bit weird
// and the math is backwards. 

#define SERVO_MIN           90 // the axis of the servo
#define SERVO_MAX           40 // CCW from top
#define SERVO_INTERVAL      50 // if this is too large does it explode? // why did it explode?

#define SERVO_EXTENDING     0
#define SERVO_RETRACTING    1


static unsigned char button_presser_pin; // pwm
static Servo servo;
static int position = SERVO_MIN;
static unsigned char current_servo_direction;
static int times_pressed;

void button_pressing_init( unsigned char button_presser){
    button_presser_pin = button_presser;
    servo.attach(button_presser_pin);
    servo.write(SERVO_MIN);
    current_servo_direction = SERVO_EXTENDING;
    times_pressed = 0;
};

// extends the servo a bit
void extend_button_presser(){
    position -= SERVO_INTERVAL;

    Serial.print("Extended to pos: ");
    Serial.println(position);

    servo.write(position);
    times_pressed++;

    current_servo_direction = SERVO_EXTENDING;
}


// retracts the servo a bit
void retract_button_presser(){
    position += SERVO_INTERVAL;

    Serial.print("Retracted to pos: ");
    Serial.println(position);

    servo.write(position);


    current_servo_direction = SERVO_RETRACTING;
}

// should return true if finished moving in any direction
unsigned char button_presser_finished(){
    if (current_servo_direction == SERVO_EXTENDING){
        return (position <= SERVO_MAX);
    } else if (current_servo_direction == SERVO_RETRACTING) {
        return (position >= SERVO_MIN);
    }
    return true; 
}

int times_button_pressed(){
    return times_pressed;
}

int times_for_num_coins(int num_coins){
    int total = 0;
    for (int i = 0; i < num_coins; ++i)
    {
        total += i + 1;
    }
    return total;
}

unsigned char pressed_enough_times_for_coins(int num_coins){
    return times_for_num_coins(num_coins) <= times_pressed;
}

// I'll need to work out the basics. 
// it looks like we will need a state diagram of some sort. 

// void extend_button_presser(){
//     // probably write a high signal to button 
// };

// void retract_button_presser(){

// };

