#include "Arduino.h"
#include "Dumping.h"
#include "Servo.h" // Arduino Servo library

#define SERVO_MIN           90 // the axis of the servo
#define SERVO_MAX           150 // CCW from top
#define SERVO_INTERVAL      30 // if this is too large does it explode? // why did it explode?

#define SERVO_EXTENDING     0
#define SERVO_RETRACTING    1


static unsigned char dumping_pin; // pwm
static Servo servo;
static int position = SERVO_MIN;
static unsigned char current_servo_direction;


void dumping_init( unsigned char dumping){
    dumping_pin = dumping;
    servo.attach(dumping_pin);
    servo.write(SERVO_MIN);
    current_servo_direction = SERVO_EXTENDING;
};

// extends the servo a bit
void extend_dumper(){
    servo.write(position);
    // Serial.print("Extended to pos: ");
    // Serial.println(position);
    position += SERVO_INTERVAL;
    current_servo_direction = SERVO_EXTENDING;
}


// retracts the servo a bit
void retract_dumper(){
    servo.write(position);
    position -= SERVO_INTERVAL;
    current_servo_direction = SERVO_RETRACTING;
}

