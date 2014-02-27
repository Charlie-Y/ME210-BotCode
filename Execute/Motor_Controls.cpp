#include "Arduino.h"
#include "Motor_Controls.h"

#define SPEED_VALUES        11


static const int speeds [SPEED_VALUES] = [ 0, 26, 51, 77, 102, 127, 153, 179, 204, 229, 255 ];


static unsigned char wheel_r_dir_pin;
static unsigned char wheel_r_speed_pin;

static unsigned char wheel_l_dir_pin;
static unsigned char wheel_l_speed_pin;


/* -------- Prototypes -------- */
unsigned char translate_speed(unsigned char);
void r_speed(unsigned char);
void l_speed(unsigned char);
void r_dir(unsigned char);
void l_dir(unsigned char);

/* -------- Public functions ----- */

// todo - be careful of overflow and people trying more than 255
// todo - accurate timing and motor speed
// todo - accounting for variation etc...
// Initialization

void motor_control_init(unsigned char wheel_r_dir, unsigned char wheel_r_speed, unsigned char wheel_l_dir, unsigned char wheel_l_speed){
    wheel_r_dir_pin = wheel_r_dir;
    wheel_r_speed_pin = wheel_r_speed;

    wheel_l_dir_pin = wheel_l_dir;
    wheel_l_speed_pin = wheel_l_speed;

    pinMode(wheel_r_dir_pin, OUTPUT);
    pinMode(wheel_l_dir_pin, OUTPUT);
}


// Move with both motors in same speed. 
void move_forwards(unsigned char speed){
    r_speed(speed);
    r_dir(WHEEL_FORWARD);
    l_speed(speed);
    l_dir(WHEEL_FORWARD);
}

void move_backwards(unsigned char speed){
    r_speed(speed);
    r_dir(WHEEL_BACKWARD);
    l_speed(speed);
    l_dir(WHEEL_BACKWARD);
}

// This should be called all the time
void stop_moving(unsigned char speed){
    r_speed(0);
    l_speed(0);
}

// Rotating in place
void rotate_right(unsigned char speed){
    r_speed(speed);
    r_dir(WHEEL_FORWARD);
    l_speed(speed);
    l_dir(WHEEL_BACKWARD);
}

void rotate_left(unsigned char speed){
    r_speed(speed);
    r_dir(WHEEL_BACKWARD);
    l_speed(speed);
    l_dir(WHEEL_FORWARD);
}

void rotate_dir(unsigned char speed, unsigned char direction){
    if (direction == DIR_LEFT){
        rotate_left(speed);
    } else {
        rotate_right(speed);
    }
}

// Pivoting on one wheel .
void pivot_right(unsigned char speed){
    r_speed(0);
    l_speed(speed);
    l_dir(WHEEL_FORWARD);
}

void pivot_left(unsigned char speed){
    r_speed(speed);
    l_speed(0);
    l_dir(WHEEL_FORWARD);
}

void pivot_dir(unsigned char speed, unsigned char direction){
    if (direction == DIR_LEFT){
        pivot_left(speed);
    } else {
        pivot_right(speed);
    }
}

// Moving in an arc. Necessary. 
void arc_right(unsigned char speed, unsigned char ratio){

}

void arc_left(unsigned char speed, unsigned char ratio){

}

void arc_dir(unsigned char speed, unsigned char some_param, unsigned char direction){

}

/* -------- Private functions ----- */


unsigned char translate_speed(unsigned char speed){
    return speeds[speed];
}

void r_speed(unsigned char speed){
    unsigned char translated_speed = translate_speed(speed);
    analogWrite(wheel_r_speed_pin, translated_speed );
}

void l_speed(unsigned char speed){
    unsigned char translated_speed = translate_speed(speed);
    analogWrite(wheel_l_speed_pin, translated_speed);
}

void r_dir(unsigned char direction){
    digitalWrite(wheel_r_dir_pin, direction);
}

void l_dir(unsigned char direction){
    digitalWrite(wheel_l_dir_pin, direction);
}



