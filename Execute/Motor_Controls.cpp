#include "Arduino.h"
#include "Motor_Controls.h"
#include "Timers.h"

#define SPEED_VALUES        11
#define PULSE_TIMER_ID      3
#define PULSE_TIMER_DELAY   200

static const int speeds [SPEED_VALUES] = {0, 26, 51, 77, 102, 127, 153, 179, 204, 229, 255 };


static unsigned char wheel_r_dir_pin;
static unsigned char wheel_r_enable_pin;

static unsigned char wheel_l_dir_pin;
static unsigned char wheel_l_enable_pin;

static unsigned char is_pulsing;
void (*pulse_fn)();// pointer to last pulse function

/* -------- Prototypes -------- */
unsigned char translate_speed(unsigned char);
void r_speed(unsigned char);
void l_speed(unsigned char);
void r_dir(unsigned char);
void l_dir(unsigned char);
void start_pulse_timer();
void no_pulse(); // a palceholder function that doesn't do anything;
unsigned char pulse_timer_finished();

/* -------- Public functions ----- */

// todo - be careful of overflow and people trying more than 255
// todo - accurate timing and motor speed
// todo - accounting for variation etc...
// Initialization

void motor_control_init(unsigned char wheel_r_dir, unsigned char wheel_r_enable, unsigned char wheel_l_dir, unsigned char wheel_l_enable){
    wheel_r_dir_pin = wheel_r_dir;
    wheel_r_enable_pin = wheel_r_enable;

    wheel_l_dir_pin = wheel_l_dir;
    wheel_l_enable_pin = wheel_l_enable;

    is_pulsing = false;

    pinMode(wheel_r_dir_pin, OUTPUT);
    pinMode(wheel_r_enable_pin, OUTPUT);
    pinMode(wheel_l_dir_pin, OUTPUT);
    pinMode(wheel_l_enable_pin, OUTPUT);
}


// Move with both motors in same speed. 
void move_forwards(unsigned char speed){
    r_speed(speed);
    r_dir(WHEEL_FORWARD);
    l_speed(speed);
    l_dir(WHEEL_FORWARD);
}

void motor_state_changed(){
    // reset the pulse timer?
    is_pulsing = false;
    pulse_fn = no_pulse;
}

void no_pulse(){

}

// i could write a wrapper pulse fn. but no. 

void pulse_forward(){ // be able to pass in the interval?
    // Serial.println("pulse_forward");
    move_forwards(10);
    start_pulse_timer();
    pulse_fn = pulse_forward;
    is_pulsing = true;
}

void pulse_backwards(){
    move_backwards(10);
    start_pulse_timer();
    pulse_fn = pulse_forward;
    is_pulsing = true;
}

void pulse_rotate_right(){
    // Serial.println("pulse_rotate_right");
    rotate_right(10);
    start_pulse_timer();
    pulse_fn = pulse_rotate_right;
    is_pulsing = true;
}



void check_pulse(){
    if (is_pulsing){
        Serial.println("Pulsing");
    } else {
        Serial.println("not pulsing");
    }
    if (is_pulsing && pulse_timer_finished()){
        stop_moving();
        is_pulsing = false;
        start_pulse_timer(); // delay timer
    }
    if (!is_pulsing && pulse_timer_finished()){
        is_pulsing = true;
        pulse_fn();
    }
}

void start_pulse_timer(){
    // starts the pulse timer
    TMRArd_InitTimer(PULSE_TIMER_ID, PULSE_TIMER_DELAY);
}

unsigned char pulse_timer_finished(){
    return TMRArd_IsTimerExpired(PULSE_TIMER_ID);
}


void move_backwards(unsigned char speed){
    r_speed(speed);
    r_dir(WHEEL_BACKWARD);
    l_speed(speed);
    l_dir(WHEEL_BACKWARD);
}

// This should be called all the time
void stop_moving(){
    r_speed(0);
    l_speed(0);
}

// Rotating in place
void rotate_right(unsigned char speed){
    l_speed(speed);
    l_dir(WHEEL_FORWARD);
    r_speed(speed);
    r_dir(WHEEL_BACKWARD);
}

void rotate_left(unsigned char speed){
    l_speed(speed);
    l_dir(WHEEL_BACKWARD);
    r_speed(speed);
    r_dir(WHEEL_FORWARD);
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
    if (speed > 10){
        Serial.print("Chose a speed over 10: ");
        Serial.println(speed);
        return 127;
    }
    return speeds[speed];
}

void r_speed(unsigned char speed){
    unsigned char translated_speed = translate_speed(speed);
    analogWrite(wheel_r_enable_pin, translated_speed );
}

void l_speed(unsigned char speed){
    unsigned char translated_speed = translate_speed(speed );
    analogWrite(wheel_l_enable_pin, translated_speed);
}

void r_dir(unsigned char direction){
    digitalWrite(wheel_r_dir_pin, direction);
}

void l_dir(unsigned char direction){
    digitalWrite(wheel_l_dir_pin, !direction);
}



