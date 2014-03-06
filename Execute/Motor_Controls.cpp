#include "Arduino.h"
#include "Motor_Controls.h"
#include "Timers.h"

#define SPEED_VALUES        11
#define PULSE_TIMER_ID      3
#define PULSE_TIMER_DELAY   200
#define BATTERY_ADJUST      0

static const int speeds [SPEED_VALUES] = {0, 26, 51, 77, 102, 127, 153, 179, 204, 229, 255 };


static unsigned char wheel_r_dir_pin;
static unsigned char wheel_r_enable_pin;

static unsigned char wheel_l_dir_pin;
static unsigned char wheel_l_enable_pin;

static unsigned char is_pulsing;
static unsigned char pulse_direction;
static unsigned int current_pulse_delay;
void (*pulse_fn)();// pointer to last pulse function

/* -------- Prototypes -------- */
unsigned char translate_speed(unsigned char);
void r_speed(unsigned char);
void l_speed(unsigned char);
void r_dir(unsigned char);
void l_dir(unsigned char);
void start_pulse_timer(int);
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
    current_pulse_delay = PULSE_TIMER_DELAY;
}

void no_pulse(){

}

// i could write a wrapper pulse fn. but no. 
// each pulse function needs to:
    // call start_timer
    // set a current_pulse_delay - for how long the pulse is off
    // set the right pulse_fn pointer
    // set is_pulsing to true;

void pulse_forward(){ // be able to pass in the interval?
    // Serial.println("pulse_forward");
    move_forwards(10);
    current_pulse_delay = 120;
    start_pulse_timer(80);
    pulse_fn = pulse_forward;
    is_pulsing = true;
}

void pulse_fine_forward(){
    move_forwards(10);
    current_pulse_delay = 200;
    start_pulse_timer(40);
    pulse_fn = pulse_fine_forward;
    is_pulsing = true;
}

void pulse_backward(){
    move_backwards(10);
    current_pulse_delay = 200;
    start_pulse_timer(100);
    pulse_fn = pulse_forward;
    is_pulsing = true;
}

void pulse_fine_backward(){
    move_backwards(9);
    current_pulse_delay = 200;
    start_pulse_timer(100);
    pulse_fn = pulse_fine_backward;
    is_pulsing = true;
}

void pulse_fine_rotate_right(){
    rotate_right(9);
    current_pulse_delay = 140;
    start_pulse_timer(40);
    pulse_fn = pulse_fine_rotate_right;
    is_pulsing = true;
}

void pulse_fine_rotate_left(){
    rotate_left(9);
    current_pulse_delay = 140;
    start_pulse_timer(40);
    pulse_fn = pulse_fine_rotate_left;
    is_pulsing = true;
}

void pulse_rotate_right(){
    // Serial.println("pulse_rotate_right");
    // this should be faster... should make delay a variable
    rotate_right(10);
    current_pulse_delay = 150;
    start_pulse_timer(40);
    pulse_fn = pulse_rotate_right;
    is_pulsing = true;
}

void pulse_rotate_left(){
    // Serial.println("pulse_rotate_right");
    // this should be faster... should make delay a variable
    rotate_left(10);
    current_pulse_delay = 150;
    start_pulse_timer(40);
    pulse_fn = pulse_rotate_left;
    is_pulsing = true;
}

void pulse_arc_back_inner(){
    is_pulsing = true;
    pulse_fn = pulse_arc_back_inner;
    start_pulse_timer(130);
    current_pulse_delay = 100;

    r_dir(WHEEL_BACKWARD);
    l_dir(WHEEL_BACKWARD);

    if (pulse_direction == DIR_LEFT){
        // arc back to the left. oriented facing forward
        // Serial.println("BACK LEFT");
        r_speed(10);
        l_speed(4);
    } else if (pulse_direction == DIR_RIGHT){
        // Serial.println("BACK RIGHT");
        r_speed(4);
        l_speed(10);
    }
}

//wrapper function and class variable that goes around function pointer
// issues
void pulse_arc_back(unsigned char direction){
    pulse_direction = direction;
    pulse_arc_back_inner();
}


void check_pulse(){
    if (is_pulsing){
        // Serial.println("Pulsing");
    } else {
        // Serial.println("not pulsing");
    }
    if (is_pulsing && pulse_timer_finished()){
        stop_moving();
        is_pulsing = false;
        start_pulse_timer(current_pulse_delay); // delay timer
    }
    if (!is_pulsing && pulse_timer_finished()){
        is_pulsing = true;
        pulse_fn();
    }
}

void start_pulse_timer(int delay_length){
    // starts the pulse timer
    TMRArd_InitTimer(PULSE_TIMER_ID, delay_length);
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
    r_speed(speed);
    l_speed(0);
    l_dir(WHEEL_FORWARD);
}

void pivot_left(unsigned char speed){
    l_speed(speed);
    r_speed(0);
    r_dir(WHEEL_FORWARD);
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

void arc_back(unsigned char direction){
    r_dir(WHEEL_BACKWARD);
    l_dir(WHEEL_BACKWARD);

    if (direction == DIR_LEFT){
        // arc back to the left. oriented facing forward
        // Serial.println("BACK LEFT");
        r_speed(10);
        l_speed(9);
    } else if (direction == DIR_RIGHT){
        // Serial.println("BACK RIGHT");

        r_speed(9);
        l_speed(10);
    }
}

/* -------- Private functions ----- */


unsigned char translate_speed(unsigned char speed){
    if (speed > 10){
        Serial.print("Chose a speed over 10: ");
        Serial.println(speed);
        return 127;
    }
    if (speed < BATTERY_ADJUST) {
        return speeds[0];
    }
    return speeds[speed - BATTERY_ADJUST];
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



