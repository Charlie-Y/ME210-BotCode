#include "Arduino.h"
#include "State_Machine.h"
#include "Debug_Leds.h"

/* ======================================== */
/* ============== Defines ================= */
/* ======================================== */

#define DEBUGGING               1// true or false. all serial prints should have a if(DEBUGGING) Serial.println(etc)
// and we can have different types of debugging, like state tracking
#define LOG_STATE               1


// ---------  Input pins  ----------- //

#define IR_SENSOR_PIN           2 // infrared sensor - needs digital pin 2
#define INTERRUPT_ID            0 // for some reason interrupt 0 maps to pin 2

// digital reads
#define TAPE_SENSOR_R_PIN       A5
#define TAPE_SENSOR_L_PIN       A6

// more digital reads
// #define BUMPER_R_PIN
// #define BUMPER_R_PIN

// ---------  Output pins  ----------- //

// motor wheel - RIGHT
// #define WHEEL_R_DIRECTION_PIN //digital
// #define WHEEL_R_SPEED_PIN // pwm

// motor wheel - LEFT
// #define WHEEL_L_DIRECTION_PIN //digital
// #define WHEEL_L_SPEED_PIN // pwm
    
// coin drop pins
// #define COIN_DROP_PIN // digital

// platform raising. its a stepper motor
// #define PLATFORM_RAISE_PIN // pwm


#define DEBUG_RED               A1
#define DEBUG_GREEN             A2
#define DEBUG_BLUE              A3

typedef enum{
    // all the state names
    STARTUP,
    BEACON_3K_SENSED,
    BEACON_850_SENSED,
    TAPE_R_SENSED,
    TAPE_L_SENSED,
    NULL_STATE,
    // last typedef enum value is guaranteed to be > then the 
    // # of states there are. kudos to the guy who thought of this
    NUM_STATES 
} STATES;

// array of functions for each state
void (*state_functions[NUM_STATES])(); 

// the state the machine is in
unsigned char state_changed;
unsigned char entered_state;
static unsigned int current_state = STARTUP;


// -------------- Generic State handling methods --------- //
// these are pretty much taken directly from the code. its a great architecture. 


void execute_current_state(new_state){
    // use the current state to access the array index of the 
    // function that i want
    state_functions[current]();
    if (!state_changed) {entered_state = false};
    state_changed = false;
}

void change_state(){
    // clear all the relevant timers
    current_state = new_state;
    entered_state = true;
    state_changed = true;
}

void log_states(){
    Serial.println("IT WORKS");
}

// -------------- Specific state methods ---------- //

// each state method implements code that tells the robot 
// what it should be doing in that state, which will be executed once

// The main startup one. all the inits sill be called here
void startup_fn(){
    Debug.init(DEBUG_RED, DEBUG_GREEN, DEBUG_BLUE);
}

void beacon_3k_sensed_fn(){

}

void beacon_850_sensed_fn(){

}

void tape_r_sensed_fn(){

}

void tape_l_sensed_fn(){

}

// send things here to end the loop
void null_state_fn(){

}

// -------------- Setup Methods ------------ //
// this goes at the end because the functions need to be defined first
void setup_states() {
    //link each state to a function
    // state_functions[] = ;
    state_functions[STARTUP] = startup_fn;
    state_functions[BEACON_3K_SENSED] = beacon_3k_sensed_fn;
    state_functions[BEACON_850_SENSED] = beacon_850_sensed_fn;
    state_functions[TAPE_R_SENSED] = tape_r_sensed_fn;
    state_functions[TAPE_L_SENSED] = tape_l_sensed_fn;
    state_functions[NULL_STATE] = null_state_fn;
    

}
