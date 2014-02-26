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


#define DEBUG_RED               A5
#define DEBUG_GREEN             A4
#define DEBUG_BLUE              A3

typedef enum{
    // all the state names
    STARTUP,
    BEACON_3K_SENSED,
    BEACON_850_SENSED,
    TAPE_R_SENSED,
    TAPE_L_SENSED,
    NULL_STATE,
    DEBUG_RED_STATE,
    DEBUG_GREEN_STATE,
    DEBUG_BLUE_STATE,
    // last typedef enum value is guaranteed to be > then the 
    // # of states there are. kudos to the guy who thought of this
    NUM_STATES 
} STATES;

// array of functions for each state
void (*state_functions[NUM_STATES])(); 

// the state the machine is in
unsigned char state_changed;
unsigned char entered_state;
unsigned char side_1; // which side of the board we are on
static unsigned char current_state = STARTUP;

// Other classes
static Debug_Led *debug_red;
static Debug_Led *debug_green;
static Debug_Led *debug_blue;


// -------------- Generic State handling functions --------- //
// these are pretty much taken directly from the code. its a great architecture. 


void execute_current_state(){
    // use the current state to access the array index of the 
    // function that i want
    state_functions[current_state]();
    if (!state_changed) entered_state = false;
    state_changed = false;
}

void change_state_to(unsigned char new_state){
    // clear all the relevant timers
    current_state = new_state;
    entered_state = true;
    state_changed = true;
}

void log_states(){
    Serial.println("IT WORKS");
}

// -------------- Event testing functions ----------- //
// various event testers that return true or false

unsigned char test_for_key(unsigned char new_state){
    // ignore new_state. this is for debugging purposes
    if (Serial.available()){
        unsigned char key = Serial.read();
        Serial.print("Key pressed: ");
        Serial.println(key);
        switch(key){
            case('r'): 
                debug_red->toggle(); 
                break;
            case('g'): 
                debug_green->toggle(); 
                break;
            case('b'): 
                debug_blue->toggle(); 
                break;
        }
    }
}

unsigned char test_for_depository(unsigned char new_state){
    // implement

    // if it returns true 
    // change_state_to(new_state)

    // else 
    return false;
}

unsigned char test_for_server(unsigned char new_state){
    // implement
    // if returns true
    // change_state_to(new_state)
    return false;
}

unsigned char test_for_tape_r(unsigned char new_state){
    return false;
}

unsigned char test_for_tape_l(unsigned char new_state){
    return false;
}

unsigned char test_for_tape(unsigned char new_state){
    return false;
}


// -------------- Specific state functions ---------- //

// each state method implements code that tells the robot 
// what it should be doing in that state, which will be executed once

// The main startup one. all the inits sill be called here
void startup_fn(){
    Serial.println("startup_fn");

    debug_red = new Debug_Led(DEBUG_RED);
    debug_green = new Debug_Led(DEBUG_GREEN);
    debug_blue = new Debug_Led(DEBUG_BLUE);

    Serial.println("end startup_fn");

    change_state_to(NULL_STATE);
}

void beacon_3k_sensed_fn(){
    if (entered_state){
        // this code will only be executed once
        Serial.println('entered beacon_3k_sensed_fn');
    }
    // Test for all events
    if (test_for_depository(BEACON_850_SENSED)) return;
    if (test_for_tape_r(TAPE_R_SENSED)) return;
    if (test_for_tape_l(TAPE_L_SENSED)) return;
}

void beacon_850_sensed_fn(){

}

void tape_r_sensed_fn(){

}

void tape_l_sensed_fn(){

}


// -- Various debugging states -- //
// send things here to end the loop
void null_state_fn(){
    Serial.println("null_state_fn");
    if (test_for_key(NULL_STATE)) return ;
}

void debug_red_state_fn(){
    if (entered_state){

    }
}

void debug_green_state_fn(){
    if (entered_state){

    }
}

void debug_blue_state_fn(){
    if (entered_state){

    }
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
    state_functions[DEBUG_RED_STATE] = debug_red_state_fn;
    state_functions[DEBUG_GREEN_STATE] = debug_green_state_fn;
    state_functions[DEBUG_BLUE_STATE] = debug_blue_state_fn;
}
