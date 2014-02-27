#include "Arduino.h"
#include "Timers.h"

#include "State_Machine.h"
#include "Debug_Leds.h"
#include "Tape_Sensing.h"
#include "Beacon_Sensing.h"
#include "Motor_Controls.h"

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
#define TAPE_SENSOR_R_PIN       A0
#define TAPE_SENSOR_L_PIN       A1

// more digital reads
// #define BUMPER_R_PIN
// #define BUMPER_R_PIN

// ---------  Output pins  ----------- //

// motor wheel - RIGHT
#define WHEEL_R_ENABLE_PIN      5// pwm
#define WHEEL_R_DIRECTION_PIN   7//digital

// motor wheel - LEFT
#define WHEEL_L_ENABLE_PIN      6// pwm
#define WHEEL_L_DIRECTION_PIN   8//digital
    
// coin drop pins
// #define COIN_DROP_PIN // digital

// platform raising. its a stepper motor
// #define PLATFORM_RAISE_PIN // pwm

// button pressing
// #define BUTTON_PRESSER_DIRECTION_PIN
// #define BUTTON_PRESSER_SPEED_PIN


#define DEBUG_RED               A5
#define DEBUG_GREEN             A4
#define DEBUG_BLUE              A3

typedef enum{
    // all the state names
    STARTUP,
    SERVER_BEACON_SENSED,
    DEPO_BEACON_SENSED,
    TAPE_R_SENSED,
    TAPE_L_SENSED,
    TAPE_BOTH_SENSED,
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
unsigned char side_1; // which side of the board we are on
static unsigned char current_state = STARTUP;

unsigned char current_coin_count;
unsigned char coins_taken_from_server;
unsigned char times_button_pressed_for_coin;


// Other classes
static Debug_Led *debug_red;
static Debug_Led *debug_green;
static Debug_Led *debug_blue;

static Tape_Sensor *tape_r;
static Tape_Sensor *tape_l;


// -------------- Prototypes ------------------------------- //

void debug_all_off(void);


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
    debug_all_off();

    current_state = new_state;
    entered_state = true;
    state_changed = true;
}

void log_states(){
    Serial.println("IT WORKS");
}

// -------------- Event testing functions ----------- //
// various event testers that return true or false

unsigned char respond_to_key(unsigned char new_state){
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
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }

}

unsigned char respond_to_beacon(unsigned char new_state){
    if (beacon_found()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_no_beacon(unsigned char new_state){
    if (no_beacon_found()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_depository_found(unsigned char new_state){
    // change_state_to(new_state)
    if (depository_found()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_server_found(unsigned char new_state){
    // implement
    // if returns true
    // change_state_to(new_state)
    if (server_found()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_tape_r_on(unsigned char new_state){
    if (tape_r->is_on_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_tape_l_on(unsigned char new_state){
    if (tape_l->is_on_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_tape_r_off(unsigned char new_state){
    if (tape_r->is_off_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_tape_l_off(unsigned char new_state){
    if (tape_l->is_off_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_tape(unsigned char new_state){
    return false;
}


// -------------- Specific state functions ---------- //

// each state method implements code that tells the robot 
// what it should be doing in that state, which will be executed once
// event checking also happens here;

// The main startup one. all the inits sill be called here
void startup_fn(){
    Serial.println("startup_fn");

    debug_red = new Debug_Led(DEBUG_RED);
    debug_green = new Debug_Led(DEBUG_GREEN);
    debug_blue = new Debug_Led(DEBUG_BLUE);

    tape_r = new Tape_Sensor(TAPE_SENSOR_R_PIN);
    tape_l = new Tape_Sensor(TAPE_SENSOR_L_PIN);

    beacon_sensing_init(INTERRUPT_ID);
    motor_control_init(WHEEL_R_DIRECTION_PIN, WHEEL_R_ENABLE_PIN, WHEEL_L_DIRECTION_PIN, WHEEL_L_ENABLE_PIN);

    // ==== yet to implement ===== //
    // button_pressing_init()
    // coin_control_init()

    Serial.println("end startup_fn");

    change_state_to(NULL_STATE);
}


void server_beacon_sensed_fn(){
    if (entered_state){
        // this code will only be executed once
        Serial.println("entered server_beacon_sensed_fn");
        debug_blue->led_on();
    }
    // Test for all events
    if (respond_to_no_beacon(NULL_STATE)) return;
    if (respond_to_depository_found(DEPO_BEACON_SENSED)) return;
}

void depo_beacon_sensed_fn(){
    if (entered_state){
        // this code will only be executed once
        Serial.println("entered depo_beacon_sensed_fn");
        debug_green->led_on();
    }
    if (respond_to_no_beacon(NULL_STATE)) return;
    if (respond_to_server_found(SERVER_BEACON_SENSED)) return;
}

void tape_r_sensed_fn(){
    if (entered_state){
        debug_red->led_on();
        Serial.println("tape_r_sensed_fn");
    }
    if (respond_to_tape_r_off(NULL_STATE)) return;
    if (respond_to_tape_l_on(TAPE_BOTH_SENSED)) return;
}

void tape_l_sensed_fn(){
    if (entered_state){
        debug_blue->led_on();
        Serial.println("tape_l_sensed_fn");
    }
    if (respond_to_tape_l_off(NULL_STATE)) return;
    if (respond_to_tape_r_on(TAPE_BOTH_SENSED)) return;

}

void tape_both_sensed_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("tape_both_sensed_fn");
    }
    if (respond_to_tape_l_off(TAPE_R_SENSED)) return;
    if (respond_to_tape_r_off(TAPE_L_SENSED)) return;
}

// -- Various debugging states -- //
// send things here to end the loop

void null_state_fn(){
    if (entered_state){
        Serial.println("null_state_fn");
    }

    // if (respond_to_key(NULL_STATE)) return;
    // if (respond_to_depository_found(DEPO_BEACON_SENSED)) return;
    // if (respond_to_server_found(SERVER_BEACON_SENSED)) return;
    // if (respond_to_tape_r_on(TAPE_R_SENSED)) return;
    // if (respond_to_tape_l_on(TAPE_L_SENSED)) return;
}

// -------------- Utility Methods ------------ //

void debug_all_off(){
    debug_red->led_off();
    debug_green->led_off();
    debug_blue->led_off();
}

// -------------- Setup Methods ------------ //
// this goes at the end because the functions need to be defined first
void setup_states() {
    //link each state to a function
    state_functions[STARTUP] = startup_fn;
    state_functions[SERVER_BEACON_SENSED] = server_beacon_sensed_fn;
    state_functions[DEPO_BEACON_SENSED] = depo_beacon_sensed_fn;
    state_functions[TAPE_R_SENSED] = tape_r_sensed_fn;
    state_functions[TAPE_L_SENSED] = tape_l_sensed_fn;
    state_functions[TAPE_BOTH_SENSED] = tape_both_sensed_fn;
    state_functions[NULL_STATE] = null_state_fn;
}
