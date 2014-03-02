#include "Arduino.h"
#include "Timers.h"

#include "State_Machine.h"
#include "Debug_Leds.h"
#include "Tape_Sensing.h"
#include "Beacon_Sensing.h"
#include "Motor_Controls.h"
#include "Button_Pressing.h"
#include "Bump_Sensing.h"

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
#define BUMPER_R_PIN            13
#define BUMPER_R_PIN            12

// ---------  Output pins  ----------- //

// motor wheel - RIGHT
#define WHEEL_R_ENABLE_PIN      6 // pwm - dark green
#define WHEEL_R_DIRECTION_PIN   8 //digital - black

// motor wheel - LEFT
#define WHEEL_L_ENABLE_PIN      5 // pwm
#define WHEEL_L_DIRECTION_PIN   7 //digital
    


// coin drop pins
// #define COIN_DROP_PIN // digital

// platform raising. its a stepper motor
// #define PLATFORM_RAISE_PIN // pwm

// button pressing
#define BUTTON_PRESSER_PIN   9 // pwm




#define DEBUG_RED               A5
#define DEBUG_GREEN             A4
#define DEBUG_BLUE              A3

// Timers

#define MAIN_TIMER              0// We'll use this for most of the states
#define SERVO_TIMER             1 // dunno...

#define RIGHT_SIDE              0 // oriented facing the arena from the start zones
#define LEFT_SIDE               1

typedef enum{
    // all the state names
    STARTUP,

    // --  Test states  -- //
    SERVER_BEACON_SENSED,
    DEPO_BEACON_SENSED,
    // tape sensor states
    TAPE_R_SENSED,
    TAPE_L_SENSED,
    TAPE_BOTH_SENSED,
    //button presser test states
    EXTENDING_BUTTON_PRESSER,
    RETRACTING_BUTTON_PRESSER,
    //motor control test states
    MOVING_FORWARD,
    MOVING_BACKWARD,
    ROTATING_RIGHT,
    ROTATING_LEFT,
    // bumper test states
    BUMPED,
    STOP_STATE, // keyboard 's'
    NULL_STATE,

    // --- Actual states --- //

    FIRST_ROTATE_TO_FIND_SERVER,
    MOVE_TOWARDS_SERVER,




    // last typedef enum value is guaranteed to be > then the 
    // # of states there are. kudos to the guy who thought of this
    NUM_STATES 
} STATES;

// array of functions for each state
void (*state_functions[NUM_STATES])(); 

// the state the machine is in
unsigned char state_changed;
unsigned char entered_state;
unsigned char arena_side; // which side of the board we are on
static unsigned char current_state = STARTUP;

unsigned char current_coin_count;
unsigned char coins_taken_from_server;
unsigned char times_button_pressed_for_coin;


// Objects 
static Debug_Led *debug_red;
static Debug_Led *debug_green;
static Debug_Led *debug_blue;

static Tape_Sensor *tape_f;
static Tape_Sensor *tape_c;

static Bump_Sensor *bumper_r;


// -------------- Prototypes ------------------------------- //

void debug_all_off(void);
void start_timer(unsigned char, int);



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
    // clear all the relevant timers?
    debug_all_off();
    stop_moving();
    current_state = new_state;
    entered_state = true;
    state_changed = true;
}

void log_states(){
    Serial.println("IT WORKS");
}

// -------------- Event testing functions ----------- //
// various event testers that return true or false

// i'm considering using pointer functions soon, so i will just have 
// a call like: 
//respond_to(function pointer, new_state);

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
            case('s'):
                change_state_to(STOP_STATE);
                return true;
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

unsigned char respond_to_tape_on(Tape_Sensor *tape_s, unsigned char new_state){
    if (tape_s->is_on_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_tape_off(Tape_Sensor *tape_s, unsigned char new_state){
    if (tape_s->is_on_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_bumper_bumped(Bump_Sensor *bump_s, unsigned char new_state){
    if (bump_s->is_bumped()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_bumper_not_bumped(Bump_Sensor *bump_s, unsigned char new_state){
    if (!bump_s->is_bumped()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}
unsigned char respond_to_button_presser_finished(unsigned char new_state){
    if (button_presser_finished()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_timer(unsigned char timer_id, unsigned char new_state){
    if (TMRArd_IsTimerExpired(timer_id)){
        change_state_to(new_state);
        return true;
    } else{
        return false;
    }
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

    bumper_r = new Bump_Sensor(BUMPER_R_PIN);

    beacon_sensing_init(INTERRUPT_ID);
    motor_control_init(WHEEL_R_DIRECTION_PIN, WHEEL_R_ENABLE_PIN, WHEEL_L_DIRECTION_PIN, WHEEL_L_ENABLE_PIN);
    button_pressing_init(BUTTON_PRESSER_PIN);

    // ==== yet to implement ===== //
    // coin_control_init()

    Serial.println("end startup_fn");

    change_state_to(NULL_STATE);
}

// Test states

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
    if (respond_to_tape_off(tape_r, NULL_STATE)) return;
    if (respond_to_tape_on(tape_l, TAPE_BOTH_SENSED)) return;
}

void tape_l_sensed_fn(){
    if (entered_state){
        debug_blue->led_on();
        Serial.println("tape_l_sensed_fn");
    }
    if (respond_to_tape_off(tape_l, NULL_STATE)) return;
    if (respond_to_tape_on(tape_r, TAPE_BOTH_SENSED)) return;

}

void tape_both_sensed_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("tape_both_sensed_fn");
    }
    if (respond_to_tape_off(tape_l, TAPE_R_SENSED)) return;
    if (respond_to_tape_off(tape_r, TAPE_L_SENSED)) return;
}


void extending_button_presser_fn(){
    if (entered_state){
        Serial.println("extending_button_presser_fn");
        extend_button_presser();
        start_timer(SERVO_TIMER, BUTTON_PRESSER_DELAY);

        // set servo timer
    }

    // reenter the state and reincrement the servo
    if (respond_to_key(EXTENDING_BUTTON_PRESSER)) return;
    if (respond_to_timer(SERVO_TIMER, EXTENDING_BUTTON_PRESSER)) return;
    if (respond_to_button_presser_finished(RETRACTING_BUTTON_PRESSER)) return;
}


void retracting_button_presser_fn(){
    if (entered_state){
        Serial.println("retracting_button_presser_fn");
        //move servo back a bit
        retract_button_presser();
        start_timer(SERVO_TIMER, BUTTON_PRESSER_DELAY);
    }
    if (respond_to_key(RETRACTING_BUTTON_PRESSER)) return;
    if (respond_to_timer(SERVO_TIMER, RETRACTING_BUTTON_PRESSER)) return;
    if (respond_to_button_presser_finished(EXTENDING_BUTTON_PRESSER)) return;
}

void moving_forward_fn(){
    if (entered_state){
        Serial.println("moving_forward_fn");
        start_timer(MAIN_TIMER, 2000);
        debug_blue->led_on();
        move_forwards(5);
    }
    if (respond_to_key(ROTATING_LEFT)) return;
    if (respond_to_timer(MAIN_TIMER, MOVING_BACKWARD));
}

void moving_backward_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("moving_backward_fn");
        start_timer(MAIN_TIMER, 2000);
        move_backwards(5);

    }
    if (respond_to_key(ROTATING_LEFT)) return;
    if (respond_to_timer(MAIN_TIMER, MOVING_FORWARD));
}

void rotating_right_fn(){
    if (entered_state){
        debug_green->led_on();
        debug_blue->led_on();
        Serial.println("rotating_right_fn");
        start_timer(MAIN_TIMER, 2000);
        rotate_right(5);
    }
    if (respond_to_key(MOVING_FORWARD)) return;
    if (respond_to_timer(MAIN_TIMER, ROTATING_LEFT)) return;
}

void rotating_left_fn(){
    if (entered_state){
        debug_red->led_on();
        debug_blue->led_on();
        Serial.println("rotating_left_fn");
        start_timer(MAIN_TIMER, 2000);
        rotate_left(5);
    }
    if (respond_to_key(MOVING_FORWARD)) return;
    if (respond_to_timer(MAIN_TIMER, ROTATING_RIGHT)) return;
}

void bumped_fn(){
    if (entered_state){
        debug_red->led_on();
        Serial.println("Bumped");
    }
    if (respond_to_bumper_not_bumped(bumper_r, NULL_STATE)) return;
}

// Actual states

void first_rotate_to_find_server_fn(){
    if (entered_state){
        rotate_right(5);
        start_timer(MAIN_TIMER, 6000);
    }

    if (respond_to_server_found(MOVE_TOWARDS_SERVER)) return ;
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
}

void move_towards_server_fn(){
    if (entered_state){
        stop_moving();
        move_forwards(5);
        start_timer(MAIN_TIMER, 5000);
    }

    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
    if (respond_to_bump(bumper_r, NULL_STATE)) {
        arena_side = RIGHT_SIDE; // now we know what side of the arena we are on
        return;
    }
    if (respond_to_bump(bumper_l, NULL_STATE)) {
        arena_side = LEFT_SIDE; 
        return;
    }
    if ()
}


// -- Various debugging states -- //
// send things here to end the loop

void null_state_fn(){
    if (entered_state){
        Serial.println("null_state_fn");
    }
    // Serial.println(digitalRead(BUMPER_R_PIN));
    // change_state_to(MOVING_FORWARD);
    // change_state_to(EXTENDING_BUTTON_PRESSER);
    // if (respond_to_bumper_bumped(bumper_r, BUMPED)) return;
    // if (respond_to_key(NULL_STATE)) return;
    // if (respond_to_depository_found(DEPO_BEACON_SENSED)) return;
    // if (respond_to_server_found(SERVER_BEACON_SENSED)) return;
    // if (respond_to_tape_on(tape_r, TAPE_R_SENSED)) return;
    // if (respond_to_tape_on(tape_l, TAPE_L_SENSED)) return;
}

void stop_state_fn(){
    if (entered_state){
        Serial.println("stop_state_fn");
    }

    if (respond_to_key(NULL_STATE)) return;

}

// -------------- Utility Methods ------------ //

void debug_all_off(){
    debug_red->led_off();
    debug_green->led_off();
    debug_blue->led_off();
}

void start_timer(unsigned char timer_id, int duration){
    TMRArd_InitTimer(timer_id, duration);
}

// -------------- Setup Methods ------------ //
// this goes at the end because the functions need to be defined first
void setup_states() {
    //link each state to a function
    state_functions[STARTUP] = startup_fn;

    // --  Test states  -- //
    state_functions[SERVER_BEACON_SENSED] = server_beacon_sensed_fn;
    state_functions[DEPO_BEACON_SENSED] = depo_beacon_sensed_fn;
    state_functions[TAPE_R_SENSED] = tape_r_sensed_fn;
    state_functions[TAPE_L_SENSED] = tape_l_sensed_fn;
    state_functions[TAPE_BOTH_SENSED] = tape_both_sensed_fn;
    state_functions[EXTENDING_BUTTON_PRESSER] = extending_button_presser_fn;
    state_functions[RETRACTING_BUTTON_PRESSER] = retracting_button_presser_fn;
    state_functions[MOVING_FORWARD] = moving_forward_fn;
    state_functions[MOVING_BACKWARD] = moving_backward_fn;
    state_functions[ROTATING_LEFT] = rotating_left_fn;
    state_functions[ROTATING_RIGHT] = rotating_right_fn;
    state_functions[BUMPED] = bumped_fn;
    state_functions[NULL_STATE] = null_state_fn;
    state_functions[STOP_STATE] = stop_state_fn;

    // --- Actual states --- //
    state_functions[FIRST_ROTATE_TO_FIND_SERVER] = first_rotate_to_find_server_fn;
    state_functions[MOVE_TOWARDS_SERVER] = move_towards_server_fn;


}
