// State machine with all the test states intact.

#include "Arduino.h"
#include "Timers.h"

#include "State_Machine.h"
#include "Debug_Leds.h"
#include "Tape_Sensing.h"
#include "Beacon_Sensing.h"
#include "Motor_Controls.h"
#include "Button_Pressing.h"
#include "Bump_Sensing.h"
#include "Dumping.h"

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
#define TAPE_SENSOR_F_PIN       A0
#define TAPE_SENSOR_C_PIN       A1

// more digital reads
#define BUMPER_R_PIN            A5
#define BUMPER_L_PIN            A4

// ---------  Output pins  ----------- //

// motor wheel - RIGHT
#define WHEEL_R_ENABLE_PIN      6 // pwm - 
#define WHEEL_R_DIRECTION_PIN   7 //digital - light green

// motor wheel - LEFT
#define WHEEL_L_ENABLE_PIN      5 // pwm - dark green
#define WHEEL_L_DIRECTION_PIN   4 //digital - black
    


// coin drop pins
// #define COIN_DROP_PIN // digital

// platform raising. its a stepper motor
// #define PLATFORM_RAISE_PIN // pwm

// button pressing
#define BUTTON_PRESSER_PIN   10 // pwm
// dumping
#define DUMPING_PIN 9 //pwm


#define DEBUG_RED               11
#define DEBUG_GREEN             12
#define DEBUG_BLUE              A3

// Timers

#define MAIN_TIMER              0// We'll use this for most of the states
#define SERVO_TIMER             1 // dunno...
#define STATE_INIT_TIMER             2 // this is important
#define PULSE_TIMER             3 // actually defined in motor_controls.cpp

#define RIGHT_SIDE              0 // oriented facing the arena from the start zones
#define LEFT_SIDE               1

typedef enum{
    // all the state names
    STARTUP,

    // --  Test states  -- //
    SERVER_BEACON_SENSED,
    DEPO_BEACON_SENSED,
    // tape sensor states
    TAPE_F_SENSED,
    TAPE_C_SENSED,
    TAPE_BOTH_SENSED,
    //button presser test states
    EXTENDING_BUTTON_PRESSER,
    RETRACTING_BUTTON_PRESSER,
    // hopper dumping test states
    LIFTING_HOPPER,
    LOWERING_HOPPER,

    //motor control test states
    MOVING_FORWARD,
    MOVING_BACKWARD,
    ROTATING_RIGHT,
    ROTATING_LEFT,
    PULSE_FORWARD,
    PULSE_ROTATE_RIGHT,
    // bumper test states
    BUMPED_B,
    BUMPED_R,
    BUMPED_L,
    STOP_STATE, // keyboard 's'
    NULL_STATE,

    // --- Actual states --- //

    FIRST_ROTATE_TO_FIND_SERVER,
    PAUSE,
    MOVE_TOWARDS_SERVER,
    ROTATE_RIGHT_OFF_WALL,
    ROTATE_LEFT_OFF_WALL,
    ROTATE_ON_SERVER_SENSOR,
    GET_COINS,
    ACCOUNT_FOR_COINS,
    SETUP_TO_CHOOSE_DEPOSITORY,
    CHOOSE_DEPOSITORY,
    MOVE_TO_DEPOSITORY,
    CHANGE_DEPOSITORY_IN_MOTION,
    SETUP_TO_DUMP,
    DUMP,
    RETRACT_DUMPER,
    
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
unsigned char state_init_finished; // a debounce statement
static unsigned char current_state = STARTUP;

unsigned char exchanges[] = {8, 5, 3, 0};
unsigned char coin_collection_round;
unsigned char current_server;
unsigned char next_server;
unsigned char coins_on_hopper = 0;
unsigned char current_coin_count = 0;
unsigned char times_button_pressed_required = 1;
unsigned char total_coins = 0;


// Objects 
static Debug_Led *debug_red;
static Debug_Led *debug_green;
static Debug_Led *debug_blue;

static Tape_Sensor *tape_f;
static Tape_Sensor *tape_c;

static Bump_Sensor *bumper_r;
static Bump_Sensor *bumper_l;


// -------------- Prototypes ------------------------------- //

void debug_all_off(void);
void start_timer(unsigned char, int);
void start_state_init_timer(int);
unsigned char state_init_timer_finished();



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
    beacon_sensing_state_changed();
    motor_state_changed(); // resets the pulse clock?

    current_state = new_state;
    entered_state = true;
    state_changed = true;
    state_init_finished = false;

    Serial.println(" "); // take this off later

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
    if (tape_s->is_off_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_only_one_tape_on(Tape_Sensor *tape_s1, Tape_Sensor *tape_s2, unsigned char new_state){
    if (tape_s1->is_on_tape() && tape_s2->is_off_tape()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_both_tape_on(Tape_Sensor *tape_s1, Tape_Sensor *tape_s2, unsigned char new_state){
    if (tape_s1->is_on_tape() && tape_s2->is_on_tape()){
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

unsigned char respond_to_any_bumper_bumped(unsigned char new_state){
    if (bumper_r->is_bumped() || bumper_l->is_bumped()){
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

unsigned char respond_to_only_one_bumper_bumped(Bump_Sensor *bump_y, Bump_Sensor *bump_n, unsigned char new_state){
    if (bump_y->is_bumped() && bump_n->is_not_bumped()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_only_r_bumper_bumped(Bump_Sensor *bump_r, Bump_Sensor *bump_l, unsigned char new_state){
    if (bump_r->is_bumped() && bump_l->is_not_bumped()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_only_l_bumper_bumped(Bump_Sensor *bump_l, Bump_Sensor *bump_r, unsigned char new_state){
    if (bump_l->is_bumped() && bump_r->is_not_bumped()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}

unsigned char respond_to_both_bumpers_bumped(Bump_Sensor *bump_1, Bump_Sensor *bump_2, unsigned char new_state){
    if (bump_1->is_bumped() && bump_2->is_bumped()){
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

unsigned char respond_to_timer_and_dumper_finished(unsigned char timer_id, unsigned char new_state){
    if (TMRArd_IsTimerExpired(timer_id) && dumper_finished()){
        change_state_to(new_state);
        return true;
    } else {
        return false;
    }
}



unsigned char respond_to_dumper_finished(unsigned char new_state){
    if (dumper_finished()){
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

    tape_f = new Tape_Sensor(TAPE_SENSOR_F_PIN);
    tape_c = new Tape_Sensor(TAPE_SENSOR_C_PIN);

    bumper_r = new Bump_Sensor(BUMPER_R_PIN);
    bumper_l = new Bump_Sensor(BUMPER_L_PIN);

    beacon_sensing_init(INTERRUPT_ID);
    motor_control_init(WHEEL_R_DIRECTION_PIN, WHEEL_R_ENABLE_PIN, WHEEL_L_DIRECTION_PIN, WHEEL_L_ENABLE_PIN);
    button_pressing_init(BUTTON_PRESSER_PIN);
    dumping_init(DUMPING_PIN);

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
    if (respond_to_server_found(SERVER_BEACON_SENSED)) return;
    if (respond_to_no_beacon(NULL_STATE)) return;

}

void tape_f_sensed_fn(){
    if (entered_state){
        debug_red->led_on();
        Serial.println("tape_f_sensed_fn");
    }
    if (respond_to_tape_off(tape_f, NULL_STATE)) return;
    if (respond_to_tape_on(tape_c, TAPE_BOTH_SENSED)) return;
}

void tape_c_sensed_fn(){
    if (entered_state){
        debug_blue->led_on();
        Serial.println("tape_c_sensed_fn");
    }
    if (respond_to_tape_off(tape_c, NULL_STATE)) return;
    if (respond_to_tape_on(tape_f, TAPE_BOTH_SENSED)) return;
}


void tape_both_sensed_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("tape_both_sensed_fn");
    }
    if (respond_to_tape_off(tape_f, TAPE_C_SENSED)) return;
    if (respond_to_tape_off(tape_c, TAPE_F_SENSED)) return;
}


void extending_button_presser_fn(){
    if (entered_state){
        Serial.println("extending_button_presser_fn");
        extend_button_presser();
        debug_green->led_on();
        start_timer(SERVO_TIMER, BUTTON_PRESSER_DELAY);

        // set servo timer
    }

    // reenter the state and reincrement the servo
    if (respond_to_key(EXTENDING_BUTTON_PRESSER)) return;
    if (respond_to_timer(SERVO_TIMER, RETRACTING_BUTTON_PRESSER)) return; // uhh...
    // if (respond_to_timer(SERVO_TIMER, EXTENDING_BUTTON_PRESSER)) return; 
    // if (respond_to_button_presser_finished(RETRACTING_BUTTON_PRESSER)) return;
}


void retracting_button_presser_fn(){
    if (entered_state){
        Serial.println("retracting_button_presser_fn");
        //move servo back a bit
        debug_blue->led_on();
        retract_button_presser();
        start_timer(SERVO_TIMER, BUTTON_PRESSER_DELAY);
    }
    if (respond_to_key(RETRACTING_BUTTON_PRESSER)) return;
    if (respond_to_timer(SERVO_TIMER, NULL_STATE)) return;
    // if (respond_to_timer(SERVO_TIMER, RETRACTING_BUTTON_PRESSER)) return;
    // if (respond_to_button_presser_finished(EXTENDING_BUTTON_PRESSER)) return;
}

void lifting_hopper_fn(){
    if (entered_state){
        Serial.println("lifting_hopper_fn");
        debug_blue->led_on();
        start_timer(SERVO_TIMER, DUMPING_DELAY);
        extend_dumper();
    }
    if (respond_to_key(NULL_STATE)) return;
    if (respond_to_timer_and_dumper_finished(SERVO_TIMER, LOWERING_HOPPER)) return;
    // if (respond_to_dumper_finished(LOWERING_HOPPER)) return; 
    // if (respond_to_timer(SERVO_TIMER, LIFTING_HOPPER)) return; 
}

void lowering_hopper_fn(){
if (entered_state){
        Serial.println("lowering_hopper_fn");
        debug_green->led_on();
        start_timer(SERVO_TIMER, DUMPING_DELAY);
        retract_dumper();
    }
    if (respond_to_key(NULL_STATE)) return;
    // if (respond_to_dumper_finished(NULL_STATE)) return; 
    if (respond_to_timer(SERVO_TIMER, NULL_STATE)) return; 
}

void moving_forward_fn(){
    if (entered_state){
        Serial.println("moving_forward_fn");
        start_timer(MAIN_TIMER, 1000);
        debug_blue->led_on();
        move_forwards(10);
        // stop_moving();
        // pivot_left(10);
        // pivot_right(10);
    }
    // if (respond_to_key(ROTATING_LEFT)) return;
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
    // if (respond_to_any_bumper_bumped(NULL_STATE)) return;
}

void moving_backward_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("moving_backward_fn");
        start_timer(MAIN_TIMER, 2000);
        move_backwards(2);
        // move_forwards(5);

    }
    if (respond_to_key(ROTATING_LEFT)) return;
    if (respond_to_timer(MAIN_TIMER, NULL_STATE));
}

void rotating_right_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("rotating_right_fn");
        start_timer(MAIN_TIMER, 2000);
        rotate_right(10);
    }
    if (respond_to_key(MOVING_FORWARD)) return;
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
    // if (respond_to_any_bumper_bumped(NULL_STATE)) return;

    // if (respond_to_timer(MAIN_TIMER, ROTATING_LEFT)) return;
}

void rotating_left_fn(){
    if (entered_state){
        debug_red->led_on();
        debug_blue->led_on();
        Serial.println("rotating_left_fn");
        start_timer(MAIN_TIMER, 2000);
        rotate_left(10);
    }
    if (respond_to_key(MOVING_FORWARD)) return;
    if (respond_to_timer(MAIN_TIMER, ROTATING_RIGHT)) return;
}

void bumped_r_fn(){
    if (entered_state){
        debug_red->led_on();
        Serial.println("bumped_r_fn");
    }
    if (respond_to_bumper_not_bumped(bumper_r, NULL_STATE)) return;
    if (respond_to_bumper_bumped(bumper_l, BUMPED_B)) return;

}
void bumped_l_fn(){
    if (entered_state){
        debug_blue->led_on();
        Serial.println("bumped_l_fn");
    }
    if (respond_to_bumper_not_bumped(bumper_l, NULL_STATE)) return;
    if (respond_to_bumper_bumped(bumper_r, BUMPED_B)) return;

}

void bumped_b_fn(){
    if (entered_state){
        debug_green->led_on();
        Serial.println("bumped_b_fn");
    }
    if (respond_to_bumper_not_bumped(bumper_r, BUMPED_L)) return;
    if (respond_to_bumper_not_bumped(bumper_l, BUMPED_R)) return;
}

// void pulse_forward_fn(){
//     if (entered_state){
//         move_forwards(10);
//         start_state_init_timer(MOTOR_PULSE_LENGTH);
//         debug_blue->led_on();
//         // Serial.println("Pulsing fw");
//     }
//     if (state_init_timer_finished()){
//         stop_moving();
//         debug_blue->led_off();
//         start_timer(MAIN_TIMER, MOTOR_PULSE_LENGTH);
//         // Serial.println("Pulsing stopped");
//     }
//     if (state_init_finished){
//         if(respond_to_timer(MAIN_TIMER, PULSE_FORWARD)) return;
//     }
//     if (respond_to_key(NULL_STATE)) return ;

// }

void pulse_forward_fn(){
    if (entered_state){
        pulse_forward();
        // Serial.println("Pulsing fw");
        debug_blue->led_on();
    }
    check_pulse();
}

void pulse_rotate_right_fn(){
    if (entered_state){
        rotate_right(10);
        start_state_init_timer(MOTOR_PULSE_LENGTH);
        debug_red->led_on();
    } 
    if (state_init_timer_finished()){
        stop_moving();
        debug_red->led_off();
        start_timer(MAIN_TIMER, MOTOR_PULSE_LENGTH);
    }
    if (state_init_finished){
        if (respond_to_timer(MAIN_TIMER, PULSE_ROTATE_RIGHT));
    }
    if (respond_to_key(NULL_STATE)) return;
    if (respond_to_server_found(PULSE_FORWARD)) return;
}
// Actual states

void first_rotate_to_find_server_fn(){
    if (entered_state){
        Serial.println("first_rotate_to_find_server_fn");
        rotate_right(8);
        debug_blue->led_on();
        start_timer(MAIN_TIMER, 6000);
    }

    if (respond_to_server_found(PAUSE)) return ;
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
}

void pause_fn(){
    // things that we do when entering state
    if (entered_state){
        stop_moving();
        start_state_init_timer(1000);
        debug_green->led_on();
    }
    if (state_init_timer_finished()){
        rotate_left(5);
        debug_blue->led_on();
        debug_green->led_off();
        state_init_finished = true;
    }
    if (state_init_finished){
        if (respond_to_server_found(MOVE_TOWARDS_SERVER)) return ;
    }
}

void move_towards_server_fn(){
    if (entered_state){
        stop_moving();
        Serial.println("move_towards_server_fn");
        move_forwards(7);
        debug_red->led_on();
        start_timer(MAIN_TIMER, 5000);
    }
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
    if (respond_to_only_r_bumper_bumped(bumper_r, bumper_l, ROTATE_LEFT_OFF_WALL)) {
         arena_side = RIGHT_SIDE; // now we know what side of the arena we are on
         return;
    }
    if (respond_to_only_l_bumper_bumped(bumper_l, bumper_r, ROTATE_RIGHT_OFF_WALL)) {
         arena_side = LEFT_SIDE; // now we know what side of the arena we are on
         return;
    }
    if (respond_to_only_one_tape_on(tape_c, tape_f, ROTATE_ON_SERVER_SENSOR)){
        return;
    }
    if (respond_to_both_bumpers_bumped(bumper_r, bumper_l, NULL_STATE)){
        return;
    }
}


void rotate_left_off_wall_fn(){
    if (entered_state){
        stop_moving();
        debug_green->led_on();
        start_timer(MAIN_TIMER, 200);
        rotate_left(5);
    }
    
    if (respond_to_timer(MAIN_TIMER, MOVE_TOWARDS_SERVER)) return;
}

void rotate_right_off_wall_fn(){
    if (entered_state){
        stop_moving();
        debug_green->led_on();
        start_timer(MAIN_TIMER, 200);
        rotate_right(5);
    }
    
    if (respond_to_timer(MAIN_TIMER, MOVE_TOWARDS_SERVER)) return;
}

void rotate_on_server_sensor_fn(){
    if (entered_state){
        debug_red->led_on();
        stop_moving();
        // if (arena_side == LEFT_SIDE) rotate_left(5);
        // if (arena_side == RIGHT_SIDE) rotate_right(5);
        // probably back up here?
    }
    if (respond_to_tape_on(tape_f,MOVE_TOWARDS_SERVER)) return;
}
                
void get_coins_fn(){ // TO BE CHANGED
    if (entered_state){
        stop_moving();
        if (current_coin_count == 0) {
            current_server = exchanges[coin_collection_round];
            next_server =exchanges[coin_collection_round+1];
        }
        start_timer(MAIN_TIMER,BUTTON_PRESSER_DELAY);
        extend_button_presser();
    }
    
    if (respond_to_timer(MAIN_TIMER, ACCOUNT_FOR_COINS)){
        times_button_pressed_required -= 1;
        if (times_button_pressed_required == 0) {
            current_coin_count += 1;
            coins_on_hopper +=1;
            total_coins += 1;
        }
        return;
    }
    
}

void account_for_coins_fn(){ // TO BE CHANGED
    if (entered_state){
        start_timer(MAIN_TIMER,BUTTON_PRESSER_DELAY);
        retract_button_presser();
        if (times_button_pressed_required == 0) times_button_pressed_required = total_coins + 1;
    }
    
    if (current_coin_count == current_server) {
        current_coin_count = 0;
        coin_collection_round += 1;
        current_state = SETUP_TO_CHOOSE_DEPOSITORY;
        return;
        
    }
    if (respond_to_timer(MAIN_TIMER, GET_COINS)) return;
}

void setup_to_choose_depository_fn(){
    if (entered_state) {
        stop_moving();
        start_timer(MAIN_TIMER, 1000);
        move_backwards(5);
    }
    
    if (respond_to_timer(MAIN_TIMER,CHOOSE_DEPOSITORY)) return;
}

                
void choose_depository_fn(){
    if (entered_state) {
        stop_moving();
        switch (current_server){
            case 3:
                if (arena_side == LEFT_SIDE) rotate_right(5);
                if (arena_side == RIGHT_SIDE) rotate_left(5);
                break;
            case 5:
                if (arena_side == RIGHT_SIDE) rotate_right(5);
                if (arena_side == LEFT_SIDE) rotate_left(5);
                break;
            case 8:
                rotate_left(5);
                break;
        }
    }
    
    switch (current_server){
        case 3:
            if (respond_to_depository_found(MOVE_TO_DEPOSITORY)) return;
            break;
        case 5:
            if (respond_to_depository_found(MOVE_TO_DEPOSITORY)) return;
            break;
        case 8:
            if (respond_to_both_tape_on(tape_f, tape_c, MOVE_TO_DEPOSITORY)) return;
            break;
    }
}
                
void move_to_depository_fn(){
    if (entered_state) {
        stop_moving();
        move_forwards(5);
        start_timer (MAIN_TIMER, 5000);
    }
    
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
    if (respond_to_no_beacon(CHANGE_DEPOSITORY_IN_MOTION)) return;
    if (respond_to_bumper_bumped(bumper_r,SETUP_TO_DUMP) || respond_to_bumper_bumped(bumper_l,SETUP_TO_DUMP)) return;
}

void change_depository_in_motion_fn(){ 
    if (entered_state) {
        stop_moving;
    }
    
    if (coins_on_hopper < next_server) {
        current_state = FIRST_ROTATE_TO_FIND_SERVER;
        return;
    }
    
    if (coins_on_hopper > next_server) { //CAN ONLY DO ONCE FINALIZED STRATEGY
        
        return;
    }
  
}

void setup_to_dump_fn(){
    if (entered_state) {
        stop_moving();
        switch (current_server){
            case 3:
                if (arena_side == LEFT_SIDE) rotate_left(5);
                if (arena_side == RIGHT_SIDE) rotate_right(5);
                break;
            case 5:
                if (arena_side == RIGHT_SIDE) rotate_left(5);
                if (arena_side == LEFT_SIDE) rotate_right(5);
                break;
            case 8:
                break;
        }
    }
    
    if (respond_to_no_beacon(CHANGE_DEPOSITORY_IN_MOTION)) return; //IS THIS NECESSARY
    
    if (respond_to_both_bumpers_bumped(bumper_r, bumper_l,DUMP)) return;
}
    
void dump_fn(){
    if (entered_state) {
        stop_moving();
        start_timer(MAIN_TIMER, DUMPING_DELAY);
        extend_dumper();
        coins_on_hopper -= current_server;
    }
    
    if (respond_to_no_beacon(CHANGE_DEPOSITORY_IN_MOTION)) return; // IS THIS NECESARY?
    
    if(respond_to_timer(MAIN_TIMER, RETRACT_DUMPER)) return;
}

void retract_dumper_fn(){
    if (entered_state) {
        stop_moving();
        start_timer(MAIN_TIMER, DUMPING_DELAY);
        retract_dumper();
        move_backwards(5);
    }
    if (respond_to_no_beacon(CHANGE_DEPOSITORY_IN_MOTION)) return; // IS THIS NECESARY?
        
    if (respond_to_timer(MAIN_TIMER, FIRST_ROTATE_TO_FIND_SERVER))return;
}

// -- Various debugging states -- //
// send things here to end the loop

void null_state_fn(){
    if (entered_state){
        Serial.println("null_state_fn");
        debug_red->led_on();
        debug_green->led_on();
        debug_blue->led_on();
        move_forwards(8);
    }

    // Serial.println(digitalRead(BUMPER_R_PIN));
    // change_state_to(MOVING_FORWARD);
    // change_state_to(PULSE_ROTATE_RIGHT);
    change_state_to(PULSE_FORWARD);
    // change_state_to(EXTENDING_BUTTON_PRESSER);
    // change_state_to(LIFTING_HOPPER);
    // change_state_to(FIRST_ROTATE_TO_FIND_SERVER);
    // if (respond_to_key(LIFTING_HOPPER)) return;
    // if (respond_to_key(FIRST_ROTATE_TO_FIND_SERVER)) return;
    // if (respond_to_any_bumper_bumped( MOVING_FORWARD)) return;
    // if (respond_to_bumper_bumped(bumper_r, MOVING_FORWARD)) return;
    // if (respond_to_bumper_bumped(bumper_l, ROTATING_RIGHT)) return;
    // if (respond_to_bumper_bumped(bumper_r, FIRST_ROTATE_TO_FIND_SERVER)) return;
    // if (respond_to_bumper_bumped(bumper_l, EXTENDING_BUTTON_PRESSER)) return;
    // if (respond_to_both_bumpers_bumped( bumper_l, bumper_r, MOVE_FORWARD)) return;
    // if (respond_to_key(NULL_STATE)) return;
    // if (respond_to_depository_found(DEPO_BEACON_SENSED)) return;
    // if (respond_to_server_found(SERVER_BEACON_SENSED)) return;
    // if (respond_to_tape_on(tape_f, TAPE_F_SENSED)) return;
    // if (respond_to_tape_on(tape_c, TAPE_C_SENSED)) return;
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


// just for the sake of clarity
void start_state_init_timer(int duration){
    start_timer(STATE_INIT_TIMER, duration);
}

unsigned char state_init_timer_finished(){
    if (state_init_finished) return false; // only call once
    unsigned char result =  TMRArd_IsTimerExpired(STATE_INIT_TIMER);
    if (result){
        state_init_finished = true;
        return true;
    } else {
        return false;
    }

}

// -------------- Setup Methods ------------ //
// this goes at the end because the functions need to be defined first
void setup_states() {
    //link each state to a function
    state_functions[STARTUP] = startup_fn;

    // --  Test states  -- //
    state_functions[SERVER_BEACON_SENSED] = server_beacon_sensed_fn;
    state_functions[DEPO_BEACON_SENSED] = depo_beacon_sensed_fn;
    state_functions[TAPE_F_SENSED] = tape_f_sensed_fn;
    state_functions[TAPE_C_SENSED] = tape_c_sensed_fn;
    state_functions[TAPE_BOTH_SENSED] = tape_both_sensed_fn;
    state_functions[EXTENDING_BUTTON_PRESSER] = extending_button_presser_fn;
    state_functions[RETRACTING_BUTTON_PRESSER] = retracting_button_presser_fn;
    state_functions[LIFTING_HOPPER] = lifting_hopper_fn;
    state_functions[LOWERING_HOPPER] = lowering_hopper_fn;
    

    state_functions[MOVING_FORWARD] = moving_forward_fn;
    state_functions[MOVING_BACKWARD] = moving_backward_fn;
    state_functions[ROTATING_LEFT] = rotating_left_fn;
    state_functions[ROTATING_RIGHT] = rotating_right_fn;
    state_functions[PULSE_FORWARD] = pulse_forward_fn;
    state_functions[PULSE_ROTATE_RIGHT] = pulse_rotate_right_fn;

    state_functions[BUMPED_R] = bumped_r_fn;
    state_functions[BUMPED_L] = bumped_l_fn;
    state_functions[BUMPED_B] = bumped_b_fn;
    state_functions[NULL_STATE] = null_state_fn;
    state_functions[STOP_STATE] = stop_state_fn;

    // --- Actual states --- //
    state_functions[FIRST_ROTATE_TO_FIND_SERVER] = first_rotate_to_find_server_fn;
    state_functions[PAUSE] = pause_fn;
    state_functions[MOVE_TOWARDS_SERVER] = move_towards_server_fn;
    state_functions [ROTATE_RIGHT_OFF_WALL] = rotate_right_off_wall_fn;
    state_functions [ROTATE_LEFT_OFF_WALL] = rotate_left_off_wall_fn;
    state_functions [ROTATE_ON_SERVER_SENSOR] = rotate_on_server_sensor_fn;
    state_functions [GET_COINS] = get_coins_fn;
    state_functions [ACCOUNT_FOR_COINS] = account_for_coins_fn;
    state_functions [SETUP_TO_CHOOSE_DEPOSITORY] = setup_to_choose_depository_fn;
    state_functions [CHOOSE_DEPOSITORY] = choose_depository_fn;
    state_functions [MOVE_TO_DEPOSITORY] = move_to_depository_fn;
    state_functions [CHANGE_DEPOSITORY_IN_MOTION] = change_depository_in_motion_fn;
    state_functions [SETUP_TO_DUMP] = setup_to_dump_fn;
    state_functions [DUMP] = dump_fn;
    state_functions [RETRACT_DUMPER] = retract_dumper_fn;


}
