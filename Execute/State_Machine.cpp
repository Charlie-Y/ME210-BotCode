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
#define STATE_INIT_TIMER        2 // this is important
#define PULSE_TIMER             3 // actually defined in motor_controls.cpp
#define SECONDARY_TIMER         4 // global
#define GAME_TIMER         5 // global


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
    //moved to actual states. I'm a real state now
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
    PULSE_ARC_BACK,
    // bumper test states
    BUMPED_B,
    BUMPED_R,
    BUMPED_L,
    STOP_STATE, // keyboard 's'
    NULL_STATE,

    // --- Actual states --- //

    // Getting to the server
    FIRST_ROTATE_TO_FIND_SERVER,
    FIRST_MOVE_TOWARDS_SERVER,
    BACKUP_SLOWLY_TO_TAPE_CENTER,
    ARC_BACK_OFF_WALL,
    POST_ARC_SERVER_SEARCH,
    MOVE_TOWARDS_SERVER,
    CENTERED_ON_SERVER_TAPE,
    CORRECTING_ROTATION_1,
    CORRECTING_ROTATION_2,
    FINAL_ALIGN_TO_SERVER,
    MOVING_FW_TO_ALIGN_WITH_SERVER,
    BUMPED_ALIGNING_WITH_SERVER,
    SEARCH_LEFT_FOR_SERVER,
    SEARCH_RIGHT_FOR_SERVER,


    EXTENDING_BUTTON_PRESSER,
    RETRACTING_BUTTON_PRESSER,

    WAIT_FOR_LAST_COIN,
    STRAIGHT_BACK_TO_DEPO,

    FINDING_DEPO_TO_SKIP,
    MOVING_TO_DEPO,
    CORRECTING_DEPO_ALIGNMENT,

    SEARCH_RIGHT_A_BIT, 
    SEARCH_LEFT_A_BIT,
    FINAL_MOVE_TO_DEPO,
    GETTING_CLOSER_TO_DEPO, 

    BACKING_AWAY_FROM_DEPO,
    // last typedef enum value is guaranteed to be > then the 
    // # of states there are. kudos to the guy who thought of this
    NUM_STATES 
} STATES;

// array of functions for each state
void (*state_functions[NUM_STATES])(); 

// the state the machine is in
static unsigned char state_changed;
static unsigned char entered_state;
static unsigned char arena_side = LEFT_SIDE; // which side of the board we are on
static unsigned char state_init_finished; // a debounce statement
static unsigned char current_state = STARTUP;

static int desperation = 0;

static unsigned char direction_of_interest;
static unsigned char times_hopper_lifted = 0;
static unsigned char times_to_lift_hopper = 1;// for safety

static unsigned char coins_for_exchanges[] = {5, 5 + 8, 5 + 8 + 3};
static unsigned char coin_collection_round = 0;

// not being used...
// static unsigned char exchanges[] = {8, 5, 3, 0};
// static unsigned char coin_collection_round;
// static unsigned char current_server;
// static unsigned char next_server;
// static unsigned char coins_on_hopper = 0;
// static unsigned char current_coin_count = 0;
// static unsigned char times_button_pressed_required = 1;
// static unsigned char total_coins = 0;


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
unsigned char respond_to_timer(unsigned char, unsigned char);




// -------------- Generic State handling functions --------- //
// these are pretty much taken directly from the code. its a great architecture. 


void execute_current_state(){
    // use the current state to access the array index of the 
    // function that i want
    if (respond_to_timer(GAME_TIMER, STOP_STATE)) return; 

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

unsigned char respond_to_enough_presses(int num_coins, unsigned char new_state){
    if (pressed_enough_times_for_coins(num_coins)){
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

unsigned char respond_to_hopper_lifted_enough_times(unsigned char new_state){
    if (times_hopper_lifted == times_to_lift_hopper){
        times_hopper_lifted = 0;
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
    // Serial.println("startup_fn");

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

    // Serial.println("end startup_fn");

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
        start_timer(MAIN_TIMER, 10000);
        rotate_right(10);
        // pulse_rotate_right();
    }
    // if (respond_to_key(MOVING_FORWARD)) return;
    // if (respond_to_timer(ROTATING_RIGHT, NULL_STATE)) return;
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
        pulse_rotate_right();
        debug_red->led_on();
    }
    check_pulse();
    if (respond_to_server_found(PULSE_FORWARD)) return;
}

void pulse_arc_back_fn(){
    if (entered_state){
        Serial.println("Pulsing arc back");
        pulse_arc_back(DIR_RIGHT);
        debug_green->led_on();
    }
    check_pulse();

}
// Actual states



// * --- Actual States ------ 
// *
// *
// *
// *

void first_rotate_to_find_server_fn(){
    if (entered_state){
        Serial.println("first_rotate_to_find_server_fn");
        // rotate_right(8);
        pulse_rotate_right();
        debug_blue->led_on();
        start_timer(MAIN_TIMER, 6000);
    }
    check_pulse();

    if (respond_to_server_found(CORRECTING_ROTATION_1)) return ;
    if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
}


void first_move_towards_server_fn(){
    if (entered_state){
        Serial.println("first_move_towards_server_fn");
        // move_forwards(7);
        pulse_forward();
        debug_red->led_on();
        // start_timer(MAIN_TIMER, 5000);
    }

    check_pulse();

    // if (respond_to_timer(MAIN_TIMER, NULL_STATE)) return;
    if (respond_to_only_r_bumper_bumped(bumper_r, bumper_l, ARC_BACK_OFF_WALL)) {
        arena_side = RIGHT_SIDE; // now we know what side of the arena we are on
        direction_of_interest = DIR_LEFT; // you want to arc back and to the left
        return;
    }
    if (respond_to_only_l_bumper_bumped(bumper_l, bumper_r, ARC_BACK_OFF_WALL)) {
        arena_side = LEFT_SIDE; // now we know what side of the arena we are on
        direction_of_interest = DIR_RIGHT; // you want to arc back and right
        return;
    }
    if (respond_to_tape_on(tape_c, BACKUP_SLOWLY_TO_TAPE_CENTER)) return;

    // if (respond_to_only_one_tape_on(tape_c, tape_f, ROTATE_ON_SERVER_SENSOR)){
    //     return;
    // }
    if (respond_to_both_bumpers_bumped(bumper_r, bumper_l, NULL_STATE)){
        return;
    }
}

void move_towards_server_fn(){
    // just moving to the server. hum didddly dum...
    if (entered_state){
        debug_red->led_on();
        pulse_forward();
    }
    check_pulse();
    if (respond_to_bumper_bumped(bumper_r, ARC_BACK_OFF_WALL)) {
        direction_of_interest = DIR_LEFT;
        return;
    }
    if (respond_to_bumper_bumped(bumper_l, ARC_BACK_OFF_WALL)) {
        direction_of_interest = DIR_RIGHT;
        return;
    }
    if (respond_to_tape_on(tape_c, CENTERED_ON_SERVER_TAPE)) return;
}

void post_arc_server_search_fn(){
    if (entered_state){
        pulse_rotate_right();
        debug_blue->led_on();
    }
    check_pulse();
    if (respond_to_server_found(CORRECTING_ROTATION_2)) return;
}

void correcting_rotation_1_fn(){
    if (entered_state){
        start_state_init_timer(400);
        debug_green->led_on();
    }
    if (state_init_timer_finished()){
        rotate_left(8);
        state_init_finished = true;
        start_timer(MAIN_TIMER, 60);
        debug_green->led_off();
        debug_red->led_off();
    }
    if (state_init_finished){
        if (respond_to_timer(MAIN_TIMER, FIRST_MOVE_TOWARDS_SERVER)) return;
    }
}

// figure out how to make these into substates or something...
void correcting_rotation_2_fn(){
    if (entered_state){
        start_state_init_timer(400);
        debug_green->led_on();
    }
    if (state_init_timer_finished()){
        rotate_left(8);
        state_init_finished = true;
        start_timer(MAIN_TIMER, 60);
        debug_green->led_off();
        debug_red->led_off();
    }
    if (state_init_finished){
        // if (respond_to_timer(MAIN_TIMER, MOVING_FW_TO_ALIGN_WITH_SERVER)) return;
        if (respond_to_timer(MAIN_TIMER, MOVE_TOWARDS_SERVER)) return;
    }
}

void arc_back_off_wall_fn(){
    if (entered_state){
        pulse_arc_back(direction_of_interest);
        start_timer(MAIN_TIMER, 4000);
        debug_green->led_on();
    }
    check_pulse();
    if (respond_to_timer(MAIN_TIMER, POST_ARC_SERVER_SEARCH)) return;
    if (respond_to_tape_on(tape_c, CENTERED_ON_SERVER_TAPE)) return;
    // if (respond_to_server_found(MOVE_TOWARDS_SERVER)) return;

}

void backup_slowly_to_tape_center_fn(){
    if (entered_state){
        debug_red->led_on();
        start_state_init_timer(500); // wait a bit before moving backwards
    }
    if (state_init_timer_finished()){
        pulse_fine_backward();
    }
    if (state_init_finished){
        check_pulse();
    }
    if (respond_to_tape_on(tape_c, CENTERED_ON_SERVER_TAPE)) return;
}   


void centered_on_server_tape_fn(){
    if (entered_state){
        debug_red->led_on();
        debug_green->led_on();
        debug_blue->led_on();
        start_timer(MAIN_TIMER, 1000);
    }
    if (respond_to_timer(MAIN_TIMER, FINAL_ALIGN_TO_SERVER)) return;
    if (respond_to_any_bumper_bumped(EXTENDING_BUTTON_PRESSER)) return;

}

// back up a bit, rotate till it sees the beacon. rotate back to compensate
// move forward 
// if one bumper is hit, then pivot until the other is hit.

// start pressing the button
///

void final_align_to_server_fn(){
    if (entered_state){
        pulse_rotate_right();
        debug_blue->led_on();
    }
    check_pulse();
    if (respond_to_server_found(MOVING_FW_TO_ALIGN_WITH_SERVER)) return;
}

void search_right_for_server_fn(){
    if (respond_to_server_found(MOVING_FW_TO_ALIGN_WITH_SERVER)){
        direction_of_interest = DIR_RIGHT;
        return;
    }
    if (entered_state){
        pulse_fine_rotate_right();
        start_timer(MAIN_TIMER, 1500 + (desperation * 550));
        debug_blue->led_on();
        desperation++;
    }
    check_pulse();
    if (respond_to_timer(MAIN_TIMER, SEARCH_LEFT_FOR_SERVER)) return;
}

void search_left_for_server_fn(){
    if (respond_to_server_found(MOVING_FW_TO_ALIGN_WITH_SERVER)){
        direction_of_interest = DIR_LEFT;
        return;
    }
    if (entered_state){
        pulse_fine_rotate_left();
        start_timer(MAIN_TIMER, 1500 + (desperation * 550));
        debug_blue->led_on();
        debug_red->led_on();
        desperation++;
    }
    check_pulse();
    if (respond_to_timer(MAIN_TIMER, SEARCH_RIGHT_FOR_SERVER)) return;
}   

void moving_fw_to_align_with_server_fn(){
    if (entered_state){
        desperation = 0;
        pulse_fine_forward();
        debug_red->led_on();
        start_timer(MAIN_TIMER, 1000);
    }
    check_pulse();
    if (direction_of_interest == DIR_LEFT){
        if (respond_to_timer(MAIN_TIMER, SEARCH_RIGHT_FOR_SERVER)) {
            direction_of_interest == DIR_RIGHT;
            return;
        }
    } 
    if (direction_of_interest == DIR_RIGHT){
        if (respond_to_timer(MAIN_TIMER, SEARCH_LEFT_FOR_SERVER)) {
            direction_of_interest == DIR_LEFT;
            return;
        }
    } 
    if (respond_to_bumper_bumped(bumper_r, BUMPED_ALIGNING_WITH_SERVER)){
        direction_of_interest = DIR_LEFT;
        
        return;
    }
    if (respond_to_bumper_bumped(bumper_l, BUMPED_ALIGNING_WITH_SERVER)){
        direction_of_interest = DIR_RIGHT;
        return;
    }
    // if (respond_to_any_bumper_bumped(EXTENDING_BUTTON_PRESSER)) return;
}



//one bumper is down, now pivot until the other is down
void bumped_aligning_with_server_fn(){
    if (entered_state){
        if (direction_of_interest == DIR_LEFT){
            debug_blue->led_on();
            pivot_left(10);
        } else if (direction_of_interest == DIR_RIGHT){
            debug_green->led_on();
            pivot_right(10);
        }
        start_timer(MAIN_TIMER, 3000);
        // move_forwards(10);
    }
    if (respond_to_timer(MAIN_TIMER, MOVING_FW_TO_ALIGN_WITH_SERVER)) return;
    // if (respond_to_timer(MAIN_TIMER, EXTENDING_BUTTON_PRESSER)) return;
    if (respond_to_both_bumpers_bumped(bumper_l, bumper_r, EXTENDING_BUTTON_PRESSER)) return;
}

void extending_button_presser_fn(){
    if (entered_state){
        Serial.println("extending_button_presser_fn");
        extend_button_presser();
        debug_green->led_on();
        start_timer(SERVO_TIMER, BUTTON_PRESSER_DELAY);

        pulse_forward();
        // set servo timer
    }
    check_pulse();

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
    // if (respond_to_enough_presses(3,STRAIGHT_BACK_TO_DEPO)) return;
    unsigned char coins_to_get = coins_for_exchanges[coin_collection_round];
    if (respond_to_enough_presses(coins_to_get,WAIT_FOR_LAST_COIN)) return;
    if (respond_to_key(RETRACTING_BUTTON_PRESSER)) return;
    if (respond_to_timer(SERVO_TIMER, EXTENDING_BUTTON_PRESSER)) return;
    // if (respond_to_timer(SERVO_TIMER, NULL_STATE)) return;
    // if (respond_to_timer(SERVO_TIMER, RETRACTING_BUTTON_PRESSER)) return;
    // if (respond_to_button_presser_finished(EXTENDING_BUTTON_PRESSER)) return;
}

void straight_back_to_depo_fn(){
    if(entered_state){
        start_state_init_timer(1000); // wait for last coin
    } 
    if (state_init_timer_finished()){
        start_timer(MAIN_TIMER, 5000);
        move_backwards(10); // wheeeeee // pray for straight movement
        debug_blue->led_on();
    }
    if (state_init_finished){
        if (respond_to_timer(MAIN_TIMER, LIFTING_HOPPER)) return;
    }
}

void wait_for_last_coin_fn(){
    if (entered_state){
        start_timer(MAIN_TIMER, 1000);
    }
    if (respond_to_timer(MAIN_TIMER, FINDING_DEPO_TO_SKIP)) return;
}


void finding_depo_to_skip_fn(){
    if (entered_state){
        debug_red->led_on();
        move_backwards(9);
        start_state_init_timer(700); // back up a bit first
    }
    if (state_init_timer_finished()){
        debug_blue->led_on();
        debug_red->led_off();    
        if (arena_side == RIGHT_SIDE){
            // rotate_right(5);
            pulse_rotate_right();
            direction_of_interest = DIR_RIGHT;
            // pulse_fine_rotate_right();
        } else if (arena_side == LEFT_SIDE){
            // rotate_left(5);
            pulse_rotate_left();
            direction_of_interest = DIR_LEFT;

            // pulse_fine_rotate_left();
        } else {
            // rotate_right(5);
            pulse_rotate_right();
            direction_of_interest = DIR_RIGHT;
            

        }
    }
    if (state_init_finished){
        check_pulse();
        // if (respond_to_tape_on(tape_f, FINAL_MOVE_TO_DEPO)) return;
        // if (respond_to_tape_on(tape_f, MOVING_TO_DEPO)) return;
        if (respond_to_depository_found(FINAL_MOVE_TO_DEPO)) {
            start_timer(SECONDARY_TIMER, 10000);// uh oh. this is for giving up
            return;
        }
        // if (respond_to_depository_found(MOVING_TO_DEPO)) return;
        // if (respond_to_depository_found(SKIPPING_DEPO)) return;
    }
}

// // make it skip a depo here?
// void skipping_depo_fn(){
//     if (entered_state){
//         pulse_rotate_right();
//     }
//     check_pulse();
//     if (respond_to_no_beacon(FINDING_FINAL_BEACON)) return;
// }

// void finding_final_depo_fn(){
//     if (entered_state){
//         pulse_rotate_right();
//     }
//     check_pulse();
//     if (respond_to_depository_found(MOVING_TO_DEPO)) return;
// }

void moving_to_depo_fn(){
    if(entered_state){
        pulse_forward();
        debug_red->led_on();
        debug_blue->led_on();
    }
    check_pulse();
    // if (respond_to_any_bumper_bumped(CORRECTING_DEPO_ALIGNMENT)) return;
    // if (respond_to_tape_on(tape_f, CORRECTING_DEPO_ALIGNMENT)) return;
}

void correcting_depo_alignment_fn(){ // might not be necessary
    if (entered_state){
        debug_red->led_on();
        debug_green->led_on();
        debug_blue->led_on();
        pulse_backward();
        start_timer(MAIN_TIMER, 400);
    }
    check_pulse();
    if(respond_to_timer(MAIN_TIMER, SEARCH_RIGHT_A_BIT)) return;

}

void search_right_a_bit_fn(){
    if (entered_state){
        // rotate_right(10);
        pulse_rotate_right();
        // pulse_fine_rotate_right();
        start_timer(MAIN_TIMER, 2000 + (desperation * 2600));
        debug_blue->led_on();
        desperation++;
    }
    check_pulse();
    if (respond_to_timer(MAIN_TIMER, SEARCH_LEFT_A_BIT)) return;
    if (respond_to_depository_found(FINAL_MOVE_TO_DEPO)) {
        direction_of_interest = DIR_RIGHT;
        return;
    }
}

void search_left_a_bit_fn(){
    if (entered_state){
        // rotate_left(10);
        pulse_rotate_left();
        // pulse_fine_rotate_left();
        start_timer(MAIN_TIMER, 2000 + (desperation * 1200));
        debug_green->led_on();
        desperation++;
    }
    check_pulse();
    if (respond_to_timer(MAIN_TIMER, SEARCH_RIGHT_A_BIT)) return;
    if (respond_to_depository_found(FINAL_MOVE_TO_DEPO)){
        direction_of_interest = DIR_LEFT;
        return;
    }
}

void final_move_to_depo_fn(){
    if (entered_state){
        desperation = 0;
        // pulse_forward();
        move_forwards(9);
        debug_red->led_on();
        debug_blue->led_on();
        start_timer(MAIN_TIMER, 300);
    }
    // check_pulse();
    if (respond_to_timer(SECONDARY_TIMER, GETTING_CLOSER_TO_DEPO)){
        // set in finding_depo_to_skip
        return;
    }
    if (!depository_found()){
        if (direction_of_interest == DIR_LEFT){
            if (respond_to_timer(MAIN_TIMER, SEARCH_RIGHT_A_BIT)) {
                direction_of_interest == DIR_RIGHT;
                return;
            }
        } 
        if (direction_of_interest == DIR_RIGHT){
            if (respond_to_timer(MAIN_TIMER, SEARCH_LEFT_A_BIT)) {
                direction_of_interest == DIR_LEFT;
                return;
            }
        }
    } 
    // if (respond_to_tape_on(tape_f, GETTING_CLOSER_TO_DEPO))return;
    if (respond_to_any_bumper_bumped(GETTING_CLOSER_TO_DEPO)) return;
    // if (respond_to_any_bumper_bumped(LIFTING_HOPPER)) return;
}

void getting_closer_to_depo_fn(){
    if (entered_state){
        move_forwards(10);
        start_timer(MAIN_TIMER, 400);
        debug_green->led_on();
    }
    if (respond_to_timer(MAIN_TIMER, LIFTING_HOPPER)) {
        times_hopper_lifted = 0;
        return;
    }
}

void lifting_hopper_fn(){
    if (entered_state){
        // pulse_backward();
        pulse_forward();
        Serial.println("lifting_hopper_fn");
        debug_blue->led_on();
        start_timer(SERVO_TIMER, 1500);
        extend_dumper();
        times_hopper_lifted++;
    }
    check_pulse();
    // if (respond_to_key(NULL_STATE)) return; 
    // if (respond_to_timer_and_dumper_finished(SERVO_TIMER, LOWERING_HOPPER)) return;
    // if (respond_to_dumper_finished(LOWERING_HOPPER)) return; 
    if (respond_to_timer(SERVO_TIMER, LOWERING_HOPPER)) return; 
    // if (respond_to_timer(SERVO_TIMER, LIFTING_HOPPER)) return; 
}

void lowering_hopper_fn(){
if (entered_state){
        // Serial.println("lowering_hopper_fn");
        debug_green->led_on();
        start_timer(SERVO_TIMER, 1000);
        retract_dumper();
    }
    // if (respond_to_key(NULL_STATE)) return;
    // if (respond_to_dumper_finished(NULL_STATE)) return; 
    if (respond_to_hopper_lifted_enough_times(BACKING_AWAY_FROM_DEPO)){
        // at this point the coins should have fallen in
        coin_collection_round++;
        return;
    }
    if (respond_to_timer(SERVO_TIMER, LIFTING_HOPPER)) return; 
}

void backing_away_from_depo_fn(){
    if (entered_state){
        debug_red->led_on();
        start_timer(MAIN_TIMER, 600);
        move_backwards(10);
    }
    if (respond_to_timer(MAIN_TIMER, POST_ARC_SERVER_SEARCH)) return;
}


///


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
    // change_state_to(ROTATING_RIGHT);
    // change_state_to(MOVING_FORWARD);
    // change_state_to(PULSE_ARC_BACK);
    // change_state_to(PULSE_ROTATE_RIGHT);
    // change_state_to(PULSE_FORWARD);
    // change_state_to(EXTENDING_BUTTON_PRESSER);
    // change_state_to(LIFTING_HOPPER);
    change_state_to(FIRST_ROTATE_TO_FIND_SERVER);
    // change_state_to(MOVING_FW_TO_ALIGN_WITH_SERVER);
    // change_state_to(SEARCH_RIGHT_A_BIT);
    // change_state_to(FINDING_DEPO_TO_SKIP);
    // change_state_to(CENTERED_ON_SERVER_TAPE);
    // change_state_to(FINAL_ALIGN_TO_SERVER);
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
// i'm not going to write all the prototypes just so i can write this at the front...
void setup_states() {
    //link each state to a function
    state_functions[STARTUP] = startup_fn;

    // --  Test states  -- //
    state_functions[SERVER_BEACON_SENSED] = server_beacon_sensed_fn;
    state_functions[DEPO_BEACON_SENSED] = depo_beacon_sensed_fn;
    state_functions[TAPE_F_SENSED] = tape_f_sensed_fn;
    state_functions[TAPE_C_SENSED] = tape_c_sensed_fn;
    state_functions[TAPE_BOTH_SENSED] = tape_both_sensed_fn;
    
    
    

    state_functions[MOVING_FORWARD] = moving_forward_fn;
    state_functions[MOVING_BACKWARD] = moving_backward_fn;
    state_functions[ROTATING_LEFT] = rotating_left_fn;
    state_functions[ROTATING_RIGHT] = rotating_right_fn;
    state_functions[PULSE_FORWARD] = pulse_forward_fn;
    state_functions[PULSE_ROTATE_RIGHT] = pulse_rotate_right_fn;
    state_functions[PULSE_ARC_BACK] = pulse_arc_back_fn;

    state_functions[BUMPED_R] = bumped_r_fn;
    state_functions[BUMPED_L] = bumped_l_fn;
    state_functions[BUMPED_B] = bumped_b_fn;
    state_functions[NULL_STATE] = null_state_fn;
    state_functions[STOP_STATE] = stop_state_fn;

    // --- Actual states --- //
    state_functions[FIRST_ROTATE_TO_FIND_SERVER] = first_rotate_to_find_server_fn;
    state_functions[FIRST_MOVE_TOWARDS_SERVER] = first_move_towards_server_fn;
    state_functions[BACKUP_SLOWLY_TO_TAPE_CENTER] = backup_slowly_to_tape_center_fn;
    state_functions[ARC_BACK_OFF_WALL] = arc_back_off_wall_fn;
    state_functions[POST_ARC_SERVER_SEARCH] = post_arc_server_search_fn;
    state_functions[MOVE_TOWARDS_SERVER] = move_towards_server_fn;
    state_functions[CENTERED_ON_SERVER_TAPE] = centered_on_server_tape_fn;
    state_functions[CORRECTING_ROTATION_1] = correcting_rotation_1_fn;
    state_functions[CORRECTING_ROTATION_2] = correcting_rotation_2_fn;
    state_functions[FINAL_ALIGN_TO_SERVER] = final_align_to_server_fn;
    state_functions[SEARCH_LEFT_FOR_SERVER] = search_left_for_server_fn;
    state_functions[SEARCH_RIGHT_FOR_SERVER] = search_right_for_server_fn;
    state_functions[MOVING_FW_TO_ALIGN_WITH_SERVER] = moving_fw_to_align_with_server_fn;
    state_functions[BUMPED_ALIGNING_WITH_SERVER] = bumped_aligning_with_server_fn;
    
    state_functions[EXTENDING_BUTTON_PRESSER] = extending_button_presser_fn;
    state_functions[RETRACTING_BUTTON_PRESSER] = retracting_button_presser_fn;
    state_functions[STRAIGHT_BACK_TO_DEPO] = straight_back_to_depo_fn;

    state_functions[FINDING_DEPO_TO_SKIP] = finding_depo_to_skip_fn;
    state_functions[WAIT_FOR_LAST_COIN] = wait_for_last_coin_fn;
    state_functions[MOVING_TO_DEPO] = moving_to_depo_fn;
    state_functions[CORRECTING_DEPO_ALIGNMENT] = correcting_depo_alignment_fn;
    state_functions[SEARCH_LEFT_A_BIT] = search_left_a_bit_fn;
    state_functions[SEARCH_RIGHT_A_BIT] = search_right_a_bit_fn;
    state_functions[FINAL_MOVE_TO_DEPO] = final_move_to_depo_fn;
    state_functions[GETTING_CLOSER_TO_DEPO] = getting_closer_to_depo_fn;

    state_functions[LIFTING_HOPPER] = lifting_hopper_fn;
    state_functions[LOWERING_HOPPER] = lowering_hopper_fn;

    state_functions[BACKING_AWAY_FROM_DEPO] = backing_away_from_depo_fn;


    start_timer(GAME_TIMER, 1000 * 60 * 2);

}
