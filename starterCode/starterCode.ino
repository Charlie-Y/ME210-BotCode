
/*  Startercode for the rinal bot - names for the bot are still up for consideration 


/* ------ Goals and functionality ------- */

/*

The goal of this code is to make it amazingly easy and quick to change between strategies
of getting and depositing coins. 

This assumes for that all the input pins will be input high or low,
because we are trying to offload the signal processing to the hardware

*/




/* ------ Imports ------- */

#import "timer.h" // seems useful
//  probably need vectors



/* ======================================== */
/* ============== Defines ================= */
/* ======================================== */

#define DEBUGGING               1// true or false. all serial prints should have a if(DEBUGGING) Serial.println(etc)
// and we can have different types of debugging, like state tracking
#define LOG_STATE               1


// ---------  Input pins  ----------- //

#define IR_SENSOR_PIN //infrared sensor

#define TAPE_SENSOR_R_PIN
#define TAPE_SENSOR_L_PIN

#define BUMPER_R_PIN
#define BUMPER_R_PIN

// ---------  Output pins  ----------- //

// motor wheel - RIGHT
#define WHEEL_R_DIRECTION_PIN
#define WHEEL_R_SPEED_PIN
// motor wheel - LEFT
#define WHEEL_L_DIRECTION_PIN
#define WHEEL_L_SPEED_PIN
    
// coin drop pins
#define COIN_DROP_PIN

// platform raising. its a stepper motor
#define PLATFORM_RAISE_PIN 

// debug led pin - probably use the one on the Arduino
#define DEBUG_LED_PIN


// ---------  Directions  ----------- //

#define FORWARD                 0
#define REVERSE                 1
#define CLOCKWISE               2// right 
#define C_CLOCKWISE             3//left

// ---------  Beacons  ----------- //

#define MTGOX                   0
#define KRAKEN                  1
#define BITINSTANT              2

#define 3_COIN_BEACON           0 
#define 5_COIN_BEACON           1
#define 8_COIN_BEACON           2

// ---------  Strategies  ----------- //
// ignore most of these for now
    
#define TESTING // just testing stuff. 
#define BEAT_THE_BRICK // according to the sheet we don't have a specific beat the brick strategy
#define ALL_IN_ONE // get 16, fill all in one go
#define BOTTOM_UP // 3,5,8 - fill and return 
#define TOP_DOWN // 8,5,3 - awful
#define TWO_FOR_ONE // get 8, get 3 and 5


// ---------  Mechanical variations  ----------- //
// other variations to account for. might not care. 

#define with_encoder // we have 'absolute' position information
#define dump_truck // drops all coins in inventory at once
#define one_by_one // drops coins one at time
#define ignore_vinyl // if we have another way of telling which beacon is which

// ---------  Timer Ids and durations  ----------- //

#define REVERSE_FROM_BUTTON_PRESS_ID
#define REVERSE_FROM_BUTTON_PRESS_DURATION

// ---------  States  ----------- //

// determining side
rotate until server found. 
rotate until next beacon found while tracking time
rotate 1/2 time back other direction
move forwards and wait for bump - and now we know what side we''re on


// server locating and moving towards
rotate while looking for server signal
moving toward server
stopped against server

// coin retrieval
moving forward to press button
waiting for coin to drop
moving backward to press again

// beacon locating and identifiying and moving towards
rotate while looking for beacon signal
moving towards beacon with no vinyl
moving towards beacon with vinyl
stopped against depository

signal disappeared // other team got there first 


// coin dropping
dropping coins
finished dropping coins
waiting for coin to drop
waiting for golden samolean // checks for missed coins etc. 


/* =========================================== */
/* ============ Class constants ============== */
/* =========================================== */

// order in which to visit beacons
// should change 
BEACON_PRIORITIES = [3_COIN_BEACON, 5_COIN_BEACON, 8_COIN_BEACON]

// # coins to get from server on each visit. each index , should 
// depend on the 
coin quotas = [8,8]




/* =========================================== */
/* ============ Class Variables ============== */
/* =========================================== */
// variables that we need to track 

// Coin counting
static unsigned char CoinsTakenFromServer = 0;
static unsigned char TimesButtonPressed = 0;
static unsigned char CurrentCoinsHeld = 0;
static unsigned char CurrentCoinQuota = 0; // # coins to get from server
static unsigned char CoinsDropped = 0;


// Beacon targeting and tracking
static unsigned char 3CoinBeaconActive = true;
static unsigned char 5CoinBeaconActive = true;
static unsigned char 8CoinBeaconActive = true; 
static unsigned char CurrentTargetBeacon = 0;


// Self location tracking - if we stick with an absolute position system
// rotation from x axis
// rotation from y axis
// floor width
// floor legnth 

// State tracking
static int state;



/* ================================================== */
/* ============ Prototype Declarations ============== */
/* ================================================== */
// I hate this part


/* ================================================== */
/* ============ Arduino Setup and Loop ============== */
/* ================================================== */

void setup(){
    // set all the appropriate pins to output mode
    // system diagnostics and fun!
}

void loop(){
    switch (state){
        // implement state diagram part for part
    }
}


/* ================================================== */
/* ================= Event Detections =============== */
/* ================================================== */

// template: 
// unsigned char testFor


// Bump tests
unsigned char testForBump(){}
unsigned char testForUnevenBump(){}
unsigned char testForEvenBump(){}

// Vinyl tests
unsigned char testForVinylOn(){}
unsigned char testForVinylOff(){}

// IR tests
unsigned char testForBeacon(){}
unsigned char testForBeaconOff (){} 

unsigned char testForServer(){}
unsigned char testForServerStrong(){} // this will take testing, but 

// Timers
unsigned char testForTimerExpired(timerId){}

// Button pressing
unsigned char testForEnoughPressesForCoin(){}
unsigned char testForNeedMorePressesForCoin(){}

// Coin counting logic
unsigned char testForEnoughCoinsFromServer(){}
unsigned char testForEnoughCoinsForBeacon(){} // changes depending on strategy

unsigned char testForNoCoins(){}

// Debugging 
unsigned char testForSerialInput(){}


/* ================================================== */
/* ================ Service Execution =============== */
/* ================================================== */


// motor controls
void stopMoving(){};
void moveForwards(speed){};
void rotate(direction){};
void moveBackwards(speed){};

// right or left, speed of stronger motor, ratio of motor to motor
void arc(direction, speed, ratio){};


// Timer starters
void startFixedTimer(int timerid){}
void startVariableTimer(int timerid, int timeInMilliseconds){}


// Button pressers
void setupButtonPresser(){}
void retrackButtonPresser(){}

// Coin dropper
void extendCoinDropper(){}; //mechanism still to be decided on
void retractCoinDropper(){};


/* =========================================================== */
/* ================= Utilties - Math and Logic =============== */
/* =========================================================== */

void setState(int newState){
    state = newState;
    if (LOG_STATES) {
        Serial.print("State: ");
        Serial.print(state);
    }
}

void setMotorSpeed(int motor, int speed){
    // calculate velocity from motor and circuit specs
    // output the right voltage
}

void setMotorDirection(int motor, int pin){
    // change the digital output on the motor pin
}

void resetButtonPressedCount(){}

void setTargetBeacon(int beacon){}

unsigned char identifyBeacon(){} // identifies the beacon you are currently looking at

unsigned char getNumberOfPressesForNextCoin(){}







