#include "Arduino.h"
#include "Timers.h"

#define SENSING_INTERVAL      100

// todo test all these things   
#define SERVER_BEACON_LOW       2600
#define SERVER_BEACON_HIGH      3300

//  todo - test test test
#define DEPOSITORY_BEACON_LOW   400
#define DEPOSITORY_BEACON_HIGH  1200


static unsigned char interrupt_pin = 0; //interrupt pin 0 = digital pin 2
unsigned int freq_count = 0;
float frequency = 0;

unsigned long last_time = 0;

// -------- Prototypes ----------- //
void inc_freq_count();
void set_frequency();
void attach_beacon_sensor_interrupt();
unsigned long time_since_last();


// -------- Public methods -------- //

void beacon_sensing_init(unsigned char pin){
    interrupt_pin = pin;
    attach_beacon_sensor_interrupt();
}

unsigned char server_found(){
    // get frequency
    // if the frequency is in the right range
    // then return true
    set_frequency();

    if ( (frequency > SERVER_BEACON_LOW) && (frequency < SERVER_BEACON_HIGH)){
        Serial.println("server_found true");
        return true;
    } else {
        Serial.println("server_found false");
        return false;
    }
}

unsigned char depository_found(){
    // get frequency
    // if the frequency is in the right range
    // then return true
    set_frequency();

    if ( (frequency > DEPOSITORY_BEACON_LOW) && (frequency < DEPOSITORY_BEACON_HIGH)){
        Serial.println("depository_found true");
        return true;
    } else {
        Serial.println("depository_found false");
        return false;
    }
}

unsigned char no_beacon_found(){
    // get frequency
    // if the frequency is in the right range
    // then return true
    set_frequency();
    if ( (frequency < DEPOSITORY_BEACON_LOW) || (frequency > SERVER_BEACON_HIGH)){
        // Serial.println("no_beacon_found true");
        return true;
    } else {
        // Serial.println("no_beacon_found false");
        return false;
    }
}

unsigned char beacon_found(){
    // get frequency
    // if the frequency is in the right range
    // then return true
    set_frequency();
    if ( (frequency > DEPOSITORY_BEACON_LOW) || (frequency < SERVER_BEACON_HIGH)){
        // Serial.println("beacon_found true");
        return true;
    } else {
        // Serial.println("beacon_found false");
        return false;
    }
}

// =======  Private methods ========== //

void attach_beacon_sensor_interrupt(){
    attachInterrupt(interrupt_pin, inc_freq_count, RISING);
}

void inc_freq_count(){
    freq_count++;
}

void set_frequency(){
    // need some kind of last time function
    long interval = time_since_last();
    frequency = freq_count * (1000 / interval);
    freq_count = 0;
    last_time = TMRArd_GetTime();
    attach_beacon_sensor_interrupt();

    // Serial.print("Frequency: ");
    // Serial.println(frequency);
}

unsigned long time_since_last(){
    // negatives?
    return ( TMRArd_GetTime() - last_time );
}


