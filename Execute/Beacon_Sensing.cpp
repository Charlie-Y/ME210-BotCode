#include "Arduino.h"
#include "Timers.h"

#define SENSING_INTERVAL        100 // not using this.

#define SERVER_BEACON_LOW       400 
#define SERVER_BEACON_HIGH      1000

#define DEPOSITORY_BEACON_LOW   1600
#define DEPOSITORY_BEACON_HIGH  3300

#define NUM_FREQUENCIES         3 // keep track of the last 10 frequencies // not using

#define NUM_IGNORES             3 // changes depnding on # of serial prints

static unsigned char interrupt_pin = 0; //interrupt pin 0 = digital pin 2
unsigned int freq_count = 0;
float frequency = 0;

static float last_freqencies[NUM_FREQUENCIES]; // not using

unsigned long last_time = 0;

unsigned char times_signal_ignored; //ignores a few 

// -------- Prototypes ----------- //
void inc_freq_count();
void set_frequency();
void add_frequency(float);
void attach_beacon_sensor_interrupt();
float average_last_frequencies();

unsigned long time_since_last();


// -------- Public methods -------- //

void beacon_sensing_init(unsigned char pin){
    interrupt_pin = pin;
    attach_beacon_sensor_interrupt();
    times_signal_ignored = 0;
    for (int i = 0; i < NUM_FREQUENCIES; ++i)
    {
        last_freqencies[i] = 0;
        /* code */
    }
}

void beacon_sensing_state_changed(){
    // reset the ignorance counter
    times_signal_ignored = 0;
}

unsigned char server_found(){
    // get frequency
    // if the frequency is in the right range
    // then return true
    set_frequency();

    if ( (frequency > SERVER_BEACON_LOW) && (frequency < SERVER_BEACON_HIGH)){
        // Serial.println("server_found true");
        return true;
    } else {
        // Serial.println("server_found false");
        return false;
    }
}

unsigned char depository_found(){
    set_frequency();
    if ( (frequency > DEPOSITORY_BEACON_LOW) && (frequency < DEPOSITORY_BEACON_HIGH)){
        // Serial.println("depository_found true");
        // ignore the first few catches
        if (times_signal_ignored == NUM_IGNORES){
            return true;
        } else if (times_signal_ignored < NUM_IGNORES){
            times_signal_ignored++;
            return false;
        }

    } else {
        // Serial.println("depository_found false");
        return false;
    }
}

unsigned char no_beacon_found(){
    // get frequency
    // if the frequency is in the right range
    // then return true
    set_frequency();
    if ( (frequency < SERVER_BEACON_LOW) || (frequency > DEPOSITORY_BEACON_HIGH)){
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

// todo - take the average of the last few frequencies
void set_frequency(){
    // need some kind of last time function
    long interval = time_since_last();
    last_time = TMRArd_GetTime();
    
    
    // -- if using average -- this makes the problem harder
    // float new_frequency = freq_count * (1000 / interval);
    // add_frequency(new_frequency);
    // frequency = average_last_frequencies();

    // -- if not using average
    frequency = freq_count * (1000 / interval);


    freq_count = 0;
    attach_beacon_sensor_interrupt();

    Serial.print("       ");// hmm it works only if it printlines...
    // Serial.print("Frequency: ");
    // Serial.println(frequency);
}

unsigned long time_since_last(){
    // negatives?
    return ( TMRArd_GetTime() - last_time );
}

// this is not useful.

// float average_last_frequencies(){
//     float total = 0;
//     for (int i = 0; i < NUM_FREQUENCIES; ++i)
//     {
//         total += last_freqencies[i];
//     }
//     return (total / NUM_FREQUENCIES); // rounding issues?
// }

// void add_frequency(float new_frequency){
//     for (int i = 0; i < NUM_FREQUENCIES - 1; ++i)
//     {
//         last_freqencies[i] = last_freqencies[i+1];
//         /* code */
//     }
//     last_freqencies[NUM_FREQUENCIES - 1] = new_frequency;
// }

