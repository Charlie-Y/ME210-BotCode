#include "State_Machine.h"


void setup(){
    // call all the initialization code for all the libraries
    Serial.begin(9600);
    Serial.println("Starting the BotCoin code...");
    // log_states();
    setup_states();
}

void loop(){
    execute_current_state();
    // delay(100);
}


// todo - implement motor controls
// todo - implement button pressing controls
// todo - implement coin controls
