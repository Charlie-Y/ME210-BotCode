#include "State_Machine.h"


void setup(){
    // call all the initialization code for all the libraries
    //beacon sensing init
    // button pressing init
    // tape sensing init
    Serial.begin(9600);
    Serial.println("Starting the BotCoin code...");
    // log_states();
    setup_states();
}

void loop(){
    execute_current_state();
    delay(40);
}

