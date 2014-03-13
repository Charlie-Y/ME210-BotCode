#include "State_Machine.h"


void setup(){
    Serial.begin(9600);
    Serial.println("Starting the BotCoin code...");
    setup_states();
}

void loop(){
    execute_current_state();
}


