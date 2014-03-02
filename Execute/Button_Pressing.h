#include "Servo.h" // Arduino Servo library

#ifndef Button_Pressing_h
#define Button_Pressing_h


#define BUTTON_PRESSER_DELAY       260 // a bit more than half a second

// Its a servo. it uses the servo library. Needs only 1 pwm pin. cool!

void button_pressing_init( unsigned char);

void extend_button_presser();
void retract_button_presser();

unsigned char button_presser_finished();

#endif
