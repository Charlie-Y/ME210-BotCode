#include "Servo.h" // Arduino Servo library

#ifndef _Dumping_h
#define _Dumping_h

#define DUMPING_DELAY       260 // a bit more than half a second

// Its a servo. it uses the servo library. Needs only 1 pwm pin. cool!

void dumping_init( unsigned char);

void extend_dumper();
void retract_dumper();


#endif

