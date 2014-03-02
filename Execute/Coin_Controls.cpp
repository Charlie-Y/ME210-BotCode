#include "Arduino.h"
#include "Coin_Controls.h"
#include "Pulse.h"

// wait we aren't using the motors... sigh...

#define PULSES_TO_90_DEGREES        45 // todo - figure this out
#define PULSE_PERIOD                100//?
#define HOPPER_DIRECTION_UP         1
#define HOPPER_DIRECTION_DOWN       0

static unsigned char hopper_enable_pin = 0; // pwm
static unsigned char hopper_direction_pin = 0; // digital

void coin_control_init(unsigned char hopper_enable, unsigned char hopper_direction){
    hopper_direction_pin = hopper_direction;
    hopper_enable_pin = hopper_enable;
    pinMode(hopper_enable_pin, OUTPUT);
    pinMode(hopper_direction_pin, OUTPUT);
    // digitalWrite(hopper_direction_pin, current_direction);
}

void lift_hopper(){
    // in case some other pin is using the pulse library
    InitPulse(hopper_direction_pin, PULSE_PERIOD);
    digitalWrite(hopper_direction_pin, HOPPER_DIRECTION_UP);
    Pulse(PULSES_TO_90_DEGREES);
}

void lower_hopper(){
    // in case some other pin is using the pulse library
    InitPulse(hopper_direction_pin, PULSE_PERIOD);

    digitalWrite(hopper_direction_pin, HOPPER_DIRECTION_DOWN);
    Pulse(PULSES_TO_90_DEGREES);
}

unsigned char done_moving_hopper(){
    return IsPulseFinished();
}
