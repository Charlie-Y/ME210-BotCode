#ifndef Coin_Controls_h
#define Coin_Controls_h

// This will hork by raising the entire hopper at once using a stepper motor

void coin_control_init(unsigned char, unsigned char);

void lift_hopper();
void lower_hopper();

// Wrapper function for is pulse finished
unsigned char done_moving_hopper();

#endif