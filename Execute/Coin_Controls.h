#ifndef Coin_Controls_h
#define Coin_Controls_h

// lots of shit

void coin_control_init(unsigned char, unsigned char);

void lift_hopper();
void lower_hopper();

// This may be more complicated than just this....
void stop_coin();
void release_coin();

#endif