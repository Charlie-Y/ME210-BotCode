#ifndef Motor_Controls_h
#define Motor_Controls_h

#define WHEEL_FORWARD             0
#define WHEEL_BACKWARD            1

#define DIR_LEFT            0
#define DIR_CLOCKWISE       0

#define DIR_RIGHT           1
#define DIR_C_CLOCKWISE     1

// Speeds go from 0 - 10
#define MAX_WHEEL_SPEED     10

// Initialiation
void motor_control_init(unsigned char, unsigned char, unsigned char, unsigned char);

// Speeds
void move_forwards(unsigned char);
void move_backwards(unsigned char);

void stop_moving();

// Rotates in place
void rotate_right(unsigned char);
void rotate_left(unsigned char);
void rotate_dir(unsigned char);

// Pivot on one wheel
void pivot_right(unsigned char);
void pivot_left(unsigned char);
void pivot_dir(unsigned char, unsigned char);
// First param is speed of major wheel, second param is ratio/second speed/something?
void arc_right( unsigned char, unsigned char);
void arc_left( unsigned char, unsigned char);
// major wheel speed, ratio, direction?
void arc_dir(unsigned char, unsigned char, unsigned char );

#endif