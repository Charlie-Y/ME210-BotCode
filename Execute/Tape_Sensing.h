#ifndef Tape_Sensing_h
#define Tape_Sensing_h

// use classes here
class Tape_Sensor{
    public:
        Tape_Sensor(unsigned char);
        unsigned char is_on_tape();
        unsigned char is_off_tape();
    private:
        unsigned char sensor_pin;
};

#endif