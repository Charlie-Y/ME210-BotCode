#ifndef Bump_Sensing_h
#define Bump_Sensing_h

// use classes here
// maybe refactor this into tape sensor. they are just boolean sensors
class Bump_Sensor{
    public:
        Bump_Sensor(unsigned char);
        unsigned char is_bumped();
        unsigned char is_not_bumped();
    private:
        unsigned char sensor_pin;
};

#endif