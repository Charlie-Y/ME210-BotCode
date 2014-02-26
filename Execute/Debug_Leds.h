#ifndef Debug_Leds_h
#define Debug_Leds_h

class Debug{
    public:
        void init(unsigned char,unsigned char,unsigned char );
        void clear_debug_leds();
        void debug_red_on();
        void debug_green_on();
        void debug_blue_on();
};


#endif