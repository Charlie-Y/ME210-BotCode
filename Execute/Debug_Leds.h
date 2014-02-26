#ifndef Debug_Leds_h
#define Debug_Leds_h

class Debug_Led{
    public:
        Debug_Led( unsigned char );
        void led_on();
        void led_off();
        void toggle();
        // void init(unsigned char,unsigned char,unsigned char );
        // void clear_debug_leds();
        // void debug_red_on();
        // void debug_green_on();
        // void debug_blue_on();
    private:
        unsigned char led_pin;
        unsigned char is_on;
};


#endif