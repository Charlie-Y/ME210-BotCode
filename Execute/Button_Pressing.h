#ifndef Button_Pressing_h
#define Button_Pressing_h

// Pass in the appropriate pins. we only need 1 button presser, so no classes
void button_pressing_init( unsigned char, unsigned char);

void extend_button_presser();
void retract_button_presser();

#endif