#ifndef Beacon_Sensing_h
#define Beacon_Sensing_h

// pass in the required pin - needs the digital pin 2
// use classes here. we can have up to 2 sensors

void beacon_sensing_init(unsigned char);

unsigned char server_found();
unsigned char depository_found();
unsigned char no_beacon_found();
unsigned char beacon_found();

#endif


// //The pin Arduino outputs information to
// int pin = 3;
// volatile int state = LOW;
// int freqcount=0;
// float freq=0;

// void setup()
// {
//  Serial.begin(9600);
//  Serial.println("Starting IR Beacon Sensing...");
//  pinMode(pin, OUTPUT);
// //mapped pin external interrupts: number 0 (digital pin 2) and 1 (digital pin 3) 
//  attachInterrupt(0, count, RISING);
//  TMRArd_InitTimer(0, TIME_INTERVAL );
// }

// void loop()
// {
//  //Serial.println(freqcount);
//    //digitalWrite(pin, state); 
//  if (TMRArd_IsTimerExpired(0)){
//    detachInterrupt(0);
//    Serial.println("The Frequency count is: ");
//    freq=freqcount*(1000/TIME_INTERVAL);
//    Serial.println(freq);  
//    freqcount=0; 
//    attachInterrupt(0,count, RISING);
//    TMRArd_InitTimer(0, TIME_INTERVAL );
//  }
// }

// void count()
// {
//  freqcount++;
// //  Serial.println(freqcount);
// //  state=!state;
// }