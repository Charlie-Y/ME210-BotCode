/*=============================================================================

  File: Pulse.cpp
  
  Notes:
    1) T_pulse = (T_Timer2 * ((255 - TCNT2) + 1)) * code_postcaler * 2_toggles_per_period
               = (1/(f_clk/prescaler) * ((255 - TCNT2) + 1) * code_postcaler * 2_toggles_per_period
               = (1/(16MHz/1024) * ((255 - TCNT2) + 1) * 12 * 2
               = 1.536ms * ((255 - TCNT2) + 1)
    2) although likely unnecessary, when the Timer2 overlow interrupt fires,
       all interrupt are globally disabled until the ISR finishes
    3) a software postscaler had to be used to sufficiently scale down the
       pulse frequency.  The range is approximately 10Hz ~< pulse_frequency ~< 1kHz
    
    
  10 Feb 2011
  Stellios Leventis

=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include "Pulse.h"
#include <limits.h> // for UCHAR_MAX macro

// include the appropriate version Arduino header file
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*---------------------------Constants---------------------------------------*/
static const unsigned char MIN_PIN = 2;
static const unsigned char MAX_PIN = 13;
static const unsigned int MIN_PERIOD = 10;    // [1/10*ms]
static const unsigned int MAX_PERIOD = 1000;  // [1/10*ms]

/*---------------------------Definitions-------------------------------------*/
/*---------------------------Helper-Function Prototypes----------------------*/
static void InitPins(unsigned char pin);
static bool IsValidPin(unsigned char pin);
static void InitTimer2();
static bool IsValidPeriod(unsigned int period);
static unsigned char ConvertPeriodToTicks(unsigned int period);

/*---------------------------Module Variables--------------------------------*/
static unsigned char output_pin;
static unsigned int remaining_toggles;
static unsigned char ticks_per_interrupt;

/*---------------------------Public Function Definitions---------------------*/
pulse_t InitPulse(unsigned char pin, unsigned int period) {
  remaining_toggles = 0;
  
  if (!IsValidPeriod(period)) return INVALID_PERIOD;
  if (!IsValidPin(pin)) return INVALID_PIN;
  
  ticks_per_interrupt = ConvertPeriodToTicks(period);
  
  InitPins(pin);
  InitTimer2();
  
  return INIT_OK;
}

void Pulse(unsigned long num_pulses) {
  remaining_toggles = 2*num_pulses;
  TCNT2 = ticks_per_interrupt;  // schedule the first interrupt
  TIMSK2 |= (1<<TOIE2);         // enable interrupts for Timer2
}

bool IsPulseFinished() {
  return (remaining_toggles == 0);
}

void EndPulse() {
  remaining_toggles = 0;
}

ISR(TIMER2_OVF_vect) {
  static unsigned char toggle = 0;
  static unsigned char counter = 0; // to scale down frequency

  // THIS IS ONLY NECESSARY IF NESTED INTERRUPTS ARE ENABLED
  cli(); // disable interrupts globally
  
  // toggle the output pin(s)
  if (12 <= ++counter ) {
    counter = 0;
    digitalWrite(output_pin, toggle == 0 ? HIGH : LOW);
    toggle = ~toggle;
    remaining_toggles--;
  }
  
  // schedule another interrupt if required, 
  if (0 < remaining_toggles) TCNT2 = ticks_per_interrupt;
  // otherwise disable this interrupt
  else TIMSK2 &= ~(1<<TOIE2);
  
  // clear the source of the interrupt
  // (TOV2 is cleared by hardware when executing the ISR)
  
  sei(); // enable interrupts globally
}


/*---------------------------Helper-Function Definitions---------------------*/
static void InitPins(unsigned char pin) {
  // configure the given pins as digital outputs and initialize to 0
  output_pin = pin;
  pinMode(output_pin, OUTPUT);
  digitalWrite(output_pin, LOW);
}

static void InitTimer2() {
  // disable the timer overflow interrupt during configuration
  TIMSK2 &= ~(1<<TOIE2);
  
  // configure Timer2 in normal mode (pure counting, no PWM etc.)
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  //---ensure compare match output modes are disabled
  TCCR2A &= ~((1<<COM2A0) | (1<<COM2A1) | (1<<COM2B0) | (1<<COM2B0));
  //---ensure force output compare bits are 0
  TCCR2B &= ~((1<<FOC2A) | (1<<FOC2B));

  // configure the clock source as internal I/O clock
  ASSR &= ~(1<<AS2);

  // disable output compare options for triggering an interrupt
  TIMSK2 &= ~(1<<OCIE2A);           // disable compare match A
  TIMSK2 &= ~(1<<OCIE2B);           // disable compare match B
  
  // configure the Timer2 tick frequency (see p.162 datasheet)
  // f_T2 = f_CLK_IO/ prescaler => 16MHz / 1024 => 15.625Hz  
  TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS21);
  
  // begin with the Timer2 interrupt disabled
  TIMSK2 &= ~(1<<TOIE2);
}

static bool IsValidPin(unsigned char pin) {
  return (MIN_PIN <= pin && pin <= MAX_PIN);
}

static bool IsValidPeriod(unsigned int period) {
  return (MIN_PERIOD <= period && period <= MAX_PERIOD);
}

static unsigned char ConvertPeriodToTicks(unsigned int period) {
  // as currently configured: T_pulse = 1.536ms * ((255 - TCNT2) + 1)

  // TCNT2 = 255 - (T_pulse / 1.536ms) - 1)
  //       = 255 - ((T_pulse_in_tenths_ms / 15.36_tenths_ms) - 1)
  //       = 255 - result
  
  // sensitivity ~= 0.391ms/TCNT2_increment
  float result = ((float)period / (float)3.91 - (float)1);
  // guard against overflow errors
  if (result < 0) result == 0;
  else if (result > 255) result == 255;
  
  return UCHAR_MAX - (unsigned char)result;
}
/*---------------------------End of File-------------------------------------*/
