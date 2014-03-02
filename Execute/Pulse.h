/*=============================================================================

  File: Pulse.h

  Description: N/A
  
  Notes:
    1) assumes exclusive use of Timer2 within the ATMega328
    
  TODO: enable multiple pins to be pulsed independently
  TODO: move to using Timer1
  
  10 Feb 2011
  Stellios Leventis

=============================================================================*/
#pragma once

/*---------------------------Public Function Prototypes----------------------*/
/*
  Type: pulse_t
  
  Description: offers improved resolution into the result of some of the 
    functions defined in this module.
*/
typedef enum {
  INIT_OK,
  INVALID_PIN,
  INVALID_PERIOD
} pulse_t;

/*---------------------------Public Function Prototypes----------------------*/
/****************************************************************************
  Function: InitPulse()

  Parameters:
    unsigned char pin,      a digital pin on which to pulse,  //only one  2 through 13 inclusive                     
    unsigned int period,    the period at which to produce pulses in units
                            of 1/10 ms, 10 through 1000 inclusive        
  Returns:
    pulse_t,                the result of the initialization, INIT_OK if 
                            everything went smoothly, an indication of the first
                            encountered error otherwise (see pulse_t definition)
    
  Description:
    Performs any initialization necessary for the Pulse library.

  Notes:
    1) assumes exclusive use of Timer2
    2) assumes a 16MHz internal clock
    3) this function must be called before using any of the library functions
   
****************************************************************************/
pulse_t InitPulse(unsigned char pin, unsigned int period);

/****************************************************************************
  Function: Pulse()

  Parameters:
    unsigned long num_pulses, the number of pulses to produce

  Returns:
    N/A

  Description:
    Begins a pulse train that will continue for the duration of the given 
    number of pulses.

  Notes:
    1) The output square wave always has a 50% duty cycle.  That is, the
       time the pulse is high is one-half of the total period specified.
****************************************************************************/
void Pulse(unsigned long num_pulses);

/****************************************************************************
  Function: IsPulseFinished()

  Parameters:
    N/A

  Returns:
    bool, whether the pulse train has finished
    
  Description:
    Returns whether the pulse train has finished.
    
  Notes:
    N/A
****************************************************************************/
bool IsPulseFinished();

/****************************************************************************
  Function: EndPulse()

  Parameters:
    N/A

  Returns:
    N/A
    
  Description:
    Enables the ability to prematurely end the pulse train.
    
  Notes:
    N/A
****************************************************************************/
void EndPulse();
