/*=============================================================================

Module
 Timers.cpp
Description
 This library contains code that enables timers by using the millis() function provided by Arduino.

Arduino IDE version: 0022
Arduino UNO R2 resources used

 10 Jan 2011
 Betai Koffi
 Stellios Leventis
 
=============================================================================*/

/*---------------------------Dependencies------------------------------------*/
#include "Timers.h"

/*---------------------------Module Definitions------------------------------*/
typedef struct {
  unsigned long duration;                 // the duration of the timer
  unsigned long start_time;      // time at which the timer was started
} timer_t;

/*---------------------------Module Variables--------------------------------*/
static unsigned int activeTimers = 0;    // use a 16-bit int to represent the status of each timer (1 for on 0 for off)
#define NUM_BITS_IN_BYTE  8
#define MAX_NUM_TIMERS    (sizeof(activeTimers)*NUM_BITS_IN_BYTE) // the maximum number of timer this module can keep track of
static timer_t timers[MAX_NUM_TIMERS];   // array of timers


/*===========================Module Code=====================================*/

/*-------------------------------------------------------------------------
Function:  TMRArd_InitTimer
Prototype: TMRArdReturn_t TMRArd_InitTimer(unsigned char Num, unsigned long NewTime)
  Parameters:unsigned char Num, the number of the timer to start (0-15)
  long NewTime, the number of ticks to be counted
  Returns:TMRArd_ERR if the requested timer does not exist, TMRArd_OK otherwise.

 Description
  Sets the NewTime (positive) into the chosen timer and clears any previous event flag and sets the timer active to begin counting.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_InitTimer(unsigned char Num, unsigned long NewTime){
  if ( Num >= MAX_NUM_TIMERS ) return TMRArd_ERR;  // tried to initialize a timer that doesn't exist
  timers[Num].duration = NewTime;    // set the duration and start time of the new timer
  timers[Num].start_time = millis();
  activeTimers |= 1<<Num;            // activate the timer
  return TMRArd_OK;
}

/*-------------------------------------------------------------------------
Function:  TMRArd_SetTimer
Prototype: TMRArdReturn_t TMRArd_SetTimer(unsigned char Num, unsigned long NewTime)
Parameters:unsigned char Num, the number of the timer to set (0-15)
long NewTime, the number of ticks to be counted
Returns:TMRArd_ERR if requested timer does not exist, TMRArd_OK otherwise

 Description
     Sets the time for a timer, but does not make it active.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_SetTimer(unsigned char Num, unsigned long NewTime){
  if( Num >= MAX_NUM_TIMERS ) return TMRArd_ERR;
  timers[Num].duration = NewTime; // set duration to NewTime but do nothing to active status
  return TMRArd_OK;
}

/*-------------------------------------------------------------------------
Function:  TMRArd_StartTimer
Prototype: TMRArdReturn_t TMRArd_StartTimer(unsigned char Num)
  Parameters:unsigned char Num the number of the timer to start (0-15)
    Returns:signed char TMRArd_ERR for error TMRArd_OK for success

 Description
     Sets the active flag in TMRArd_ActiveFlags to start a timer that
     was previously set  or to restart a stopped timer.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_StartTimer(unsigned char Num){
  if( Num >= MAX_NUM_TIMERS ) return TMRArd_ERR;
  timers[Num].start_time = millis();  // reset the startTime
  activeTimers |= 1<<Num;             // activate timer
  return TMRArd_OK;
}

/*-------------------------------------------------------------------------
Function: TMRArd_StopTimer
Prototype:TMRArdReturn_t TMRArd_StopTimer(unsigned char Num)
Parameters:unsigned char Num the number of the timer to stop (0-15)
ReturnsTMRArd_ERR for error (timer doesn't exist) TMRArd_OK for success.

 Description
     Clears the bit in TMRArd_ActiveFlags associated with this
     timer. This will cause it to stop counting.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_StopTimer(unsigned char Num){
  if( Num >= MAX_NUM_TIMERS || TMRArd_IsTimerActive(Num) == TMRArd_NOT_ACTIVE ) return TMRArd_ERR;
  timers[Num].duration -= (millis() - timers[Num].start_time); // keep track of how much time has passed
  activeTimers &= ~(1<<Num);     // deactivate the timer
  return TMRArd_OK;
}

/*-------------------------------------------------------------------------
Function: TMRArd_IsTimerActive
Prototype:TMRArdReturn_t TMRArd_IsTimerActive(unsigned char Num)
Parameters:unsigned char Num the number of the timer to check (0-15)
Returns:TMRArd_ERR if requested timer is not valid
TMRArd_NOT_ACTIVE if timer is not active 
TMRArd_ACTIVE if it is active

 Description
     This function is used to determine if a timer is currently counting.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_IsTimerActive(unsigned char Num){
  if( Num >= MAX_NUM_TIMERS ) return TMRArd_ERR;
  if( activeTimers & 1<<Num ) return TMRArd_ACTIVE;
  return TMRArd_NOT_ACTIVE;
}

/*-------------------------------------------------------------------------
Function: TMRArd_IsTimerExpired
Prototype:TMRArdReturn_t TMRArd_IsTimerExpired(unsigned char Num)
Parameters:unsigned char Num, the number of the timer to test (0-15)
ReturnsTMRArd_ERR if requested timer does not exist
TMRArd_NOT_EXPIRED if not expired 
TMRArd_EXPIRED if expired

 Description
This function tests the flags to determine if the requested timer has expired.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_IsTimerExpired(unsigned char Num){
  if ( Num > MAX_NUM_TIMERS || TMRArd_IsTimerActive(Num) == TMRArd_NOT_ACTIVE ) return TMRArd_ERR;
  if (timers[Num].duration < (millis() - timers[Num].start_time)) // check to see if target duration > delta time
    return TMRArd_EXPIRED;
  return TMRArd_NOT_EXPIRED;
}

/*-------------------------------------------------------------------------
Function: TMRArd_ClearTimerExpired
Prototype:TMRArdReturn_t TMRArd_ClearTimerExpired(unsigned char Num)
Parametersunsigned char Num the timer whose event flag should be 
cleared (0-15)
ReturnsTMRArd_ERR if requested timer does not exist
TMRArd_OK otherwise

 Description
     Clears the appropriate bit in TMRArd_EventFlags to show that
     the event has been serviced.
  -------------------------------------------------------------------------*/

TMRArdReturn_t TMRArd_ClearTimerExpired(unsigned char Num){
  if( Num >= MAX_NUM_TIMERS ) return TMRArd_ERR;
  activeTimers &= ~(1<<Num); // deactivate timer
  return TMRArd_OK;
}

/*-------------------------------------------------------------------------
Function:  TMRArd_GetTime
Prototype: unsigned long TMRArd_GetTime(void)
ParametersNone
Returnsa 32 bit number between 0 and 4,294,967,295 (232-1) reflecting the current number of ticks on the timer.  Timer rollover will occur after about 50 days.

 Description
     Simply returns the value on a free-running counter kept by the module. 
  -------------------------------------------------------------------------*/

unsigned long TMRArd_GetTime(void){
  return millis();
}
