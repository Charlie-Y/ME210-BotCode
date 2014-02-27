/*=============================================================================
 
  Timers.h
 
  10 Jan 2012
  Betai Koffi
  Stellios Leventis

  description: This module encapsulates a 32-bit software timer based 
    on the millis() library of Arduino Uno.  

=============================================================================*/
#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

                            
/*---------------------------Public Functions--------------------------------*/

// A list of the different return types
typedef enum { TMRArd_ERR           = -1,
               TMRArd_ACTIVE        =  1,
               TMRArd_EXPIRED       =  1,
               TMRArd_OK            =  0,
               TMRArd_NOT_ACTIVE    =  0,
               TMRArd_NOT_EXPIRED   =  0
} TMRArdReturn_t;

TMRArdReturn_t TMRArd_InitTimer(unsigned char Num, unsigned long NewTime);
TMRArdReturn_t TMRArd_SetTimer(unsigned char Num, unsigned long NewTime);
TMRArdReturn_t TMRArd_StartTimer(unsigned char Num);
TMRArdReturn_t TMRArd_StopTimer(unsigned char Num);
TMRArdReturn_t TMRArd_IsTimerActive(unsigned char Num);
TMRArdReturn_t TMRArd_IsTimerExpired(unsigned char Num);
TMRArdReturn_t TMRArd_ClearTimerExpired(unsigned char Num);
unsigned long  TMRArd_GetTime(void);
