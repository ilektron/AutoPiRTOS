/*****************************************************************************
 *   timer32.h:  Header file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.08.20  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#ifndef __TIMER32_H 
#define __TIMER32_H

// CodeRed - variable name changed in CMSIS 1.3
#define SystemFrequency SystemCoreClock

#define TIMER_FREQ		62
#define TIME_INTERVAL	((SystemFrequency/LPC_SYSCON->SYSAHBCLKDIV)/TIMER_FREQ - 1)

#define TIMER_COUNTS_FROM_uS(X)		(X)*((SystemFrequency/LPC_SYSCON->SYSAHBCLKDIV) / 1000000)
#define TIMER_uS_FROM_COUNTS(X)		(X)/((SystemFrequency/LPC_SYSCON->SYSAHBCLKDIV) / 1000000)

/* depending on the SystemFrequency and System AHB clock divider setting, 
if SystemFrequency = 60Mhz, SYSAHBCLKDIV = 4, SystemAHBFrequency = 1/4 SystemFrequency, 
10mSec = 150.000-1 counts */

void delay32Ms(uint8_t timer_num, uint32_t delayInMs);

#define TIMER32_0_DEFAULT_HANDLER 1
//#define TIMER32_1_DEFAULT_HANDLER 0

#ifdef TIMER32_0_DEFAULT_HANDLER
extern volatile uint32_t timer32_0_counter;
#endif

#ifdef TIMER32_1_DEFAULT_HANDLER
extern volatile uint32_t timer32_1_counter;
#endif

void enable_timer32(uint8_t timer_num);
void disable_timer32(uint8_t timer_num);
void reset_timer32(uint8_t timer_num);
void init_timer32(uint8_t timer_num, uint32_t timerInterval);

#endif /* end __TIMER32_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
