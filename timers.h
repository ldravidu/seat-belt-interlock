/*
 * timers.h
 * 
 * Timer Control Library
 * for STM32F10x Microcontroller Family
 *
 * Used to control Timers and Counters
 * Delay functions with General Purpose Timers and SysTick 
 *
 * Version: 1.1 | Ravidu Liyanage
 */

#ifndef __TIMERS_H
#define __TIMERS_H

#include "stm32f10x.h"

/********************* Function Prototypes ********************/
void init_SysTick(void); // Initializes SysTick
void init_timer(void); // Initializes Timer

void delay_us(int us); // Delay Function, uses micro seconds
void delay_ms(int ms); // Delay Function, uses milli seconds
void delay(int ms); // Delay Function, uses milli seconds

#endif /* __TIMERS_H */

