/*
 * HAL_GPIO.c
 * 
 * GPIO Driver Library
 * for STM32F10x Microcontroller Family
 *
 * Version: 1.0 | Ravidu Liyanage
 */
 
#include "pwr_ctrl.h"

/******************************************************************************************/
/*                             Go to Standby Mode Function                                */
/******************************************************************************************/

void gotoStandby(void)
{
	// Enable Power Interface Clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	// Set SLEEPDEEP bit of Cortex -M3 System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP;
	
	// Enter Standby Mode when CPU enters Deep Sleep 
	PWR->CR |= PWR_CR_PDDS;
	
	// Clears WUF bit (Wakeup Flag) in Power Control / Status Register
	PWR->CR |= PWR_CR_CWUF;
	
	// Enable Wake up pin
	PWR->CSR |= PWR_CSR_EWUP;
	
	// Request Wait For Interrupt
	__WFI();
}

