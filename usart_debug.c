/*
 * usart_debug.c
 * 
 * USART Debug Library
 * for STM32F10x Microcontroller Family
 *
 * Used to do debugging by sending messages
 * as ASCII characters by using USART1 
 *
 * Version: 1.0 | Ravidu Liyanage
 */
 
#include "usart_debug.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

/***************    USART Initialization Function    *******************************************/

void init_Debug(DEBUG_Type printer)
{
	// define macro
	#ifndef DEBUG_UART
		#define DEBUG_UART
	#endif
	
	//-----------| Alternate Function  |    Enable Port A   |     Enable USART1
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
	
	// Configures pins PA9 and PA10 as USART Tx and Rx
	if (printer.USART_portConfig == USART_portConfig1)
		AFIO->MAPR &= ~(AFIO_MAPR_USART1_REMAP);
	
	// Configures pins PB6 and PB7 as USART Tx and Rx
	else if (printer.USART_portConfig == USART_portConfig2)
		AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
	
	// Sets PA9 as Alternate Function Output Push-pull, max speed 50 MHz
	GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);
	
	/*
	 * PA10 - Rx pin should be set to floating input
	 * The reset value of all GPIO pins are floating input, so it don't need to be changed.
	 *
	 */
	
	// Calculates USARTDIV for BaudRate for 72MHz on APB1 bus
	float USARTDIV = ((float) (72000000/16) / printer.baud);
	USART1->BRR = ((((int) USARTDIV) << 4) | ((int)((USARTDIV - ((int) USARTDIV)) * 16)));
	
	//----------|   Enable Tx   |  Enable UART
	USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;

	debug_print("Connection Established\n");
}

/***************    USART Print Message Function    ********************************************/
void debug_print(char *msg, ...)
{
	char buff[80];
	#ifdef DEBUG_UART
	
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
	
	for(int i = 0; i < strlen(buff); i++)
	{
		USART1->DR = buff[i];
		while( !(USART1->SR & USART_SR_TXE) );
	}
	
	#endif
}

