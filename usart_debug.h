/*
 * usart_debug.h
 * 
 * USART Debug Library
 * for STM32F10x Microcontroller Family
 *
 * Used to do debugging by sending messages
 * as ASCII characters by using USART1 
 *
 * Version: 1.0 | Ravidu Liyanage
 */

#ifndef __USART_DEBUG_H
#define __USART_DEBUG_H

#include "stm32f10x.h"

// USART Port Remapping Enumeration
typedef enum {
	USART_portConfig1, // To use PA9 as Tx & PA10 as Rx
	USART_portConfig2 // To use PB6 as Tx & PB7 as Rx
} USART_portConfigType;

// Configuration Structure
typedef struct {
	uint32_t baud;
	USART_portConfigType USART_portConfig;
} DEBUG_Type;

/***********************************************************************/
/*                      Function Prototypes                            */
/***********************************************************************/

// USART Initialization
void init_Debug(DEBUG_Type printer);
void debug_print(char *msg, ...);

#endif /* __USART_DEBUG_H */

