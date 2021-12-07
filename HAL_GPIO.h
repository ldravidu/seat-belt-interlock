/*
 * HAL_GPIO.h
 * 
 * GPIO Driver Library
 * for STM32F10x Microcontroller Family
 *
 * Version: 1.1 | Ravidu Liyanage
 */

#ifndef __HAL_GPIO_H
#define __HAL_GPIO_H

#include "stm32f10x.h"

// Digital States
#define LOW		0
#define HIGH	1

// Port Names
#define PORTA	GPIOA
#define PORTB	GPIOB
#define PORTC	GPIOC
#define PORTD	GPIOD
#define PORTE	GPIOE
#define PORTF	GPIOF
#define PORTG	GPIOG

// Pin Modes
#define OUTPUT_MODE					((uint32_t) 0x01)
#define INPUT_MODE					((uint32_t) 0x02)

// Input Mode Types
#define INPUT_ANALOG				((uint32_t) 0x00)
#define INPUT_FLOATING				((uint32_t) 0x01)	// Default at reset
#define INPUT_PU_PD					((uint32_t) 0x02)	// Input with Pull Up or Pull Down

// Output Mode Types
#define OUTPUT_GEN_PURPOSE			((uint32_t) 0x00)	// General Purpose Output push-pull
#define OUTPUT_OD					((uint32_t) 0x01)	// General Purpose Output Open Drain
#define OUTPUT_ALT_FUNCTION			((uint32_t) 0x02)	// Alternate Function Output push-pull
#define OUTPUT_ALT_FUNCTION_OD		((uint32_t) 0x03)	// Alternate Function Output Open Drain

// Pin Speeds / Slew Rates
#define SPEED_10MHZ					((uint32_t) 0x01)
#define SPEED_2MHZ					((uint32_t) 0x02)
#define SPEED_50MHZ					((uint32_t) 0x03)

// Clock Enabling
#define GPIO_CLOCK_ENABLE_ALT_FUNC		(RCC->APB2ENR |= (1 << 0))
#define GPIO_CLOCK_ENABLE_PORT_A		(RCC->APB2ENR |= (1 << 2))
#define GPIO_CLOCK_ENABLE_PORT_B		(RCC->APB2ENR |= (1 << 3))
#define GPIO_CLOCK_ENABLE_PORT_C		(RCC->APB2ENR |= (1 << 4))
#define GPIO_CLOCK_ENABLE_PORT_D		(RCC->APB2ENR |= (1 << 5))
#define GPIO_CLOCK_ENABLE_PORT_E		(RCC->APB2ENR |= (1 << 6))

// High bit positions for CRH Register Config and Mode
#define CNF_POS_BIT1				(PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2				(PINPOS[pinNumber] + 3)

// Configuration Structure
typedef struct {
	GPIO_TypeDef *port;
	uint32_t pin;
	uint32_t mode;
	uint32_t mode_type;
	uint32_t pull;
	uint32_t speed;
	uint32_t alt_func;
} GPIO_TYPE;

// Edge Select Enumeration
typedef enum {
	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE
} edge_select;

/***********************************************************************/
/*                      Function Prototypes                            */
/***********************************************************************/

// GPIO Configuration
static void config_pin (GPIO_TypeDef *port, uint32_t pinNumber, uint32_t mode_type);
static void config_pin_speed (GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode);

// GPIO Pin Functions
void gpio_write (GPIO_TYPE gpio_type, uint8_t state);
void gpio_toggle (GPIO_TypeDef *port, uint32_t pinNumber);
void gpio_init (GPIO_TYPE gpio_type);
uint32_t gpio_read (GPIO_TYPE gpio_type);

// GPIO Interrupt Functions
void configure_gpio_interrupt (GPIO_TypeDef *port, uint32_t pinNumber, edge_select edge);
void enable_gpio_interrupt (uint32_t pinNumber, IRQn_Type irqNumber);
void clear_gpio_interrupt (uint32_t pinNumber);

#endif /* __HAL_GPIO_H */

