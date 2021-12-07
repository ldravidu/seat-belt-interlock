/*
 * HAL_GPIO.c
 * 
 * GPIO Driver Library
 * for STM32F10x Microcontroller Family
 *
 * Version: 1.1 | Ravidu Liyanage
 */
 
#include "HAL_GPIO.h"
#include <stdint.h>

/*
 * Every pin in the high and low control registers has 4 associated bits.
 * Thus, the position of every pin is shifted 4 bits. This array keeps track 
 * of how much each pinNumber has to be shifted in the proper bit.
 */
uint32_t PINPOS[16] = {
	(0x00),
	(0x04),
	(0x08),
	(0x0C),
	(0x10),
	(0x14),
	(0x18),
	(0x1C),
	(0x00),
	(0x04),
	(0x08),
	(0x0C),
	(0x10),
	(0x14),
	(0x18),
	(0x1C)
};

/******************************************************************************************/
/*                         GPIO Pin Configuration Function                                */
/******************************************************************************************/

static void config_pin (GPIO_TypeDef *port, uint32_t pinNumber, uint32_t mode_type)
{
	if (pinNumber >= 8) // Control High Register
	{
		switch (mode_type)
		{
			//---------------INPUT/OUTPUT MODES-----------------
			
			case OUTPUT_GEN_PURPOSE | INPUT_ANALOG:
				port->CRH &= ~( (1 << CNF_POS_BIT1) | (1 << CNF_POS_BIT2) );
			break;
			
			case OUTPUT_OD | INPUT_FLOATING:
				port->CRH &= ~( 1 << CNF_POS_BIT2 );
				port->CRH |= ( 1 << CNF_POS_BIT1 );
			break;
			
			case OUTPUT_ALT_FUNCTION | INPUT_PU_PD:
				port->CRH |= OUTPUT_ALT_FUNCTION << (CNF_POS_BIT1);
			break;
			
			case OUTPUT_ALT_FUNCTION_OD:
				port->CRH |= OUTPUT_ALT_FUNCTION_OD << (CNF_POS_BIT1);
			break;
		}
	}
	else // Control Low Register
	{
		switch (mode_type)
		{
			//---------------INPUT/OUTPUT MODES-----------------
			
			case OUTPUT_GEN_PURPOSE | INPUT_ANALOG:
				port->CRL &= ~( (1 << CNF_POS_BIT1) | (1 << CNF_POS_BIT2) );
			break;
			
			case OUTPUT_OD | INPUT_FLOATING:
				port->CRL &= ~( 1 << CNF_POS_BIT2 );
				port->CRL |= ( 1 << CNF_POS_BIT1 );
			break;
			
			case OUTPUT_ALT_FUNCTION | INPUT_PU_PD:
				port->CRL |= OUTPUT_ALT_FUNCTION << (CNF_POS_BIT1);
			break;
			
			case OUTPUT_ALT_FUNCTION_OD:
				port->CRL |= OUTPUT_ALT_FUNCTION_OD << (CNF_POS_BIT1);
			break;
		}
	}
}

/******************************************************************************************/
/*                      GPIO Pin Speed Configuration Function                             */
/******************************************************************************************/

static void config_pin_speed (GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode)
{
	if (pinNumber >= 8)
	{
		if (mode == INPUT_MODE) // Set Control High Register to input mode
			port->CRH &= ~( (1 << (PINPOS[pinNumber])) | (1 << (PINPOS[pinNumber] + 1)) );
		else
			port->CRH |= (pinSpeed << (PINPOS[pinNumber])); // Set Control High Register to Output mode at given speed
	}
	else
	{
		if (mode == INPUT_MODE) // Set Control Low Register to input mode
			port->CRL &= ~( (1 << (PINPOS[pinNumber])) | (1 << (PINPOS[pinNumber] + 1)) );
		else
			port->CRL |= (pinSpeed << (PINPOS[pinNumber])); // Set Control Low Register to Output mode at given speed
	}
}

/******************************************************************************************/
/*                               GPIO Pin Write Function                                  */
/******************************************************************************************/

void gpio_write (GPIO_TYPE gpio_type, uint8_t state)
{
	if (state)
		gpio_type.port->BSRR |= (1 << gpio_type.pin);
	else
		gpio_type.port->BSRR |= (1 << (gpio_type.pin + 16));
}

/******************************************************************************************/
/*                               GPIO Pin Read Function                                  */
/******************************************************************************************/

uint32_t gpio_read (GPIO_TYPE gpio_type)
{
	return ( (1 << (gpio_type.pin)) & (gpio_type.port->IDR) );
}

/******************************************************************************************/
/*                                GPIO Pin Toggle Function                                */
/******************************************************************************************/

void gpio_toggle (GPIO_TypeDef *port, uint32_t pinNumber)
{
	port->ODR ^= (1 << pinNumber);
}

/******************************************************************************************/
/*                          GPIO Pin Initialization Function                              */
/******************************************************************************************/

void gpio_init (GPIO_TYPE gpio_type)
{
	// Enable Clock for the relevant Port
	if (gpio_type.port == PORTA)
		GPIO_CLOCK_ENABLE_PORT_A;
	
	else if (gpio_type.port == PORTB)
		GPIO_CLOCK_ENABLE_PORT_B;
	
	else if (gpio_type.port == PORTC)
		GPIO_CLOCK_ENABLE_PORT_C;
	
	else if (gpio_type.port == PORTD)
		GPIO_CLOCK_ENABLE_PORT_D;
	
	else if (gpio_type.port == PORTE)
		GPIO_CLOCK_ENABLE_PORT_E;
	
	// Setup GPIO Pin
	config_pin (gpio_type.port, gpio_type.pin, gpio_type.mode_type);
	config_pin_speed (gpio_type.port, gpio_type.pin, gpio_type.speed, gpio_type.mode);
}

/******************************************************************************************/
/*                       GPIO Interrupt Configuration Function                            */
/******************************************************************************************/

void configure_gpio_interrupt (GPIO_TypeDef *port, uint32_t pinNumber, edge_select edge)
{
	// Enable Alternate Function
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	if (port == PORTA) // For port A
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PA;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PA;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PA;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PA;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PA;
			break;
		}
	}
	
	else if (port == PORTB) // For port B
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PB;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PB;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PB;
			break;
		}
	}
	
	else if (port == PORTC) // For port C
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PC;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PC;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PC;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PC;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PC;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PC;
			break;
		}
	}
	
	else if (port == PORTD) // For port D
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PD;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PD;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PD;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PD;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PD;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PD;
			break;
		}
	}
	
	else if (port == PORTE) // For port E
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PE;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PE;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PE;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PE;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PE;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PE;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PE;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PE;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PE;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PE;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PE;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PE;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PE;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PE;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PE;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PE;
			break;
		}
	}
	
	else if (port == PORTF) // For port F
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PF;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PF;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PF;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PF;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PF;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PF;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PF;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PF;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PF;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PF;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PF;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PF;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PF;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PF;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PF;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PF;
			break;
		}
	}
	
	if (port == PORTG) // For port G
	{
		switch(pinNumber) // Select Pin Number
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PG;
			break;
			
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PG;
			break;
			
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PG;
			break;
			
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PG;
			break;
			
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PG;
			break;
			
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PG;
			break;
			
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PG;
			break;
			
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PG;
			break;
			
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PG;
			break;
			
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PG;
			break;
			
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PG;
			break;
			
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PG;
			break;
			
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PG;
			break;
			
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PG;
			break;
			
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PG;
			break;
			
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PG;
			break;
		}
	}
	
	// Enables interrupt on Rising Edge
	if (edge == RISING_EDGE)
		EXTI->RTSR |= (1 << pinNumber);
	
	// Enables interrupt on Falling Edge
	else if (edge == FALLING_EDGE)
		EXTI->FTSR |= (1 << pinNumber);
	
	// Enables interrupt on both Rising and Falling Edges
	else if (edge == RISING_FALLING_EDGE)
	{
		EXTI->RTSR |= (1 << pinNumber);
		EXTI->FTSR |= (1 << pinNumber);
	}
}

/******************************************************************************************/
/*                            GPIO Interrupt Enable Function                              */
/******************************************************************************************/

void enable_gpio_interrupt (uint32_t pinNumber, IRQn_Type irqNumber)
{
	// Enables interrupt in EXTI
	EXTI->IMR |= (1 << pinNumber);
	
	// Enables interrupt in NVIC
	NVIC_EnableIRQ(irqNumber);
}

/******************************************************************************************/
/*                             GPIO Interrupt Clear Function                              */
/******************************************************************************************/

void clear_gpio_interrupt (uint32_t pinNumber)
{
	// Clear interrupt by setting the relevant bit in the Pending Register to the respective pin number
	EXTI->PR |= (1 << pinNumber);
}

