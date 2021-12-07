/*
 * main.c
 * 
 * Seat Belt Interlock and Alarm System for Forklift
 * Microcontroller: STM32F103C8T6
 *
 * Follows AS2359.1 Standard with the use of Handbrake,
 * Neutral, Seat Pressure and Seat Belt sensors.
 *
 * Output of LED3 becomes high when the correct sequence is completed.
 *
 * This output can be used as a signal for the internal motor interlock
 * in the forklift.
 *
 * This system follows the sequence of sitting on the seat prior to engaging 
 * the seat belt while the handbrake is engaged and gear is in neutral position.
 *
 * Version: 1.0 | Ravidu Liyanage
 */

/*******************| HEADERS |*******************/
#include "stm32f10x.h"
#include "HAL_GPIO.h"
#include "usart_debug.h"
#include "timers.h"
/*************************************************/

const int breakTimer = 2; // Time-out in seconds
const int blinkTimer = 35; // Time between two blinks in milliseconds

uint8_t breakFlag = 0;
uint8_t blinkFlag = 0;
uint8_t countFlag = 0;
uint8_t toggle = 0;
uint32_t counter = 0;
uint32_t blinkCounter = 0;

// Initializes Timer2 for time out
void init_timInt()
{
	// Enable Timer clock gating
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// Configure Timer 2 with specific time base
	TIM2->PSC = 1; // Disable the prescalar
	TIM2->ARR = 72; // Divide the system clock by 72 to get 1 MHz
	TIM2->CR1 |= TIM_CR1_URS; // Only counter overflow/underflow generates an update interrupt
	TIM2->DIER |= TIM_DIER_UIE; // Update Interrupt Enable
	TIM2->EGR |= TIM_EGR_UG; // Update Generation
	
	NVIC_EnableIRQ(TIM2_IRQn); // Enable Interrupt Request Handler
}

// Initializes Timer3 for blinking LED
void init_blink()
{
	// Enable Timer clock gating
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Configure Timer 3 with specific time base
	TIM3->PSC = 1; // Disable the prescalar
	TIM3->ARR = 72; // Divide the system clock by 72 to get 1 MHz
	TIM3->CR1 |= TIM_CR1_URS; // Only counter overflow/underflow generates an update interrupt
	TIM3->DIER |= TIM_DIER_UIE; // Update Interrupt Enable
	TIM3->EGR |= TIM_EGR_UG; // Update Generation
	
	NVIC_EnableIRQ(TIM3_IRQn); // Enable Interrupt Request Handler
}

// Enable time out
void enable_breakTimer()
{
	counter = 0; // Reset the Time-out
	countFlag = 1; // Set the Counter Flag
	TIM2->CR1 |= TIM_CR1_CEN; // Enable the counter
}

// Disable time out
void disable_breakTimer()
{
	counter = 0; // Reset the Time-out
	countFlag = 0; // Reset the Counter Flag
	TIM2->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

void enable_blink()
{
	blinkCounter = 0; // Reset the Blink Counter
	blinkFlag = 1; // Set the Blink Flag
	TIM3->CR1 |= TIM_CR1_CEN; // Enable the counter
}

void disable_blink()
{
	blinkCounter = 0; // Reset the Blink Counter
	blinkFlag = 0; // Reset the Blink Flag
	TIM3->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

// Interrupt Request Handler for time out
void TIM2_IRQHandler(void)
{
	counter++;// Increase the count variable at every overflow/underflow
	TIM2->SR &= ~TIM_SR_UIF; // Clear Interrupt Flag
	
	// Disable the Time-out when the required time is reached and Reset the system
	if(counter > (breakTimer*1000000))
	{	
		breakFlag = 1;
		disable_breakTimer();
	}
}

// Interrupt Request Handler for blinking LED
void TIM3_IRQHandler(void)
{
	blinkCounter++;// Increase the count variable at every overflow/underflow
	TIM3->SR &= ~TIM_SR_UIF; // Clear Interrupt Flag
	
	// Toggle LED state when an interrupt is generated
	if(blinkCounter > (blinkTimer*1000))
	{	
		if(toggle)
		{
			toggle = 0;
			blinkCounter = 0;
			GPIOA->BSRR |= (1<<4);
		}
		else
		{
			toggle = 1;
			blinkCounter = 0;
			GPIOA->BSRR |= (1<<(4+16));
		}
	}
}

int main()
{
	init_timInt(); // Initialize break Timer
	init_blink(); // Initialize Blink Timer
	init_SysTick(); // Initialize System timer for delays
	
	// Handbrake Input Setup
	GPIO_TYPE handBrake;
	handBrake.port = PORTB;
	handBrake.pin = 4;
	handBrake.mode = INPUT_MODE;
	handBrake.mode_type = INPUT_FLOATING;
	gpio_init(handBrake);
	
	// Neutral Input Setup
	GPIO_TYPE neutral;
	neutral.port = PORTB;
	neutral.pin = 6;
	neutral.mode = INPUT_MODE;
	neutral.mode_type = INPUT_FLOATING;
	gpio_init(neutral);
	
	// Seat Pressure Input Setup
	GPIO_TYPE seatPressure;
	seatPressure.port = PORTB;
	seatPressure.pin = 7;
	seatPressure.mode = INPUT_MODE;
	seatPressure.mode_type = INPUT_FLOATING;
	gpio_init(seatPressure);
	
	// Seat Belt Input Setup
	GPIO_TYPE seatBelt;
	seatBelt.port = PORTB;
	seatBelt.pin = 8;
	seatBelt.mode = INPUT_MODE;
	seatBelt.mode_type = INPUT_FLOATING;
	gpio_init(seatBelt);
	
	// Horn Output Setup
	GPIO_TYPE horn;
	horn.port = PORTA;
	horn.pin = 0;
	horn.mode = OUTPUT_MODE;
	horn.mode_type = OUTPUT_GEN_PURPOSE;
	horn.speed = SPEED_50MHZ;
	gpio_init(horn);
	
	// LED1 Output Setup
	GPIO_TYPE led1;
	led1.port = PORTA;
	led1.pin = 1;
	led1.mode = OUTPUT_MODE;
	led1.mode_type = OUTPUT_GEN_PURPOSE;
	led1.speed = SPEED_50MHZ;
	gpio_init(led1);
	
	// LED2 Output Setup
	GPIO_TYPE led2;
	led2.port = PORTA;
	led2.pin = 2;
	led2.mode = OUTPUT_MODE;
	led2.mode_type = OUTPUT_GEN_PURPOSE;
	led2.speed = SPEED_50MHZ;
	gpio_init(led2);
	
	// LED3 Output Setup
	GPIO_TYPE led3;
	led3.port = PORTA;
	led3.pin = 3;
	led3.mode = OUTPUT_MODE;
	led3.mode_type = OUTPUT_GEN_PURPOSE;
	led3.speed = SPEED_50MHZ;
	gpio_init(led3);
	
	// Power LED Output Setup
	GPIO_TYPE led;
	led.port = PORTA;
	led.pin = 4;
	led.mode = OUTPUT_MODE;
	led.mode_type = OUTPUT_GEN_PURPOSE;
	led.speed = SPEED_50MHZ;
	gpio_init(led);	
	
	while(1)
	{
		breakFlag = 0;
		
		if(!breakFlag && gpio_read(handBrake) && gpio_read(neutral) && !(gpio_read(seatPressure)) && gpio_read(seatBelt))
		{
			delay(10); // To prevent error pulses from sensors
			
			if(!breakFlag && gpio_read(handBrake) && gpio_read(neutral) && !(gpio_read(seatPressure)) && gpio_read(seatBelt)) // Correct sequence step 1
			{
				
				while(!breakFlag && gpio_read(handBrake) && gpio_read(neutral) && !(gpio_read(seatPressure))) // Check if seated with Handbrake and Neutral On
				{
					// LED 2 ON
					gpio_write(horn, LOW);
					gpio_write(led1, LOW);
					gpio_write(led2, HIGH);
					gpio_write(led3, LOW);
					disable_blink();
					gpio_write(led, HIGH);
					
					if(!breakFlag && !(gpio_read(seatPressure)) && !(gpio_read(seatBelt))) 
					{
						delay(10);
		
						while(!breakFlag && gpio_read(handBrake) && gpio_read(neutral) && !(gpio_read(seatPressure)) && !(gpio_read(seatBelt))) // Correct sequence step 2
						{	
							// LED 3 ON
							gpio_write(horn, LOW);
							gpio_write(led1, LOW);
							gpio_write(led2, LOW);
							gpio_write(led3, HIGH);
							disable_blink();
							gpio_write(led, HIGH);
							
							while(!breakFlag)
							{
								if(!(gpio_read(seatPressure)) && !(gpio_read(seatBelt)))
								{
									// LED 3 ON
									gpio_write(horn, LOW);
									gpio_write(led1, LOW);
									gpio_write(led2, LOW);
									gpio_write(led3, HIGH);
									disable_blink();
									gpio_write(led, HIGH);
									
									if(countFlag)
										disable_breakTimer();
								}
								else if(gpio_read(handBrake) && gpio_read(neutral))
								{
									break;
								}
								else
								{
									// LED 2 and LED 3 ON Power LED blink
									gpio_write(horn, LOW);
									gpio_write(led1, LOW);
									gpio_write(led2, HIGH);
									gpio_write(led3, HIGH);
									if(!blinkFlag)
										enable_blink();
									
									if(!breakFlag && !countFlag)
										enable_breakTimer();
								}
							}
						}
					}
				}
			}
		}
		else if(!(gpio_read(handBrake)) || !(gpio_read(neutral)))
		{
			// Horn ON LED 1 ON Power LED blink
			gpio_write(horn, HIGH);
			gpio_write(led1, HIGH);
			gpio_write(led2, LOW);
			gpio_write(led3, LOW);
			if(!blinkFlag)
				enable_blink();
		}
		else
		{
			// LED 1 ON
			gpio_write(horn, LOW);
			gpio_write(led1, HIGH);
			gpio_write(led2, LOW);
			gpio_write(led3, LOW);
			disable_blink();
			gpio_write(led, HIGH);
		}
	}
}
