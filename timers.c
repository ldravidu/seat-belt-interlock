/*
 * timers.c
 * 
 * Timer Control Library
 * for STM32F10x Microcontroller Family
 *
 * Used to control Timers and Counters
 * Delay functions with General Purpose Timer4 and SysTick 
 *
 * Version: 1.1 | Ravidu Liyanage
 */
 
#include "timers.h"

/********************** Global Variables **********************/
long int ticks = 0;
uint8_t SysTick_initFlag = 0;
uint8_t tim2_en = 0, tim3_en = 0, tim4_en = 0;

/************************* Functions **************************/
void init_SysTick(void)
{
	SysTick_initFlag = 1;
	
	SysTick_Config(SystemCoreClock / 1000);
}

void init_timer(void)
{
	// Enable Timer clock gating 
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// Configure Timer 4 with specific time base
	TIM4->PSC = 1; // Disable the prescalar
	TIM4->ARR = 72; // Divide the system clock by 72 to get 1 MHz
	TIM4->CR1 |= TIM_CR1_URS; // Only counter overflow/underflow generates an update interrupt
	TIM4->DIER |= TIM_DIER_UIE; // Update Interrupt Enable
	TIM4->EGR |= TIM_EGR_UG; // Update Generation
	NVIC_EnableIRQ(TIM4_IRQn);
	tim4_en = 1;
}

void delay(int ms)
{
	if(SysTick_initFlag)
		init_SysTick();
	
	ticks = 0;
	
	while(ticks < ms);
}

void SysTick_Handler(void)
{
	ticks++;
}

void TIM4_IRQHandler(void)
{
	ticks++; // Increase the count variable at every overflow/underflow
	TIM4->SR &= ~TIM_SR_UIF; // Clear Interrupt Flag
}

void delay_us(int us)
{
	if(!tim4_en)
	{
		init_timer();
	}
	
	TIM4->CR1 |= TIM_CR1_CEN; // Enable the counter
	
	ticks = 0; // Reset the count variable
	
	while(ticks < us); // Stuck in a while loop until the count becomes equal to us
	
	TIM4->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

void delay_ms(int ms)
{
	if(!tim4_en)
	{
		init_timer();
	}
	
	TIM4->CR1 |= TIM_CR1_CEN; // Enable the counter
	
	ticks = 0; // Reset the count variable
	
	while(ticks < ms*1000); // Stuck in a while loop until the count becomes equal to us
	
	TIM4->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

