#include "main.h"
#include "stm32f4xx_periph.h"
#include "stm32f4xx_it.h"
#include "nmea.h"
#include "stm32f4xx_init_rcc.h"
#include <stm32f4xx_usart.h> 
#include <string.h>
#include <stdio.h>

#define ARRAYSIZE 128*4

void TextOut(const char *str);

void TextOut(const char *str)											//ITM Stimulus port 0 -> 1-be
{
	do {
		if(*str=='\n') ITM_SendChar('\r');
		ITM_SendChar(*str);														//ITM Send char printf helyett
	} while (*str++);
}

int main(void)
{
	 if (SysTick_Config(SystemCoreClock / 1000))
	  {
	    /* Capture error */
	    while (1);
	  }
	SysTick_Config(SystemCoreClock/1000);
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_EnableIRQ(SysTick_IRQn);
	RCC_Init();
	LED_Init();
	GPIOD->ODR	|= GPIO_ODR_ODR_12;
	GPIOD->ODR	|= GPIO_ODR_ODR_13;
	GPIOD->ODR	|= GPIO_ODR_ODR_14;
	GPIOD->ODR	|= GPIO_ODR_ODR_15;
	Delay(1000);
	GPIOD->ODR	^= GPIO_ODR_ODR_12;
	GPIOD->ODR	^= GPIO_ODR_ODR_13;
	GPIOD->ODR	^= GPIO_ODR_ODR_14;
	GPIOD->ODR	^= GPIO_ODR_ODR_15;

	TextOut("Start init...n\r");
	ADC_Init();
	DMA1_Init();
	DMA2_Init();
	TIM2_Init();
	TIM3_Init();
	TIM4_Init();
	EXTI1_Init();
	EXTI2_Init();
	//USART1_INIT(38400);
	USART2_INIT();
	TextOut("End init...\n\r");
	
	while(1)
	{
	}
}
