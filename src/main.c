#include "main.h"
#include "stm32f4xx_periph.h"
#include "stm32f4xx_it.h"
#include "nmea.h"
#include "stm32f4xx_rcc.h"
#include <string.h>
#include <stdio.h>

#define ARRAYSIZE 128*4

uint16_t adc_buffer[ARRAYSIZE];

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

	ADC_Init();
	DMA1_Init();
	DMA2_Init();
	TIM2_Init();
	USART2_INIT();
	//delay_ms(1000);
	
	while(1)
	{
	}
}
