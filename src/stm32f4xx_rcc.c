#include "stm32f4xx.h"

void RCC_Init(void);

void RCC_Init(void)
{
	RCC 	->	APB1ENR |=  RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN;
	RCC		->	AHB1ENR	|=	RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN; 		// Enable PORTD
	RCC 	->	APB2ENR |=	RCC_APB2ENR_ADC1EN;				// Enable ADC1 clock
	RCC 	->	AHB1ENR |=  RCC_AHB1ENR_DMA1EN;				// Enable DMA1 clock
	RCC 	-> 	AHB1ENR |= 	RCC_AHB1ENR_DMA2EN;				// Enable DMA1 clock
}
