/* stm32f4xx_it.c */
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "nmea.h"
#include "main.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

void USART2_IRQHandler(void);
void TIM2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void SysTick_Handler(void);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

uint16_t temp;
extern char usart_buffer[MAX_STRLEN];
static __IO uint32_t TimingDelay;
uint32_t i;

void SysTick_Handler(void)
{
	//SysTick->CTRL = 0x07;
	//TimingDelay--;
    TimingDelay_Decrement();
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void Delay(__IO uint32_t nTime)
{
	TimingDelay = nTime;
	//while((SysTick->CTRL & 0x00010000)==0);
	while(TimingDelay != 0);
}

void delay_ms(uint32_t ms)
{
	uint32_t time;
	uint32_t loop_time;
	time = SystemCoreClock / 	4000;
	for(; ms != 0; ms--)
		{
			loop_time = time;
			for(; loop_time != 0; loop_time--);
		}
}

void USART2_IRQHandler(void)
	{	
		/*
		temp = USART2->SR;
		temp = USART2->DR;
		t = 0;
		if ((USART2 ->	SR &	USART_SR_RXNE) == 0)
		{
			char t = USART2->DR; // the character from the USART1 data register is saved in t
				}
		*/
	}

void TIM2_IRQHandler(void)
{
	if ( TIM2->SR & TIM_SR_UIF )
	{
		TIM2->SR &= ~TIM_SR_UIF;
		GPIOD->ODR	|= GPIO_ODR_ODR_12;
		Delay(30);
		GPIOD->ODR &= ~GPIO_ODR_ODR_12;
		Read_USART();
		Read_ADC();
		Average();
	}
}

void DMA1_Stream5_IRQHandler(void)
{
	if (DMA1 -> HISR & DMA_HISR_HTIF5)
	{
		DMA1 -> HIFCR |= DMA_HIFCR_CHTIF5;	
	}
	if (DMA1 -> HISR & DMA_HISR_TCIF5)
	{
		// clear DMA1 interrupt pending bit
		DMA1 -> HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5;
	}
	NVIC->ICPR[0] |=(1<<7);
}

void DMA2_Stream0_IRQHandler(void)
{
	if (DMA2 -> LISR & DMA_LISR_HTIF0)
	{
		DMA2 -> LIFCR |= DMA_LIFCR_CHTIF0;
	}
	if (DMA2 -> LISR & DMA_LISR_TCIF0)
	{
		// clear DMA2 interrupt pending bit
		DMA2 -> LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
	}
	NVIC->ICPR[0] |=(1<<7);
}
