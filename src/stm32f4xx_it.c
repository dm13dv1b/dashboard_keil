/* stm32f4xx_it.c */
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_periph.h"
#include "nmea.h"
#include "main.h"
#include "misc_funct.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>

void USART2_IRQHandler(void);
void TIM2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void SysTick_Handler(void);
void EXTI1_IRQHandler(void)__irq;
void EXTI2_IRQHandler(void)__irq;
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

uint16_t temp;
extern char usart_buffer[MAX_STRLEN];
static __IO uint32_t TimingDelay;
//static uint32_t i;

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

void USART1_IRQHandler(void){
	
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t
		
		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
		if( (t != '\n') && (cnt < MAX_STRLEN) ){ 
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			//USART_puts(USART2, received_string);
		}
	}
}
	
void TIM2_IRQHandler(void)
{
	if ( TIM2->SR & TIM_SR_UIF )
	{
		TIM2->SR &= ~TIM_SR_UIF;
		GPIOD->ODR	|= GPIO_ODR_ODR_12;
		Delay(30);
		//USART_puts(USART1, "Sampling\r\n");
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

void EXTI1_IRQHandler(void)__irq //Kezeli a PA1 lábra kapcsolt számlálást
	{
		//TIM2	->	CR1				^=	(1<<0);							//Timer Disable
		NVIC->ICER[0] 			&= ~(1<<7);							//Disable this interrupt during processing
			//Itt fogsz azt csinálni amit szeretnél
		//EXTI1_counter = TIM2->CNT;													//STM Studio debughoz
		//EXTI1_clock = ((SystemCoreClock/2)/TIM2->CNT);
		//TIM2	->	CNT = 0;
		//TIM2	->	SR				 =	0;													//Clear UIF bit
		//TIM2	->	CR1				|=	(1<<0);								//Timer Enable
		EXTI->PR 	|=(1<<1);													//Pending request clear
		NVIC->ICPR[0] 			|= (1<<7);								//Clear the Pending Interrupt
		NVIC->ISER[0] |= (1 << 7) | (1 << 8);										//Reenable the EXTI1 Interrupt
	}

void EXTI2_IRQHandler(void)__irq //Kezeli a PA2 lábra kapcsolt számlálást
	{
		//TIM5	->	CR1				^=	(1<<0);							//Timer Disable
		NVIC->ICER[0] 			&= ~(1<<8);							//Disable this interrupt during processing
			//Itt fogsz azt csinálni amit szeretnél
		//EXTI2_counter = TIM5->CNT;													//STM Studio debughoz
		//EXTI2_clock = ((SystemCoreClock/2)/TIM5->CNT);
		//TIM5	->	CNT = 0;
		//TIM5	->	SR		=	0;													//Clear UIF bit
		//TIM5	->	CR1				|=	(1<<0);								//Timer Enable
		EXTI->PR 	|= (1<<3);													//Pending request clear
		NVIC->ICPR[0] 			|= (1<<8);								//Clear the Pending Interrupt
		NVIC->ISER[0] |= (1 << 8) | (1 << 7);										//Reenable the EXTI2 Interrupt
	}
