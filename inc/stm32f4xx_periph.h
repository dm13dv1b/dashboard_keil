/* stm32f4xx_periph.c */
#include "stm32f4xx.h"

unsigned int BaudRate;
unsigned int BRR;
//static uint32_t Timer_Frequency = 42000;
uint32_t prescaler;

void USART1_INIT(void);
void USART2_INIT(void);
void TIM2_Init(void);
void DMA1_Init(void);
void DMA2_Init(void);
void LED_Init(void);
void ADC_Init(void);
