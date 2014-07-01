/* stm32f4xx_it.h */
#include "stm32f4xx.h"

#define MAX_STRLEN 256

extern char received_string[MAX_STRLEN+1];
extern char received_buff[MAX_STRLEN+1];

void USART2_IRQHandler(void);
void TIM2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void SysTick_Hadler(void);
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

