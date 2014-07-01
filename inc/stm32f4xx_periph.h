/* stm32f4xx_periph.c */
#include "stm32f4xx.h"

static unsigned int BaudRate;
static unsigned int BRR;
//static uint32_t Timer_Frequency = 42000;
static uint32_t prescaler;

void USART1_INIT(uint32_t baudrate);
void USART2_INIT(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);
void DMA1_Init(void);
void DMA2_Init(void);
void LED_Init(void);
void ADC_Init(void);
void EXTI1_Init(void);
void EXTI2_Init(void);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
