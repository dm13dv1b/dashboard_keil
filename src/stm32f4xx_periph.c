/* stm32f4xx_periph.c */
#include "stm32f4xx.h"
#include "stm32f4xx_periph.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

#define MAX_STRLEN 256
uint32_t Timer_Frequency = 42000;
//uint32_t prescaler;
char usart_buffer[MAX_STRLEN];
#define ARRAYSIZE 128*4
extern uint16_t adc_buffer[ARRAYSIZE];

void USART2_INIT(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);
void DMA1_Init(void);
void DMA2_Init(void);
void ADC_Init(void);
void LED_Init(void);
void USART1_Init(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void EXTI1_Init(void);
void EXTI2_Init(void);

void EXTI1_Init(void)
{
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; //EXTI 1 PB 1
	EXTI->IMR |= EXTI_IMR_MR1;		//Enable interrupt mask
	EXTI->FTSR |= EXTI_FTSR_TR1;  //Falling trigger selection
	NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI2_Init(void)
{
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB; //EXTI 2 PB 2
	EXTI->IMR |= EXTI_IMR_MR2;
	EXTI->FTSR |= EXTI_FTSR_TR2;
	NVIC_EnableIRQ(EXTI1_IRQn);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USART1_INIT(uint32_t baudrate){

	// PA9 USART 1 TX
	// PA10 USART 1 RX

	GPIOA	->	MODER 		|=	GPIO_MODER_MODER9_1;		// RX PD6 to alternate function output push-pull at 50 MHz 0x10
	GPIOA	->	MODER 		|=	GPIO_MODER_MODER10_1;		// TX PD5 to alternate function output push-pull at 50 MHz 0x10
	GPIOA ->	OSPEEDR		|=  GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR9_1;		//50Mhz fast speed
	GPIOA ->	OSPEEDR		|=	GPIO_OSPEEDER_OSPEEDR10_0 | GPIO_OSPEEDER_OSPEEDR10_1;
	GPIOA ->	PUPDR			|=  GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;
	GPIOA	->	AFR[1]		|=	(7<<4);
	GPIOA	->	AFR[1]		|=	(7<<8);

	BRR = (SystemCoreClock/4) / (BaudRate*16);
	USART1 ->	BRR = (68 << 4 ) + 0x06; //38400br 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	NVIC_SetPriority(USART1_IRQn, 2);
	NVIC_EnableIRQ(USART1_IRQn);
	USART1 -> CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

void LED_Init(void)
{
	//LED init PD12
		GPIOD	->	MODER			|=	(1<<24);						//PD12 = output
		GPIOD ->	OTYPER		&=	(1<<12);						//Output Push/pull
		GPIOD ->	OSPEEDR		&=	~(3<<24);						//50Mhz fast speed
		GPIOD ->	OSPEEDR		|=	(2<<24);
		GPIOD ->	PUPDR			&=	~(3<<24);						//PD12 pull up
		GPIOD	->	PUPDR			|=	(1<<24);
	//LED init PD12

	//LED init PD13
		GPIOD	->	MODER			|=	(1<<26);						//PD13 = output
		GPIOD ->	OTYPER		&=	(1<<13);						//Output Push/pull
		GPIOD ->	OSPEEDR		&=	~(3<<26);						//50Mhz fast speed
		GPIOD ->	OSPEEDR		|=	(2<<26);
		GPIOD ->	PUPDR			&=	~(3<<26);						//PD13 pull up
		GPIOD	->	PUPDR			|=	(1<<26);
	//LED init PD13

	//LED init PD14
		GPIOD	->	MODER			|=	(1<<28);						//PD14 = output
		GPIOD ->	OTYPER		&=	(1<<14);						//Output Push/pull
		GPIOD ->	OSPEEDR		|=	GPIO_OSPEEDER_OSPEEDR14_1;
		GPIOD ->	PUPDR			|=	GPIO_PUPDR_PUPDR14_0;
	//LED init PD14

	//LED init PD15
		GPIOD	->	MODER			|=	(1<<30);						//PD15 = output
		GPIOD ->	OTYPER		&=	(1<<15);						//Output Push/pull
		GPIOD ->	OSPEEDR		|=	GPIO_OSPEEDER_OSPEEDR15_1;						//50Mhz fast speed
		GPIOD ->	PUPDR			|=	GPIO_PUPDR_PUPDR15_0;						//PD14 pull up
	//LED init PD14
}

void ADC_Init(void)
{
	ADC1-> CR1 |=ADC_CR1_SCAN;		// Enable channel scan
	ADC1-> CR2 |=ADC_CR2_CONT;		//continous
	ADC1-> SQR1 |=(1<<20);				// 0001: 2 conversion
	ADC1-> SQR3	|=(16<<0);				// Channel 16 first sequence
	ADC1-> SQR3	|=(18<<5);				// Channel 18 second sequence
	ADC1-> CR2 |=ADC_CR2_ADON;		// Enable ADC1
	ADC1->CR2 &=~ADC_CR2_DMA;
	ADC1->CR2 |=ADC_CR2_DMA;
	ADC1->SMPR1 |= ADC_SMPR1_SMP16_1 | ADC_SMPR1_SMP16_2;	// sampling time 144 cycles
	ADC1->SMPR1 |= ADC_SMPR1_SMP18_1 | ADC_SMPR1_SMP18_2;	// sampling time 144 cycles
	ADC -> CCR |=ADC_CCR_TSVREFE;	// Enable TSVREFE
	ADC -> CCR |=ADC_CCR_VBATE;		// Enable VBATE
	ADC1 -> DR = 0;
}

void TIM2_Init(void)
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
	prescaler = SystemCoreClock/2;
	prescaler /=Timer_Frequency;
	prescaler -=1;
	TIM2	->	PSC			=		(Timer_Frequency-1);
	TIM2	->	ARR			=		prescaler;
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0x01);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_Init(void)
{
	TIM3->PSC = 1999;
  TIM3->ARR = 41999;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM4_Init(void)
{
	TIM4->PSC = 1999;
  TIM4->ARR = 41999;
	TIM4->CR1 |= TIM_CR1_CEN;
}

void USART2_INIT(void)
{
	BaudRate = 38400;
	GPIOA	->	MODER 		|=	GPIO_MODER_MODER3_1;		// RX PD6 to alternate function output push-pull at 50 MHz 0x10
	GPIOA	->	MODER 		|=	GPIO_MODER_MODER2_1;		// TX PD5 to alternate function output push-pull at 50 MHz 0x10
	GPIOA ->	OSPEEDR		|=  GPIO_OSPEEDER_OSPEEDR3_0 | GPIO_OSPEEDER_OSPEEDR3_1;		//50Mhz fast speed
	GPIOA ->	OSPEEDR		|=	GPIO_OSPEEDER_OSPEEDR2_0 | GPIO_OSPEEDER_OSPEEDR2_1;
	GPIOA ->	PUPDR			|=  GPIO_PUPDR_PUPDR2_0;
	GPIOA	->	AFR[0]		|=	(7<<8);
	GPIOA	->	AFR[0]		|=	(7<<12);
	BRR = (SystemCoreClock/4) / (BaudRate*16);
	USART2 ->	BRR = (68 << 4 ) + 0x06; //38400br
	USART2 -> CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART2 -> CR3 |= USART_CR3_DMAR;
}

void DMA1_Init(void)
{
	DMA1_Stream5 -> CR &= ~DMA_SxCR_EN;					// disable Stream6
	DMA1_Stream5 -> PAR = (uint32_t)&USART2->DR;							// from periph port register
	DMA1_Stream5 -> M0AR = (uint32_t) &usart_buffer;					// write to memory
	DMA1_Stream5 -> NDTR = 256;									// number of data items
	DMA1_Stream5 -> CR |= DMA_SxCR_MINC;				// memory increment
	DMA1_Stream5 -> CR |= DMA_SxCR_CHSEL_2;		// select chanel0
	DMA1_Stream5 -> CR &= ~DMA_SxCR_MSIZE_0;			// half-word (16bit) memory data size
	DMA1_Stream5 -> CR &= ~DMA_SxCR_PSIZE_0;			// half-word (16bit) peripherial data size
	DMA1_Stream5 -> CR |= DMA_SxCR_TCIE;				// transaction complete interrupt
	DMA1_Stream5 -> CR |= DMA_SxCR_CIRC;				// non enable continous mode
	DMA1_Stream5 -> CR |= DMA_SxCR_EN;					// enable Stream6
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);					// enable IRQ	
}

void DMA2_Init(void)
{
	DMA2_Stream0 -> CR &= ~DMA_SxCR_EN;					// disable Stream0
	DMA2_Stream0 -> PAR = (uint32_t)&ADC1->DR;							// from periph port register
	DMA2_Stream0 -> M0AR = (uint32_t) &adc_buffer;					// write to memory
	DMA2_Stream0 -> NDTR = ARRAYSIZE;						// number of data items
	DMA2_Stream0 -> CR |= DMA_SxCR_MINC;				// memory increment
	DMA2_Stream0 -> CR &= ~(DMA_SxCR_CHSEL);		// select chanel0
	DMA2_Stream0 -> CR |= DMA_SxCR_MSIZE_0;			// half-word (16bit) memory data size
	DMA2_Stream0 -> CR |= DMA_SxCR_PSIZE_0;			// half-word (16bit) peripherial data size
	DMA2_Stream0 -> CR |= DMA_SxCR_TCIE;				// transaction complete interrupt
	DMA2_Stream0 -> CR |= DMA_SxCR_CIRC;				// non enable continous mode
	DMA2_Stream0 -> CR |= DMA_SxCR_EN;					// enable Stream0
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);					// enable IRQ
}
