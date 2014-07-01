/* stm32f4xx_periph.c */
#include "stm32f4xx.h"
#include "stm32f4xx_periph.h"
#define MAX_STRLEN 256
uint32_t Timer_Frequency = 42000;
//uint32_t prescaler;
char usart_buffer[MAX_STRLEN];
#define ARRAYSIZE 128*4
uint16_t adc_buffer[ARRAYSIZE];

void USART2_INIT(void);
void TIM2_Init(void);
void DMA1_Init(void);
void DMA2_Init(void);
void ADC_Init(void);
void LED_Init(void);

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
		GPIOD ->	OSPEEDR		&=	~(3<<28);						//50Mhz fast speed
		GPIOD ->	OSPEEDR		|=	(2<<28);
		GPIOD ->	PUPDR			&=	~(3<<28);						//PD14 pull up
		GPIOD	->	PUPDR			|=	(1<<28);
	//LED init PD14

	//LED init PD14
		GPIOD	->	MODER			|=	(1<<30);						//PD14 = output
		GPIOD ->	OTYPER		&=	(1<<15);						//Output Push/pull
		GPIOD ->	OSPEEDR		|=	(2<<30);						//50Mhz fast speed
		GPIOD ->	PUPDR			|=	(1<<30);						//PD14 pull up
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
	TIM2	->	PSC			=		Timer_Frequency-1;
	TIM2	->	ARR			=		prescaler;
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0x04);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
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
