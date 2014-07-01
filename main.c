#include	"system_stm32f4xx.c"
#include "main.h"
#include "stm32f4xx_periph.h"
#include "stm32f4xx_it.h"
#include "nmea.h"
#include "stm32f4xx_rcc.h"
#include <string.h>
#include <stdio.h>

char received_string[MAX_STRLEN+1];
char received_buff[MAX_STRLEN+1];
char received[6];
extern char usart_buffer[MAX_STRLEN];
extern char * pch;
extern char *p;
uint16_t i;

void delay_ms(uint32_t ms)
{
uint32_t time = SystemCoreClock / 4000;
uint32_t loop_time;

  for(; ms != 0; ms--)
  {
    loop_time = time;
    for(; loop_time != 0; loop_time--);
  }
}

void Read_USART(void)
{
	memcpy(received_buff, usart_buffer, MAX_STRLEN);
	p = (char *)received_buff;
	pch = strchr(p, '$');
	i = pch-p+1;														//  megkeressük az elsö csillagot
	pch = strchr(pch+1, '$');
	NMEA_Parse(received_buff, MAX_STRLEN);
}

int main(void)
{
	RCC_Init();
	DMA1_Init();
	TIM2_Init();
	USART2_INIT();
	delay_ms(1000);
	
	while(1)
	{
	}
}
