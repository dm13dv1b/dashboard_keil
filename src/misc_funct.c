//misc.c
#include "stm32f4xx.h"
#include "stm32f4xx_periph.h"
#include "nmea.h"
#include "main.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>
#include "misc_funct.h"

#define MAX_STRLEN 256
#define MAX_STRLEN2 12
#define ARRAYSIZE 128*4
uint16_t adc_buffer[ARRAYSIZE];

uint8_t send_voltage, send_temperature, send_gps;

float temperature;
float voltage, temp_adc;
uint8_t step;
uint16_t i;

char USART1_string[12];
char received_buff[MAX_STRLEN+1];
char received[6];
char my_string[8];

extern char usart_buffer[MAX_STRLEN];
extern char * pch;
extern char *p;
uint16_t i;

void Read_USART(void)
{
	memcpy(received_buff, usart_buffer, MAX_STRLEN);
	p = (char *)received_buff;
	pch = strchr(p, '$');
	i = pch-p+1;														//  megkeressük az elsö csillagot
	pch = strchr(pch+1, '$');
	if (send_gps == 1)
	{
		USART_puts(USART1, received_buff);
	}
	NMEA_Parse(received_buff, MAX_STRLEN);
}

void Read_ADC(void)
{
	ADC1->CR2 &=~ADC_CR2_DMA;
	ADC1->CR2 |=ADC_CR2_DMA;
	ADC1->SR &=~ADC_SR_STRT;
	ADC1->SR &=~ADC_SR_OVR;
	ADC1->SR &=~ADC_SR_EOC;
	ADC1-> CR2 |=ADC_CR2_SWSTART;	//Start conversion of regular channels
	while (DMA1->LISR & DMA_LIFCR_CHTIF0){};
	ADC1-> CR2 &= ~ADC_CR2_SWSTART;
}

void Average(void)
{
	if (adc_buffer[0] > 1500)
	{ step = 0; } else step = 1;
	temp_adc = 0;
	for ( i=step; i < ARRAYSIZE; i+=2)
		{
			temp_adc += adc_buffer[i];
		}
	temp_adc /= ARRAYSIZE/2;
	temp_adc -= 760;
	temp_adc /= 2500;
	temp_adc +=25;

	temperature = temp_adc;
	if (send_temperature == 1)
	{
		USART_puts(USART1, "Internal temperature: ");
		sprintf(my_string, "%f", temperature);
		USART_puts(USART1, my_string);
		USART_puts(USART1, "\n\r");		
	}
	temp_adc = 0;

	for ( i=step+1; i < ARRAYSIZE; i+=2)
		{
			temp_adc += adc_buffer[i];
		}
	temp_adc /= ARRAYSIZE/2;
	temp_adc *= 3300;
	temp_adc /= 0xFFF;
	temp_adc /= 1000;
	temp_adc *= 2;
	voltage = temp_adc;
	if (send_voltage == 1)
	{
		USART_puts(USART1, "Internal voltage: ");
		sprintf(my_string, "%f", voltage);
		USART_puts(USART1, my_string);
		USART_puts(USART1, "\n\r");
	}
	
}
