//misc.c
#include "stm32f4xx.h"
#include "nmea.h"
#include "misc.h"
#include <string.h>

#define MAX_STRLEN 256
#define ARRAYSIZE 128*4
uint16_t adc_buffer[ARRAYSIZE];

float temperature;
float voltage, temp_adc;
uint8_t step;
uint16_t i;

char received_string[MAX_STRLEN+1];
char received_buff[MAX_STRLEN+1];
char received[6];
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
}
