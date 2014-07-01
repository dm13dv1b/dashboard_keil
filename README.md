dashboard_keil
==============

Aprilia Pegaso secondary dashboard in Keil UVision

Board: STM32F4Discovery (STM32F407VG)

USART1 APB2 SYSCLK/2 38400 added debug console // PB6 USART 1 TX
						// PB7 USART 1 RX
	                       
USART2 APB1 SYSCLE/4 GPS 38400            //PA2 USART 2 TX
					//PA3 USART 2 RX
                            
DMA1 STREAM 5 USART2 buffer
DMA2 STREAM 0 ADC buffer, internal temperature meter, on board volatge meter
ADC1 in continous mode, scanning, into DMA on channel 16 & 18
TIM2 set for 1 sec, toggle PD12 for 30ms indicating  ADC and USART GPS reading
SYSTICK for 10ms interrupt

LEDs PD12, PD13, PD14, PD15

NMEA parsing in nmea.c

EXTI1, EXTI2 added to PB1 PB2 without timer

Added ITM_SendChar debug
