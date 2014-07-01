#include "stm32f4xx.h"
#include "nmea.h"
#include <string.h>
#include <stdlib.h>

struct gpsdata_t gpsdata;
static uint16_t chk_ok;
char *pch;
char *p;
uint32_t tmp = 0;

void TextOut(char str)											//ITM Stimulus port 0 -> 1-be
{
	do {
		if(str=='\n') ITM_SendChar('\r');
		ITM_SendChar(str);														//ITM Send char printf helyett
	} while (str++);
}

static uint32_t NMEA_atoi(char *p)
{
	uint32_t out = 0;

	while ((*p >= '0' && *p <= '9') || *p == '.')
	{
		if (*p == '.') {
			p++;
			continue;
		}
		out *= 10;
		out += *p - '0';
		p++;
	}

	return out;
}

void NMEA_Parse(char *buf, uint16_t len)
{
	p = pch;

	if (!strncmp(pch, "$GP",3 ))
	{
		chk_ok = 1;
	}
		
	if(!strncmp(p, "$GPGGA", 6))
	{	
		p += 7;
		// perse time
		gpsdata.hour = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.min = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.sec = (p[4] - '0') * 10 + p[5] - '0';
		// parse time		
		
		p = strstr(p, ",") + 1;
		// parse lat
		tmp = NMEA_atoi(p);
		p = strstr(p, ",") + 1;
		if (p[0] == 'S')
			tmp = -tmp;
		gpsdata.lat = tmp;

		p = strstr(p, ",")+1;
		// parse lon
		tmp = NMEA_atoi(p);
		p=strstr(p, ",") + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		gpsdata.lon = tmp;

		p = strstr(p, ",") + 1;
		gpsdata.valid = (p[0] - '0')?1:0;
		// ez nem tudom mi
		
		p = strstr(p, ",") + 1;
		//gpsdata.alt = NMEA_atoi(p);
		//gpsdata.sats = (p[0] - '0') * 10 + p[1] - '0';

		p = strstr(p, ",") + 1;
		// HDOP
		gpsdata.hdop = NMEA_atoi(p);

		p = strstr(p, ",") + 1;
		// alt
		gpsdata.alt = NMEA_atoi(p);
		GPIOD->ODR ^= GPIO_ODR_ODR_13;
 }
	
	if(!strncmp(p, "$GPRMC", 6))
		{
		p += 7;
		gpsdata.hour = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.min = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.sec = (p[4] - '0') * 10 + p[5] - '0';

		p = strstr(p, ",") + 1;
		gpsdata.rmc_valid = (p[0] == 'A')?1:0;

		p = strstr(p, ",") + 1;
		// parse lat
		tmp = NMEA_atoi(p);
		p = strstr(p, ",") + 1;
		if (p[0] == 'S')
			tmp = -tmp;
		gpsdata.lat = tmp;

		p = strstr(p, ",")+1;
		// parse lon
		tmp = NMEA_atoi(p);
		p=strstr(p, ",") + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		gpsdata.lon = tmp;

		p = strstr(p, ",") + 1;
		// speed
		gpsdata.speed = NMEA_atoi(p);

		p = strstr(p, ",") + 1;
		// heading
		gpsdata.heading = NMEA_atoi(p);

		p = strstr(p, ",") + 1;
		// date
		gpsdata.day = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.month = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.year = (p[4] - '0') * 10 + p[5] - '0';
		GPIOD->ODR ^= GPIO_ODR_ODR_14;
 }
		
	if(!strncmp(p+3, "GSV", 3)) {	
		p += 11;
		gpsdata.sats = (p[0] - '0') * 10 + p[1] - '0';
		GPIOD->ODR ^= GPIO_ODR_ODR_15;
	}
	if (gpsdata.rmc_valid && gpsdata.valid == 1)
	{
		//GPIOD->ODR |= GPIO_ODR_ODR_15;
	}
	else
	{
		//GPIOD->ODR &= ~GPIO_ODR_ODR_15;
	}
}
