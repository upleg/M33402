/**
  ******************************************************************************
  * @file           : 7seg_led.c
  * @brief          : Библиотека работы с 3-х знаковым 7-ми сегментным индикатором
  ******************************************************************************
  * Date:            10.12.2019
  * Auhtor:          Oleg Borisenko
  ******************************************************************************	 
  */

#include "7seg_led.h"
#include <math.h>

uint8_t R1 = 0;
uint8_t R2 = 0;
uint8_t R3 = 0;

uint16_t int_value = 0;
uint8_t dp_position = 0;


/*	Округление до 3-х знаков и преобразование значения	переменной <float>[value] в формат 
 *	целого <uint16_t>[int_value] с определением позиции запятой <uint8_t>[dp_position]
 **/
void led_print(float value) {

	int_value = 0;
	dp_position = 0;

	if (value == 0) {
		int_value = 0;
		dp_position = 0;

	}
	else if (value > 0 && value < 10) {

		int_value = round(value * 100);
		dp_position = 1;

		if (int_value % 10 == 0) {

			int_value = int_value / 10;
			dp_position++;
		}
	}
	else if (value >= 10 && value < 100) {

		int_value = round(value * 10);
		dp_position = 2;

	}
	else if (value >= 100 && value < 1000) {

		int_value = round(value);
		dp_position = 0;

	}
	
	R1 = int_value % 10;
	R2 = int_value % 100 / 10;
	R3 = int_value / 100;
}


void seg_print(uint8_t seg)
{
	switch (seg)
	{
	case 1:
		SA_RESET; SB_SET; SC_SET; SD_RESET; SE_RESET; SF_RESET; SG_RESET;
		break;
		
	case 2:
		SA_SET; SB_SET; SC_RESET; SD_SET; SE_SET; SF_RESET; SG_SET;
		break;
		
	case 3:
		SA_SET; SB_SET; SC_SET; SD_SET; SE_RESET; SF_RESET; SG_SET;
		break;
		
	case 4:
		SA_RESET; SB_SET; SC_SET; SD_RESET; SE_RESET; SF_SET; SG_SET;
		break;
		
	case 5:
		SA_SET; SB_RESET; SC_SET; SD_SET; SE_RESET; SF_SET; SG_SET;
		break;
		
	case 6:
		SA_SET; SB_RESET; SC_SET; SD_SET; SE_SET; SF_SET; SG_SET;
		break;
		
	case 7:
		SA_SET; SB_SET; SC_SET; SD_RESET; SE_RESET; SF_RESET; SG_RESET;
		break;
		
	case 8:
		SA_SET; SB_SET; SC_SET; SD_SET; SE_SET; SF_SET; SG_SET;
		break;
		
	case 9:
		SA_SET; SB_SET; SC_SET; SD_SET; SE_RESET; SF_SET; SG_SET;
		break;
		
	case 0:
		SA_SET; SB_SET; SC_SET; SD_SET; SE_SET; SF_SET; SG_RESET;
		break;
	}
}
