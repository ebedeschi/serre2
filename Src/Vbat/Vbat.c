/*
 * Vbat.c
 *
 *  Created on: 30 apr 2017
 *      Author: Emanuele
 */

#include "Vbat.h"

uint16_t getVbat(ADC_HandleTypeDef hadc)
{
	uint32_t g_ADCValue = 0;
	//double con = 0.00118359375;
	double con = 0.0008056640625;
  //  double con = 0.0023671875;
	double v_adc = 0;
	double v_bat = 0;
	double offset = 0.053979492187499999;
	//double offset = 0.063979492187499999;

	HAL_GPIO_WritePin(ADCEN_GPIO_Port, ADCEN_Pin, GPIO_PIN_SET);
	HAL_Delay(10); //delay

	HAL_ADC_Start(&hadc);

	if (HAL_ADC_PollForConversion(&hadc, 1000000) == HAL_OK)
	{
		g_ADCValue = HAL_ADC_GetValue(&hadc);
	}

	v_adc = con * g_ADCValue + offset;
	v_bat = v_adc * 4;

	HAL_GPIO_WritePin(ADCEN_GPIO_Port, ADCEN_Pin, GPIO_PIN_RESET);

	return v_bat * 100;
}

