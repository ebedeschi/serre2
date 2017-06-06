/*
 * Vbat.h
 *
 *  Created on: 30 apr 2017
 *      Author: Emanuele
 */

#ifndef VBAT_VBAT_H_
#define VBAT_VBAT_H_

#include "stm32l4xx_hal.h"

uint16_t getVbat(ADC_HandleTypeDef hadc);

#endif /* VBAT_VBAT_H_ */
