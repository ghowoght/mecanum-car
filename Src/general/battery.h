#ifndef _BATTERY_H
#define _BATTERY_H

#include <stdint.h>
#include "stm32g4xx_hal.h"


extern ADC_HandleTypeDef hadc2;

void Battery_Init();
void Battery_Task(uint32_t dT_ms);

#endif