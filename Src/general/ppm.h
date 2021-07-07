/**
 * @file ppm.h
 * @brief 
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2021-01-02
 * 
 * @copyright Copyright (c) 2021  WHU-EIS
 * 
 */
#ifndef __PPM_H_
#define __PPM_H_

//#include "sys.h"

#include "stm32g4xx_hal.h"

extern int CH[8];

void PPM_Decode(void);

void PPM_Cal(int pulseHigh);

void RemoteCtrl_Task(uint32_t dT_ms);

#endif

