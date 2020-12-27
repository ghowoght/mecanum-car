#ifndef __LED_H
#define __LED_H

#include "sys.h"

void Drv_LED_Init(void);
void LED_1ms_DRV(void );
void LED_Task(u8 dT_ms);
void LED_Task2(u8 dT_ms);

#endif
