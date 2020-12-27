/**
 * @file UserCtrl.h
 * @brief 用户控制程序
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020  WHU-EIS
 * 
 */
#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "sys.h"

void Ctrl_Task(u32 dT_ms);
void Set_Vel(float linear_x, float linear_y, float angular_z);
#endif             
