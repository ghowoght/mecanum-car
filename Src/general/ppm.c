/**
 * @file ppm.c
 * @brief PPM遥控数据解码
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2021-01-02
 * 
 * @copyright Copyright (c) 2021  WHU-EIS
 * 
 */
#include "ppm.h"
#include "data.h"
#include "Scheduler.h"
#include "motor.h"
#include "UserCtrl.h"

extern TIM_HandleTypeDef htim16;

#define PULSE_MIN   800
#define PULSE_MAX   2200

void PPM_Decode(void)
{
	static int periodVal[2] = {0};
	int pulseHigh = 0;
	
	periodVal[0] = TIM16->CCR1;
	if(periodVal[0] > periodVal[1])
	{
		pulseHigh = (periodVal[0] - periodVal[1]);
	}
	else
	{
		pulseHigh = (periodVal[0] - periodVal[1] + 0xFFFF);
	}
	periodVal[1] = periodVal[0];
	PPM_Cal(pulseHigh);
	
}

int CH[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int CH_Ori[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int16_t deadzone = 50;

float ppm_deadzone(float x,float ref,float zoom)
{
	float t;
	if(x>ref)
	{
		t = x - zoom;
		if(t<ref)
		{
			t = ref;
		}
	}
	else
	{
		t = x + zoom;
		if(t>ref)
		{
			t = ref;
		}
	}
  return (t);
}

void PPM_Cal(int pulseHigh)
{
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
	static uint8_t CNT = 0;
	if(pulseHigh > 5000) //说明一帧结束
	{
		CNT = 0;
		
		for(int i = 0; i < 16; i++)
		{
			CH[i] = ppm_deadzone(CH_Ori[i], 1500, deadzone);
		}
		flag.remote_ctrl_timestamp = GetSysRunTimeUs() / 1000;
		
	}
	else
	{
		
		
		if(pulseHigh >PULSE_MIN && pulseHigh < PULSE_MAX)
		{
			
			if(CNT < 16)
			{
				CNT = CNT > 7 ? 7 : CNT;
				CH_Ori[CNT++] = pulseHigh;
			}
		}
		
	}
}

void RemoteCtrl_Task(uint32_t dT_ms)
{
	// 连接未超时
	if(GetSysRunTimeUs() / 1000 - flag.remote_ctrl_timestamp < 1000)
	{
		if(CH[6] > 1900) // 遥控模式
		{
			flag.robot_sta = MODE_REMOTE_CTRL;
			const float maxVel = 1.3f;
			float linear_x  = (CH[1] - 1500) / (500.0f - deadzone) * kinematics.max_linear_vel_;
			float linear_y  = (CH[3] - 1500) / (500.0f - deadzone) * kinematics.max_linear_vel_; 
			float angular_z = (CH[0] - 1500) / (500.0f - deadzone) * kinematics.max_angular_z_;
				
			Set_Vel(linear_x, -linear_y, -angular_z);
		}
		else
			flag.robot_sta = MODE_UART_CTRL;
	}
	else
		flag.robot_sta = MODE_UART_CTRL;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim16)
	{
		PPM_Decode();
	}
}
