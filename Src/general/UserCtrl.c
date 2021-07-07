/**
 * @file UserCtrl.c
 * @brief 用户控制程序
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020  WHU-EIS
 * 
 */
#include "main.h"
#include "stm32g4xx_hal.h"
#include "motor.h"
#include "math.h"
#include "stdio.h"
#include "UserCtrl.h"

/**
 * @brief 设置小车速度
 * @param  linear_x         x方向线速度
 * @param  linear_y         y方向线速度
 * @param  angular_z        z轴角速度
 */
void Set_Vel(float linear_x, float linear_y, float angular_z)
{
	kinematics.exp_vel.linear_x  = linear_x;
	kinematics.exp_vel.linear_y  = linear_y;
	kinematics.exp_vel.angular_z = angular_z;
}

/**
 * @brief FSM状态枚举体
 */
enum
{
	S0,
	S1,
	S2,
	S3,
	S4,
	S5,
	S6,
	S7,
	S8,
	S9
}FSM;

uint8_t state = S0; // 当前状态
int state_cnt = 0;  // 状态计时
/**
 * @brief 用户控制任务
 * @param  dT_ms  执行周期          
 */
void Ctrl_Task(u32 dT_ms)
{
	if(state == S0)
	{
		state_cnt += dT_ms;
		Set_Vel(0, 0, 0);
		if(state_cnt >= 1000)
		{
			state_cnt = 0;
			state = S1;
		}
	}
	else if(state == S1)
	{
		state_cnt += dT_ms;
		Set_Vel(0, 0.5, 0);
		if(state_cnt >= 1500)
		{
			state_cnt = 0;
			state = S2;
		}
	}
	else if(state == S2)
	{
		state_cnt += dT_ms;
		Set_Vel(0, 0, 0);
		if(state_cnt >= 100)
		{
			state_cnt = 0;
			state = S3;
		}
	}
	else if(state == S3)
	{
		state_cnt += dT_ms;
		Set_Vel(0.5, 0, 0);
		if(state_cnt >= 1600)
		{
			state_cnt = 0;
			state = S4;
		}
	}
	else if(state == S4)
	{
		state_cnt += dT_ms;
		Set_Vel(0, 0, 0);
		if(state_cnt >= 100)
		{
			state_cnt = 0;
			state = S8;
		}
	}
	else if(state == S5)
	{
		state_cnt += dT_ms;
		Set_Vel(0, -0.5, 0);
		if(state_cnt >= 1500)
		{
			state_cnt = 0;
			state = S6;
		}
	}
	else if(state == S6)
	{
		state_cnt += dT_ms;
		Set_Vel(0, 0, 0);
		if(state_cnt >= 100)
		{
			state_cnt = 0;
			state = S7;
		}
	}
	else if(state == S7)
	{
		state_cnt += dT_ms;
		Set_Vel(-0.5, 0, 0);
		if(state_cnt >= 1600)
		{
			state_cnt = 0;
			state = S9;
		}
	}
	else if(state == S8)
	{
		state_cnt += dT_ms;
		Set_Vel(0, 0, 1.57);
		if(state_cnt >= 1000)
		{
			state_cnt = 0;
			state = S9;
		}
	}
	else if(state == S9)
	{
		Set_Vel(0, 0, 0);
	}
	
}
