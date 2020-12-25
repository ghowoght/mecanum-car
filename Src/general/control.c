#include "main.h"
#include "stm32g4xx_hal.h"
#include "motor.h"
#include "math.h"
#include "stdio.h"
#include "control.h"

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

uint8_t state = S0;
int state_cnt = 0;
void Ctrl_Task(u32 dT_ms)
{
	if(state == S0)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 1000)
		{
			state_cnt = 0;
			state = S1;
		}
	}
	else if(state == S1)
	{
//		state_cnt += dT_ms;
//		kinematics.exp_vel.linear_x = 0;
//		kinematics.exp_vel.linear_y = 0;
//		kinematics.exp_vel.angular_z = 1.57;
//		if(state_cnt >= 4000)
//		{
//			state_cnt = 0;
//			state = S6;
//		}
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0.5;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 1500)
		{
			state_cnt = 0;
			state = S2;
		}
	}
	else if(state == S2)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 100)
		{
			state_cnt = 0;
			state = S3;
		}
	}
	else if(state == S3)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0.5;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 1600)
		{
			state_cnt = 0;
			state = S4;
		}
	}
	else if(state == S4)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 100)
		{
			state_cnt = 0;
			state = S5;
		}
	}
	else if(state == S5)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = -0.5;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 1500)
		{
			state_cnt = 0;
			state = S6;
		}
	}
	else if(state == S6)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 100)
		{
			state_cnt = 0;
			state = S7;
		}
	}
	else if(state == S7)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = -0.5;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
		if(state_cnt >= 1600)
		{
			state_cnt = 0;
			state = S9;
		}
	}
	else if(state == S8)
	{
		state_cnt += dT_ms;
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 1.57;
		if(state_cnt >= 4000)
		{
			state_cnt = 0;
			state = S9;
		}
	}
	
	else if(state == S9)
	{
		kinematics.exp_vel.linear_x = 0;
		kinematics.exp_vel.linear_y = 0;
		kinematics.exp_vel.angular_z = 0;
	}
	
	
}
