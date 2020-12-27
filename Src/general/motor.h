/**
 * @file motor.h
 * @brief 
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020  WHU-EIS
 * 
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "sys.h"

/**
 * 小车坐标系
 *   +x
 *   |
 * --|--+y
 *   |
 * 
 */

typedef struct 
{
	volatile float linear_x;  // x轴线速度 m/s
	volatile float linear_y;  // y轴线速度 m/s
	volatile float angular_z; // z轴角速度 rad/s 右手准则
}vel_st; 

typedef struct
{
	float motor_1;
	float motor_2;
	float motor_3;
	float motor_4;
}motor_st;

typedef struct
{
	float x;		 // x方向坐标
	float y;		 // y方向坐标
	float theta; // 方向角
}pose_st;

typedef struct
{
	vel_st  vel; 	// 速度
	pose_st pose; // 位姿
}odom_st;


// 小车运动学模型结构体
typedef struct
{
	int 	max_rpm_; 						// 每分钟车轮的最大转速
	int 	total_wheels_;				// 车轮数量
	float wheels_x_distance_;		// x方向上车轮的距离				↑x
	float wheels_y_distance_;		// y方向上车轮的距离				|-->y
	float pwm_res_;							// PWM输出最大值
	float wheel_circumference_;	// 车轮周长
	
	vel_st exp_vel;					// 小车的期望速度
	vel_st fb_vel;					// 测量和解算得到的小车速度
	motor_st exp_wheel_rpm;	// 使用逆运动学模型转换得到的轮子的目标转速
	motor_st fb_wheel_cmps;	// 编码器反馈值 单位：cm/s
	motor_st fb_wheel_rpm;	// 编码器反馈值 单位：rad/min
	motor_st pwm;						// 电机的PWM输出值
	
	odom_st  odom;					// 里程计数据
}kinematics_st;
extern kinematics_st kinematics;

// PID参数结构体
typedef struct
{
	float kp;							// 比例系数
	float ki;							// 积分系数
	float kd;							// 微分系数
	
	float err;						// 误差
	float err_last;				// 前一次的误差
	float err_inte;				// 误差积分
	float err_diff;				// 误差微分

	float expect;					// 期望值
	float expect_last;		// 前一次的目标值
	float feedback;				// 反馈值
	float feedback_last;	// 前一次的反馈值
	
	float out;						// PID输出
}pid_st;
extern pid_st pid[4];		// 车轮转速控制PID参数
extern pid_st pid_yaw;  // 方向控制PID参数

void Kinematics_Init(void);
void Encoder_Task(u32 dT_us);

void Set_PWM(void);
void PID_Init(void);
void Motor_Task(u32 dT_us);

void Exp_Speed_Cal(u32 dT_us);
void Fb_Speed_Cal(u32 dT_us);

void PID_Controller( u32 dT_us,       // 控制周期
										float expect,     // 期望值
										float feedback,   // 反馈值
										pid_st *pid,      // pid参数结构体
										float inte_d_lim, // 单次积分限幅
										float inte_lim 		// 积分限幅
										);
										
float Get_MiMx(float x, float min, float max );

#endif 
