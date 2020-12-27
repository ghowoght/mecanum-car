/**
 * @file motor.c
 * @brief 
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
#include "Sensor_Basic.h"
#include "imu.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim20;

enum
{
	FL = 0,
	FR,
	BL,
	BR
}WHEELS;

kinematics_st kinematics;
pid_st pid[4];
pid_st pid_yaw;

/**
 * @brief 限幅函数
 * @param  x                输入数据
 * @param  min              最小值
 * @param  max              最大值
 * @return float 
 */
float Get_MiMx(float x, float min, float max ) 
{
	return ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) );
}

/**
 * @brief 运动学模型初始化
 */
void Kinematics_Init(void)
{
	kinematics.max_rpm_ 						= 330; 				// 空载转速330rpm
	kinematics.wheels_x_distance_		= 0.16;
	kinematics.wheels_y_distance_		= 0.26;
	kinematics.pwm_res_							= 500;
	kinematics.wheel_circumference_	= 0.2356194;  //轮子周长
	kinematics.total_wheels_				= 4;
	
	kinematics.exp_vel.linear_x  = 0.0;
	kinematics.exp_vel.linear_y  = 0.0;
	kinematics.exp_vel.angular_z = 0.0;
	
	kinematics.odom.pose.x = 0;
	kinematics.odom.pose.y = 0;
}

/**
 * @brief PID参数初始化
 */
void PID_Init(void)
{
	// 电机速度PID参数初始化
	for(int i = 0; i < 4; i++)
	{
		pid[i].kp							= 0.0015;	
		pid[i].ki							= 0;	
		pid[i].kd							= 0.0001;	
		pid[i].err						= 0;	
		pid[i].err_last				= 0;	
		pid[i].err_inte				= 0;	
		pid[i].err_diff				= 0;	
		pid[i].expect					= 0;	
		pid[i].expect_last		= 0;	
		pid[i].feedback				= 0;	
		pid[i].feedback_last	= 0;	
		pid[i].out						= 0;	
	}
	// 方向PID参数初始化
	pid_yaw.kp							= 4;	
	pid_yaw.ki							= 0;	
	pid_yaw.kd							= 0.1;	
	pid_yaw.err							= 0;	
	pid_yaw.err_last				= 0;	
	pid_yaw.err_inte				= 0;	
	pid_yaw.err_diff				= 0;	
	pid_yaw.expect					= 0;	
	pid_yaw.expect_last			= 0;	
	pid_yaw.feedback				= 0;	
	pid_yaw.feedback_last		= 0;	
	pid_yaw.out							= 0;	
}

/**
 * @brief 读取编码器数据
 * @param  dT_us            读取时间间隔
 */
void Encoder_Task(u32 dT_us)
{
	int encoder[4] = {0};
	encoder[0] =  (short)TIM2->CNT;
	encoder[2] =  (short)TIM3->CNT; 
	encoder[1] = -(short)TIM4->CNT; 
	encoder[3] = -(short)TIM20->CNT;
	
	TIM2->CNT  = 0;
	TIM3->CNT  = 0;
	TIM4->CNT  = 0;
	TIM20->CNT = 0;
//	printf("%d %d %d %d\r\n", encoder[0], encoder[1], encoder[2], encoder[3]);
	
	float dT_s = dT_us * 1e-6;
	
	kinematics.fb_wheel_rpm.motor_3 = encoder[0] / 29700.0f / dT_s * 60.0f;
	kinematics.fb_wheel_rpm.motor_2 = encoder[1] / 30000.0f / dT_s * 60.0f;
	kinematics.fb_wheel_rpm.motor_1 = encoder[2] / 29700.0f / dT_s * 60.0f;
  kinematics.fb_wheel_rpm.motor_4 = encoder[3] / 27000.0f / dT_s * 60.0f;
}

/**
 * @brief 设置速电机PWM输出
 */
void Set_PWM(void)
{	
	const int 	ZERO 						= 500; 		// 电机静止时的设置值
	const float HUNDRED_PERCENT = 500.0f;  // 电机满幅输出时的设置值
	TIM1->CCR2 = -kinematics.pwm.motor_1 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR3 =  kinematics.pwm.motor_2 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR1 = -kinematics.pwm.motor_3 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR4 =  kinematics.pwm.motor_4 * HUNDRED_PERCENT + ZERO;
}

/**
 * @brief 电机控制任务
 * @param  dT_us            控制周期
 */
void Motor_Task(u32 dT_us)
{
	Encoder_Task(dT_us); 		// 获取编码器读数
	Exp_Speed_Cal(dT_us);  	// 计算期望速度
	Fb_Speed_Cal(dT_us); 		// 计算反馈速度
	
	// 左前轮PID控制
	PID_Controller(	dT_us,  													// 控制周期
									kinematics.exp_wheel_rpm.motor_1, // 期望值
									kinematics.fb_wheel_rpm.motor_1, 	// 反馈值
									&pid[FL], 												// PID参数
									0,																// 单次积分限幅
									0);																// 积分限幅
	pid[FL].out  = Get_MiMx(pid[FL].out, -1.0, 1.0); 	// 输出限幅
	
	// 右前轮PID控制
	PID_Controller(	dT_us,  													// 控制周期
									kinematics.exp_wheel_rpm.motor_2, // 期望值
									kinematics.fb_wheel_rpm.motor_2, 	// 反馈值
									&pid[FR], 												// PID参数
									0,																// 单次积分限幅
									0);																// 积分限幅
	pid[FR].out  = Get_MiMx(pid[FR].out, -1.0, 1.0); 	// 输出限幅
	
	// 左后轮PID控制
	PID_Controller(	dT_us,  													// 控制周期
									kinematics.exp_wheel_rpm.motor_3, // 期望值
									kinematics.fb_wheel_rpm.motor_3, 	// 反馈值
									&pid[BL], 												// PID参数
									0,																// 单次积分限幅
									0);																// 积分限幅
	pid[BL].out  = Get_MiMx(pid[BL].out, -1.0, 1.0); 	// 输出限幅
	
	// 右后轮PID控制
	PID_Controller(	dT_us,  													// 控制周期
									kinematics.exp_wheel_rpm.motor_4, // 期望值
									kinematics.fb_wheel_rpm.motor_4, 	// 反馈值
									&pid[BR], 												// PID参数
									0,																// 单次积分限幅
									0);																// 积分限幅
	pid[BR].out  = Get_MiMx(pid[BR].out, -1.0, 1.0); 	// 输出限幅
	
	// 将PID计算结果转换为PWM占空比
	kinematics.pwm.motor_1 = -pid[FL].out;
	kinematics.pwm.motor_2 = -pid[FR].out;
	kinematics.pwm.motor_3 = -pid[BL].out;
	kinematics.pwm.motor_4 = -pid[BR].out;
	// 输出到电机
	Set_PWM();

}

/**
 * @brief 计算期望速度
 */
void Exp_Speed_Cal(u32 dT_us)
{
	float linear_vel_x_mins;
	float linear_vel_y_mins;
	float angular_vel_z_mins;
	float tangential_vel;
	float x_rpm;
	float y_rpm;
	float tan_rpm;
	
	// 将 m/s 转换为 m/min
	linear_vel_x_mins = kinematics.exp_vel.linear_x * 60.0f;
	linear_vel_y_mins = kinematics.exp_vel.linear_y * 60.0f;

	// 将 rad/s 转换为 rad/min
	angular_vel_z_mins = kinematics.exp_vel.angular_z * 60.0f;

	// 切向速度
	tangential_vel = angular_vel_z_mins * ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2));

	x_rpm = linear_vel_x_mins / kinematics.wheel_circumference_;
	y_rpm = linear_vel_y_mins / kinematics.wheel_circumference_;
//	tan_rpm = tangential_vel / kinematics.wheel_circumference_;
	
	// 当用姿态传感器测量得到的z轴角速度和用编码器数据解算得到的角速度相差很大时，
	// 可认为轮子打滑
//	if(my_abs(sensor.Gyro_rad[Z] - kinematics.fb_vel.angular_z) > 0.5) 
//	{
//		tan_rpm = tangential_vel / kinematics.wheel_circumference_;
//		pid_yaw.out = tan_rpm;
//	}
//	else
	{
		PID_Controller(	dT_us,  											// 控制周期 us
										kinematics.exp_vel.angular_z, // 目标值
										sensor.Gyro_rad[Z], 					// 反馈值
										&pid_yaw, 										// PID参数
										0,														// 单次积分限幅
										0);														// 积分限幅
		// 输出限幅
		pid_yaw.out = Get_MiMx( pid_yaw.out, 
													 -kinematics.max_rpm_, 
													  kinematics.max_rpm_);
		tan_rpm = pid_yaw.out;
	}
	
	// 使用逆运动学模型计算车轮的期望速度
	// 左前轮电机
	kinematics.exp_wheel_rpm.motor_1 = x_rpm - y_rpm - tan_rpm;
	kinematics.exp_wheel_rpm.motor_1 = Get_MiMx(kinematics.exp_wheel_rpm.motor_1, -kinematics.max_rpm_, kinematics.max_rpm_);

	// 右前轮电机
	kinematics.exp_wheel_rpm.motor_2 = x_rpm + y_rpm + tan_rpm;
	kinematics.exp_wheel_rpm.motor_2 = Get_MiMx(kinematics.exp_wheel_rpm.motor_2, -kinematics.max_rpm_, kinematics.max_rpm_);

	// 左后轮电机
	kinematics.exp_wheel_rpm.motor_3 = x_rpm + y_rpm - tan_rpm;
	kinematics.exp_wheel_rpm.motor_3 = Get_MiMx(kinematics.exp_wheel_rpm.motor_3, -kinematics.max_rpm_, kinematics.max_rpm_);

	// 右后轮电机
	kinematics.exp_wheel_rpm.motor_4 = x_rpm - y_rpm + tan_rpm;
	kinematics.exp_wheel_rpm.motor_4 = Get_MiMx(kinematics.exp_wheel_rpm.motor_4, -kinematics.max_rpm_, kinematics.max_rpm_);

}

/**
 * @brief 计算反馈速度
 * @param  dT_us            计算周期
 */
void Fb_Speed_Cal(u32 dT_us)
{
	float average_rps_x;
	float average_rps_y;
	float average_rps_a;
	
	float rpm1 = kinematics.fb_wheel_rpm.motor_1;
	float rpm2 = kinematics.fb_wheel_rpm.motor_2;
	float rpm3 = kinematics.fb_wheel_rpm.motor_3;
	float rpm4 = kinematics.fb_wheel_rpm.motor_4;

	// 将平均每分钟转速(rpm)转换为平均每秒转速(rps)
	average_rps_x = ( rpm1 + rpm2 + rpm3 + rpm4) / kinematics.total_wheels_ / 60.0f;
	average_rps_y = (-rpm1 + rpm2 + rpm3 - rpm4) / kinematics.total_wheels_ / 60.0f;
	average_rps_a = (-rpm1 + rpm2 - rpm3 + rpm4) / kinematics.total_wheels_ / 60.0f;
	
	// 将rps转换为m/s
	kinematics.fb_vel.linear_x  =  average_rps_x * kinematics.wheel_circumference_; // m/s
	kinematics.fb_vel.linear_y  =  average_rps_y * kinematics.wheel_circumference_; // m/s
	kinematics.fb_vel.angular_z = (average_rps_a * kinematics.wheel_circumference_) 
															/ ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2)); //  rad/s

	// 更新里程计数据
	kinematics.odom.vel.linear_x  = kinematics.fb_vel.linear_x;
	kinematics.odom.vel.linear_y  = kinematics.fb_vel.linear_y;	
	kinematics.odom.vel.angular_z = sensor.Gyro_rad[Z];	
	
	kinematics.odom.pose.theta = -imu_data.yaw;	
	float dT_s = dT_us * 1e-6f;
	float theta_rad = kinematics.odom.pose.theta / 180.0f * 3.1415926535f; // 转化为弧度
	kinematics.odom.pose.x += (kinematics.odom.vel.linear_x * my_cos(theta_rad) + kinematics.odom.vel.linear_y * my_sin(theta_rad)) * dT_s;
	kinematics.odom.pose.y += (kinematics.odom.vel.linear_x * my_sin(theta_rad) + kinematics.odom.vel.linear_y * my_cos(theta_rad)) * dT_s;


}
/**
 * @brief PID控制器
 * @param  dT_us            控制周期
 * @param  expect           期望值
 * @param  feedback         反馈值
 * @param  pid              pid参数结构体
 * @param  inte_d_lim       单次积分限幅
 * @param  inte_lim         积分限幅
 */
void PID_Controller(u32     dT_us,
										float   expect, 
										float   feedback, 
										pid_st* pid,
										float   inte_d_lim,
										float   inte_lim 		
										)
{
	float dT_s = (float)dT_us * 1e-6f;
	float freq = safe_div(1.0f, dT_s, 0);
	
	// 计算偏差
	pid->err = expect - feedback;
	
	// 积分
	pid->err_inte += Get_MiMx(pid->err,		   -inte_d_lim, inte_d_lim) * dT_s;
	pid->err_inte  = Get_MiMx(pid->err_inte, -inte_lim, 	 inte_lim ); // 积分限幅
	
	// 微分
	pid->err_diff = (pid->err - pid->err_last) * freq;
	
	// 比例、积分、微分相加
	pid->out += pid->kp * pid->err 
					  + pid->ki * pid->err_inte 
					  + pid->kd * pid->err_diff;	
	
	pid->err_last = pid->err;
}

