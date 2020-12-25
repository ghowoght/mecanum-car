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
odom_st odom;

float Get_MiMx(float x, float min, float max ) 
{
	return ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) );
}

/* 运动学模型初始化 */
void Kinematics_Init(void)
{
	kinematics.max_rpm_ 						= 330; // 空载330rpm
	kinematics.wheels_x_distance_		= 0.16;
	kinematics.wheels_y_distance_		= 0.26;
	kinematics.pwm_res_							= 500;
	kinematics.wheel_circumference_	= 0.2356194; //轮子周长
	kinematics.total_wheels_				= 4;
	
	kinematics.exp_vel.linear_x  = 0.0;
	kinematics.exp_vel.linear_y  = 0.0;
	kinematics.exp_vel.angular_z = 0.0;
	
	odom.pose.x = 0;
	odom.pose.y = 0;
	
}

/* PID参数初始化 */
void PID_Init(void)
{
	for(int i = 0; i < 4; i++)
	{
		pid[i].kp							= 0.0018;	
		pid[i].ki							= 0;	
		pid[i].kd							= 0.0008;	
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
	pid_yaw.kp							= 8;	
	pid_yaw.ki							= 0;	
	pid_yaw.kd							= 8;	
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

#define MIN_ENCODER_PRICISE 0.000791812 // π*7.5/29757 cm

/* 读取编码器数据 */
void Encoder_Task(u32 dT_us)
{
	int encoder[4] = {0};
	encoder[0] =  (short)TIM2->CNT;
	encoder[2] =  (short)TIM3->CNT; 
	encoder[1] = -(short)TIM4->CNT; 
	encoder[3] = -(short)TIM20->CNT;
	
	TIM2->CNT = 0;
	TIM3->CNT = 0;
	TIM4->CNT = 0;
	TIM20->CNT = 0;
//	printf("%d %d %d %d\r\n", encoder[0], encoder[1], encoder[2], encoder[3]);
	
	float dT_s = dT_us * 1e-6;
//	kinematics.fb_wheel_cmps.motor_3 = encoder[0] * MIN_ENCODER_PRICISE / dT_us * 10e6;
//	kinematics.fb_wheel_cmps.motor_2 = encoder[1] * MIN_ENCODER_PRICISE / dT_us * 10e6;
//	kinematics.fb_wheel_cmps.motor_1 = encoder[2] * MIN_ENCODER_PRICISE / dT_us * 10e6;
//	kinematics.fb_wheel_cmps.motor_4 = encoder[3] * MIN_ENCODER_PRICISE / dT_us * 10e6;
	
	kinematics.fb_wheel_rpm.motor_3 = encoder[0] / 29700.0f / dT_s * 60.0f;
	kinematics.fb_wheel_rpm.motor_2 = encoder[1] / 30000.0f / dT_s * 60.0f;
	kinematics.fb_wheel_rpm.motor_1 = encoder[2] / 29700.0f / dT_s * 60.0f;
  kinematics.fb_wheel_rpm.motor_4 = encoder[3] / 27000.0f / dT_s * 60.0f;
}

#define ZERO 500
#define HUNDRED_PERCENT 500.0
void Set_Speed(void)
{	
//	if ( kinematics.pwm.motor_1 * HUNDRED_PERCENT == 0
//		&& kinematics.pwm.motor_2 * HUNDRED_PERCENT == 0
//		&& kinematics.pwm.motor_3 * HUNDRED_PERCENT == 0
//		&& kinematics.pwm.motor_4 * HUNDRED_PERCENT == 0)
//	{
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
//			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_4);
//	}
//	else
//	{
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);
//	}
	
	TIM1->CCR2 = -kinematics.pwm.motor_1 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR3 =  kinematics.pwm.motor_2 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR1 = -kinematics.pwm.motor_3 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR4 =  kinematics.pwm.motor_4 * HUNDRED_PERCENT + ZERO;
}


void Motor_Task(u32 dT_us)
{
	Encoder_Task(dT_us);
	Exp_Speed_Cal();
	Fb_Speed_Cal(dT_us);
	
	PID_Controller(	dT_us,  	// 时间间隔
									kinematics.exp_wheel_rpm.motor_1, 			// 目标值
									kinematics.fb_wheel_rpm.motor_1, // 反馈值
									&pid[FL], 	// PID参数
									100,			// 单次积分限幅
									2000);		// 积分限幅
	pid[FL].out  = Get_MiMx(pid[FL].out, -1.0, 1.0);
	PID_Controller(	dT_us,  	// 时间间隔
									kinematics.exp_wheel_rpm.motor_2, 			// 目标值
									kinematics.fb_wheel_rpm.motor_2, // 反馈值
									&pid[FR], 	// PID参数
									100,			// 单次积分限幅
									2000);		// 积分限幅
	pid[FR].out  = Get_MiMx(pid[FR].out, -1.0, 1.0);
	PID_Controller(	dT_us,  	// 时间间隔
									kinematics.exp_wheel_rpm.motor_3, 			// 目标值
									kinematics.fb_wheel_rpm.motor_3, // 反馈值
									&pid[BL], 	// PID参数
									100,			// 单次积分限幅
									2000);		// 积分限幅
	pid[BL].out  = Get_MiMx(pid[BL].out, -1.0, 1.0);
	PID_Controller(	dT_us,  	// 时间间隔
									kinematics.exp_wheel_rpm.motor_4, 			// 目标值
									kinematics.fb_wheel_rpm.motor_4, // 反馈值
									&pid[BR], 	// PID参数
									100,			// 单次积分限幅
									2000);		// 积分限幅
	pid[BR].out  = Get_MiMx(pid[BR].out, -1.0, 1.0);
	
	kinematics.pwm.motor_1 = -pid[FL].out;
	kinematics.pwm.motor_2 = -pid[FR].out;
	kinematics.pwm.motor_3 = -pid[BL].out;
	kinematics.pwm.motor_4 = -pid[BR].out;
	
	Set_Speed();

}

void Exp_Speed_Cal(void)
{
	float linear_vel_x_mins;
	float linear_vel_y_mins;
	float angular_vel_z_mins;
	float tangential_vel;
	float x_rpm;
	float y_rpm;
	float tan_rpm;
	
	// 将 m/s 转换为 m/min
	linear_vel_x_mins = kinematics.exp_vel.linear_x * 60;
	linear_vel_y_mins = kinematics.exp_vel.linear_y * 60;

	// 将 rad/s 转换为 rad/min
	angular_vel_z_mins = kinematics.exp_vel.angular_z * 60;

//	float x = fabs(kinematics.exp_vel.angular_z);
//	float y = exp(1.74459 * x - 1.47749);
//	angular_vel_z_mins = y * 60;
	tangential_vel = angular_vel_z_mins * ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2));

	x_rpm = linear_vel_x_mins / kinematics.wheel_circumference_;
	y_rpm = linear_vel_y_mins / kinematics.wheel_circumference_;
//	tan_rpm = tangential_vel / kinematics.wheel_circumference_;
	

//	if(my_abs(sensor.Gyro_rad[Z] - kinematics.fb_vel.angular_z) > 0.5) // 轮子打滑
//	{
//		tan_rpm = tangential_vel / kinematics.wheel_circumference_;
//		pid_yaw.out = tan_rpm;
//	}
//	else
	{
		PID_Controller(	50000,  	// 时间间隔 us
										kinematics.exp_vel.angular_z, 			// 目标值
										sensor.Gyro_rad[Z], // 反馈值
										&pid_yaw, 	// PID参数
										100,			// 单次积分限幅
										2000);		// 积分限幅
		pid_yaw.out = Get_MiMx(pid_yaw.out, -kinematics.max_rpm_, kinematics.max_rpm_);
		tan_rpm = pid_yaw.out;
	}
	
	// 使用逆运动学模型计算轮子的目标速度
	// 左前方电机
	kinematics.exp_wheel_rpm.motor_1 = x_rpm - y_rpm - tan_rpm;
	kinematics.exp_wheel_rpm.motor_1 = LIMIT(kinematics.exp_wheel_rpm.motor_1, -kinematics.max_rpm_, kinematics.max_rpm_);

	// 右前方电机
	kinematics.exp_wheel_rpm.motor_2 = x_rpm + y_rpm + tan_rpm;
	kinematics.exp_wheel_rpm.motor_2 = LIMIT(kinematics.exp_wheel_rpm.motor_2, -kinematics.max_rpm_, kinematics.max_rpm_);

	// 左后方电机
	kinematics.exp_wheel_rpm.motor_3 = x_rpm + y_rpm - tan_rpm;
	kinematics.exp_wheel_rpm.motor_3 = LIMIT(kinematics.exp_wheel_rpm.motor_3, -kinematics.max_rpm_, kinematics.max_rpm_);

	// 右后方电机
	kinematics.exp_wheel_rpm.motor_4 = x_rpm - y_rpm + tan_rpm;
	kinematics.exp_wheel_rpm.motor_4 = LIMIT(kinematics.exp_wheel_rpm.motor_4, -kinematics.max_rpm_, kinematics.max_rpm_);

}


void Fb_Speed_Cal(u32 dT_us)
{
	float average_rps_x;
	float average_rps_y;
	float average_rps_a;
	
	float rpm1 = kinematics.fb_wheel_rpm.motor_1;
	float rpm2 = kinematics.fb_wheel_rpm.motor_2;
	float rpm3 = kinematics.fb_wheel_rpm.motor_3;
	float rpm4 = kinematics.fb_wheel_rpm.motor_4;

	// convert average revolutions per minute to revolutions per second
	average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / kinematics.total_wheels_) / 60; // RPM
	kinematics.fb_vel.linear_x = average_rps_x * kinematics.wheel_circumference_; // m/s

	// convert average revolutions per minute in y axis to revolutions per second
	average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / kinematics.total_wheels_) / 60; // RPM
	kinematics.fb_vel.linear_y = average_rps_y * kinematics.wheel_circumference_; // m/s

	// convert average revolutions per minute to revolutions per second
	average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / kinematics.total_wheels_) / 60;
	kinematics.fb_vel.angular_z =  (average_rps_a * kinematics.wheel_circumference_) / ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2)); //  rad/s

	// 里程计原始数据赋值
	odom.vel.linear_x = kinematics.fb_vel.linear_x;
	odom.vel.linear_y = kinematics.fb_vel.linear_y;
	
	odom.vel.angular_z = sensor.Gyro_rad[Z];	
	odom.pose.theta = -imu_data.yaw;
	
	float dT_s = dT_us * 1e-6;
	float theta_rad = odom.pose.theta / 180.0f * 3.1415926535f; // 转化为弧度
	odom.pose.x += (odom.vel.linear_x * my_cos(theta_rad) + odom.vel.linear_y * my_sin(theta_rad)) * dT_s;
	odom.pose.y += (odom.vel.linear_x * my_sin(theta_rad) + odom.vel.linear_y * my_cos(theta_rad)) * dT_s;


}
void PID_Controller( u32 dT_us,
										float expect, 
										float feedback, 
										pid_st *pid,
										float inte_d_lim, // 单次积分限幅
										float inte_lim // 积分限幅
											)
{
	float dT_s = (float)dT_us * 10e-6;
	float freq = safe_div(1.0f, dT_s, 0);
	
	pid->err = expect - feedback;
	
	pid->err_inte += LIMIT(pid->err, -inte_d_lim, inte_d_lim) * dT_s;
	pid->err_inte = LIMIT(pid->err_inte, -inte_lim, inte_lim);
	
	pid->err_diff = (pid->err - pid->err_last) * freq;
	
	pid->out += pid->kp * pid->err 
					  + pid->ki * pid->err_inte 
					  + pid->kd * pid->err_diff;	
	
	pid->err_last = pid->err;
}

