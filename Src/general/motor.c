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
 * @brief �޷�����
 * @param  x                ��������
 * @param  min              ��Сֵ
 * @param  max              ���ֵ
 * @return float 
 */
float Get_MiMx(float x, float min, float max ) 
{
	return ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) );
}

/**
 * @brief �˶�ѧģ�ͳ�ʼ��
 */
void Kinematics_Init(void)
{
	kinematics.max_rpm_ 						= 330; 				// ����ת��330rpm
	kinematics.wheels_x_distance_		= 0.16;
	kinematics.wheels_y_distance_		= 0.26;
	kinematics.pwm_res_							= 500;
	kinematics.wheel_circumference_	= 0.2356194;  //�����ܳ�
	kinematics.total_wheels_				= 4;
	
	kinematics.exp_vel.linear_x  = 0.0;
	kinematics.exp_vel.linear_y  = 0.0;
	kinematics.exp_vel.angular_z = 0.0;
	
	kinematics.odom.pose.x = 0;
	kinematics.odom.pose.y = 0;
}

/**
 * @brief PID������ʼ��
 */
void PID_Init(void)
{
	// ����ٶ�PID������ʼ��
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
	// ����PID������ʼ��
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
 * @brief ��ȡ����������
 * @param  dT_us            ��ȡʱ����
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
 * @brief �����ٵ��PWM���
 */
void Set_PWM(void)
{	
	const int 	ZERO 						= 500; 		// �����ֹʱ������ֵ
	const float HUNDRED_PERCENT = 500.0f;  // ����������ʱ������ֵ
	TIM1->CCR2 = -kinematics.pwm.motor_1 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR3 =  kinematics.pwm.motor_2 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR1 = -kinematics.pwm.motor_3 * HUNDRED_PERCENT + ZERO;
	TIM1->CCR4 =  kinematics.pwm.motor_4 * HUNDRED_PERCENT + ZERO;
}

/**
 * @brief �����������
 * @param  dT_us            ��������
 */
void Motor_Task(u32 dT_us)
{
	Encoder_Task(dT_us); 		// ��ȡ����������
	Exp_Speed_Cal(dT_us);  	// ���������ٶ�
	Fb_Speed_Cal(dT_us); 		// ���㷴���ٶ�
	
	// ��ǰ��PID����
	PID_Controller(	dT_us,  													// ��������
									kinematics.exp_wheel_rpm.motor_1, // ����ֵ
									kinematics.fb_wheel_rpm.motor_1, 	// ����ֵ
									&pid[FL], 												// PID����
									0,																// ���λ����޷�
									0);																// �����޷�
	pid[FL].out  = Get_MiMx(pid[FL].out, -1.0, 1.0); 	// ����޷�
	
	// ��ǰ��PID����
	PID_Controller(	dT_us,  													// ��������
									kinematics.exp_wheel_rpm.motor_2, // ����ֵ
									kinematics.fb_wheel_rpm.motor_2, 	// ����ֵ
									&pid[FR], 												// PID����
									0,																// ���λ����޷�
									0);																// �����޷�
	pid[FR].out  = Get_MiMx(pid[FR].out, -1.0, 1.0); 	// ����޷�
	
	// �����PID����
	PID_Controller(	dT_us,  													// ��������
									kinematics.exp_wheel_rpm.motor_3, // ����ֵ
									kinematics.fb_wheel_rpm.motor_3, 	// ����ֵ
									&pid[BL], 												// PID����
									0,																// ���λ����޷�
									0);																// �����޷�
	pid[BL].out  = Get_MiMx(pid[BL].out, -1.0, 1.0); 	// ����޷�
	
	// �Һ���PID����
	PID_Controller(	dT_us,  													// ��������
									kinematics.exp_wheel_rpm.motor_4, // ����ֵ
									kinematics.fb_wheel_rpm.motor_4, 	// ����ֵ
									&pid[BR], 												// PID����
									0,																// ���λ����޷�
									0);																// �����޷�
	pid[BR].out  = Get_MiMx(pid[BR].out, -1.0, 1.0); 	// ����޷�
	
	// ��PID������ת��ΪPWMռ�ձ�
	kinematics.pwm.motor_1 = -pid[FL].out;
	kinematics.pwm.motor_2 = -pid[FR].out;
	kinematics.pwm.motor_3 = -pid[BL].out;
	kinematics.pwm.motor_4 = -pid[BR].out;
	// ��������
	Set_PWM();

}

/**
 * @brief ���������ٶ�
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
	
	// �� m/s ת��Ϊ m/min
	linear_vel_x_mins = kinematics.exp_vel.linear_x * 60.0f;
	linear_vel_y_mins = kinematics.exp_vel.linear_y * 60.0f;

	// �� rad/s ת��Ϊ rad/min
	angular_vel_z_mins = kinematics.exp_vel.angular_z * 60.0f;

	// �����ٶ�
	tangential_vel = angular_vel_z_mins * ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2));

	x_rpm = linear_vel_x_mins / kinematics.wheel_circumference_;
	y_rpm = linear_vel_y_mins / kinematics.wheel_circumference_;
//	tan_rpm = tangential_vel / kinematics.wheel_circumference_;
	
	// ������̬�����������õ���z����ٶȺ��ñ��������ݽ���õ��Ľ��ٶ����ܴ�ʱ��
	// ����Ϊ���Ӵ�
//	if(my_abs(sensor.Gyro_rad[Z] - kinematics.fb_vel.angular_z) > 0.5) 
//	{
//		tan_rpm = tangential_vel / kinematics.wheel_circumference_;
//		pid_yaw.out = tan_rpm;
//	}
//	else
	{
		PID_Controller(	dT_us,  											// �������� us
										kinematics.exp_vel.angular_z, // Ŀ��ֵ
										sensor.Gyro_rad[Z], 					// ����ֵ
										&pid_yaw, 										// PID����
										0,														// ���λ����޷�
										0);														// �����޷�
		// ����޷�
		pid_yaw.out = Get_MiMx( pid_yaw.out, 
													 -kinematics.max_rpm_, 
													  kinematics.max_rpm_);
		tan_rpm = pid_yaw.out;
	}
	
	// ʹ�����˶�ѧģ�ͼ��㳵�ֵ������ٶ�
	// ��ǰ�ֵ��
	kinematics.exp_wheel_rpm.motor_1 = x_rpm - y_rpm - tan_rpm;
	kinematics.exp_wheel_rpm.motor_1 = Get_MiMx(kinematics.exp_wheel_rpm.motor_1, -kinematics.max_rpm_, kinematics.max_rpm_);

	// ��ǰ�ֵ��
	kinematics.exp_wheel_rpm.motor_2 = x_rpm + y_rpm + tan_rpm;
	kinematics.exp_wheel_rpm.motor_2 = Get_MiMx(kinematics.exp_wheel_rpm.motor_2, -kinematics.max_rpm_, kinematics.max_rpm_);

	// ����ֵ��
	kinematics.exp_wheel_rpm.motor_3 = x_rpm + y_rpm - tan_rpm;
	kinematics.exp_wheel_rpm.motor_3 = Get_MiMx(kinematics.exp_wheel_rpm.motor_3, -kinematics.max_rpm_, kinematics.max_rpm_);

	// �Һ��ֵ��
	kinematics.exp_wheel_rpm.motor_4 = x_rpm - y_rpm + tan_rpm;
	kinematics.exp_wheel_rpm.motor_4 = Get_MiMx(kinematics.exp_wheel_rpm.motor_4, -kinematics.max_rpm_, kinematics.max_rpm_);

}

/**
 * @brief ���㷴���ٶ�
 * @param  dT_us            ��������
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

	// ��ƽ��ÿ����ת��(rpm)ת��Ϊƽ��ÿ��ת��(rps)
	average_rps_x = ( rpm1 + rpm2 + rpm3 + rpm4) / kinematics.total_wheels_ / 60.0f;
	average_rps_y = (-rpm1 + rpm2 + rpm3 - rpm4) / kinematics.total_wheels_ / 60.0f;
	average_rps_a = (-rpm1 + rpm2 - rpm3 + rpm4) / kinematics.total_wheels_ / 60.0f;
	
	// ��rpsת��Ϊm/s
	kinematics.fb_vel.linear_x  =  average_rps_x * kinematics.wheel_circumference_; // m/s
	kinematics.fb_vel.linear_y  =  average_rps_y * kinematics.wheel_circumference_; // m/s
	kinematics.fb_vel.angular_z = (average_rps_a * kinematics.wheel_circumference_) 
															/ ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2)); //  rad/s

	// ������̼�����
	kinematics.odom.vel.linear_x  = kinematics.fb_vel.linear_x;
	kinematics.odom.vel.linear_y  = kinematics.fb_vel.linear_y;	
	kinematics.odom.vel.angular_z = sensor.Gyro_rad[Z];	
	
	kinematics.odom.pose.theta = -imu_data.yaw;	
	float dT_s = dT_us * 1e-6f;
	float theta_rad = kinematics.odom.pose.theta / 180.0f * 3.1415926535f; // ת��Ϊ����
	kinematics.odom.pose.x += (kinematics.odom.vel.linear_x * my_cos(theta_rad) + kinematics.odom.vel.linear_y * my_sin(theta_rad)) * dT_s;
	kinematics.odom.pose.y += (kinematics.odom.vel.linear_x * my_sin(theta_rad) + kinematics.odom.vel.linear_y * my_cos(theta_rad)) * dT_s;


}
/**
 * @brief PID������
 * @param  dT_us            ��������
 * @param  expect           ����ֵ
 * @param  feedback         ����ֵ
 * @param  pid              pid�����ṹ��
 * @param  inte_d_lim       ���λ����޷�
 * @param  inte_lim         �����޷�
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
	
	// ����ƫ��
	pid->err = expect - feedback;
	
	// ����
	pid->err_inte += Get_MiMx(pid->err,		   -inte_d_lim, inte_d_lim) * dT_s;
	pid->err_inte  = Get_MiMx(pid->err_inte, -inte_lim, 	 inte_lim ); // �����޷�
	
	// ΢��
	pid->err_diff = (pid->err - pid->err_last) * freq;
	
	// ���������֡�΢�����
	pid->out += pid->kp * pid->err 
					  + pid->ki * pid->err_inte 
					  + pid->kd * pid->err_diff;	
	
	pid->err_last = pid->err;
}

