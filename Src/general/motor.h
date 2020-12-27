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
 * С������ϵ
 *   +x
 *   |
 * --|--+y
 *   |
 * 
 */

typedef struct 
{
	volatile float linear_x;  // x�����ٶ� m/s
	volatile float linear_y;  // y�����ٶ� m/s
	volatile float angular_z; // z����ٶ� rad/s ����׼��
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
	float x;		 // x��������
	float y;		 // y��������
	float theta; // �����
}pose_st;

typedef struct
{
	vel_st  vel; 	// �ٶ�
	pose_st pose; // λ��
}odom_st;


// С���˶�ѧģ�ͽṹ��
typedef struct
{
	int 	max_rpm_; 						// ÿ���ӳ��ֵ����ת��
	int 	total_wheels_;				// ��������
	float wheels_x_distance_;		// x�����ϳ��ֵľ���				��x
	float wheels_y_distance_;		// y�����ϳ��ֵľ���				|-->y
	float pwm_res_;							// PWM������ֵ
	float wheel_circumference_;	// �����ܳ�
	
	vel_st exp_vel;					// С���������ٶ�
	vel_st fb_vel;					// �����ͽ���õ���С���ٶ�
	motor_st exp_wheel_rpm;	// ʹ�����˶�ѧģ��ת���õ������ӵ�Ŀ��ת��
	motor_st fb_wheel_cmps;	// ����������ֵ ��λ��cm/s
	motor_st fb_wheel_rpm;	// ����������ֵ ��λ��rad/min
	motor_st pwm;						// �����PWM���ֵ
	
	odom_st  odom;					// ��̼�����
}kinematics_st;
extern kinematics_st kinematics;

// PID�����ṹ��
typedef struct
{
	float kp;							// ����ϵ��
	float ki;							// ����ϵ��
	float kd;							// ΢��ϵ��
	
	float err;						// ���
	float err_last;				// ǰһ�ε����
	float err_inte;				// ������
	float err_diff;				// ���΢��

	float expect;					// ����ֵ
	float expect_last;		// ǰһ�ε�Ŀ��ֵ
	float feedback;				// ����ֵ
	float feedback_last;	// ǰһ�εķ���ֵ
	
	float out;						// PID���
}pid_st;
extern pid_st pid[4];		// ����ת�ٿ���PID����
extern pid_st pid_yaw;  // �������PID����

void Kinematics_Init(void);
void Encoder_Task(u32 dT_us);

void Set_PWM(void);
void PID_Init(void);
void Motor_Task(u32 dT_us);

void Exp_Speed_Cal(u32 dT_us);
void Fb_Speed_Cal(u32 dT_us);

void PID_Controller( u32 dT_us,       // ��������
										float expect,     // ����ֵ
										float feedback,   // ����ֵ
										pid_st *pid,      // pid�����ṹ��
										float inte_d_lim, // ���λ����޷�
										float inte_lim 		// �����޷�
										);
										
float Get_MiMx(float x, float min, float max );

#endif 
