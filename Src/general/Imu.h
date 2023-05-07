#ifndef _IMU_H
#define _IMU_H

#include "Sensor_Basic.h"
#include <math.h>
#include <stdint.h>

typedef struct
{
	float w;//q0;
	float x;//q1;
	float y;//q2;
	float z;//q3;

	float x_vec[VEC_XYZ];     //X轴向量
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];
	float hx_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float w_acc[VEC_XYZ];
	float h_acc[VEC_XYZ];
	
	float w_mag[VEC_XYZ];
	
	float gacc_deadzone[VEC_XYZ];
	
	float obs_acc_w[VEC_XYZ];
	float obs_acc_a[VEC_XYZ];
	float gra_acc[VEC_XYZ];
	
	float est_acc_a[VEC_XYZ];
	float est_acc_h[VEC_XYZ];
	float est_acc_w[VEC_XYZ];
	
	float est_speed_h[VEC_XYZ];
	float est_speed_w[VEC_XYZ];

	
	float rol;
	float pit;
	float yaw;
} _imu_st ;
extern _imu_st imu_data;

extern uint16_t imu_data_cnt;

#define IMU_FILTER_CUTOFF_FREQ	30.0f

//校准时间
#define ACC_CALC_TIME  3000//ms
#define GYRO_CALC_TIME   3	//s

#define M_PI_F 3.1415926
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* 癈			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

#define so3_comp_params_Kp 	2.0f
#define so3_comp_params_Ki  0.05f

extern float q0, q1, q2, q3;	/** quaternion of sensor frame relative to auxiliary frame */
extern float dq0, dq1, dq2, dq3;	/** quaternion of sensor frame relative to auxiliary frame */
extern float gyro_bias[3]; /** bias estimation */
extern float q0q0, q0q1, q0q2, q0q3;
extern float q1q1, q1q2, q1q3;
extern float q2q2, q2q3;
extern float q3q3;
extern int bFilterInit;

float invSqrt(float number);

void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

void ImuUpdate_Task(uint32_t dT_ms);

#endif

