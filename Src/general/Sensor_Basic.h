#ifndef __SENSOR_BASIC_H
#define __SENSOR_BASIC_H

//==引用
#include "sys.h"

//==定义
#define OFFSET_AV_NUM 50
#define GRAVITY_ACC_PN16G    2048
#define RANGE_PN2000_TO_RAD  0.001065f
#define RANGE_PN16G_TO_CMSS  0.4790f

enum 
{
	ROL,
	PIT,
	TAW,
	VEC_RPY,
};

enum
{
	X,
	Y,
	Z,
	VEC_XYZ,
};

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 MPU_ITEMS ,
};

typedef struct
{
	int x;
	int y;
}_vector2_st;

typedef struct
{
	int x;
	int y;
	int z;
}_vector3_st;

enum ROBOT_STATUS
{
	MODE_REMOTE_CTRL = 0, 	// 遥控控制
	MODE_UART_CTRL,				// 串口控制
};

typedef struct
{
	u8 sensor_imu_ok;
	u8 mems_temperature_ok;
	
	u8 motionless;
	
	u32 remote_ctrl_timestamp;
	u8 robot_sta; // 机器人状态
	u8 low_power; // 电量
	
} _flag_st;
extern _flag_st flag;

typedef struct
{
  u8 surface_CALIBRATE;
	float surface_vec[VEC_XYZ];
	float surface_unitvec[VEC_XYZ];
	
}_sensor_rotate_st;
extern _sensor_rotate_st sensor_rot ;

typedef struct 
{
	//校准参数 
	u8 acc_CALIBRATE;
	u8 gyr_CALIBRATE;
	u8 acc_z_auto_CALIBRATE;
	
	//原始数据
	s16 Acc_Original[VEC_XYZ];
	s16 Gyro_Original[VEC_XYZ];	
	
	s16 Acc[VEC_XYZ];
	s32 Acc_cmss[VEC_XYZ];			//单位：cm/s2
	float Gyro[VEC_XYZ];
	float Gyro_deg[VEC_XYZ];    //单位：角度
	float Gyro_rad[VEC_XYZ];		//单位：弧度
	float gyro_rps[3];
	float accel_mpss[3];

	s16 Tempreature;
	float Tempreature_C;				//单位：℃
	
}_sensor_st;//__attribute__((packed)) 

extern _sensor_st sensor;

//==函数声明

//public
void Sensor_Data_Prepare(u8 dT_ms);
void Sensor_Get(void);





#endif
