#ifndef _DATA_H_
#define _DATA_H_
#include "sys.h"

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
	
} _flag_st;
extern _flag_st flag;

typedef struct
{
	u8 first_f;
	float acc_offset[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	
	float surface_vec[VEC_XYZ];
	
	float mag_offset[VEC_XYZ];
	float mag_gain[VEC_XYZ];

} _save_st ;
extern _save_st save;

#endif
