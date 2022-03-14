#ifndef __SENSOR_BASIC_H
#define __SENSOR_BASIC_H

//==����
#include "sys.h"

//==����
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
	MODE_REMOTE_CTRL = 0, 	// ң�ؿ���
	MODE_UART_CTRL,				// ���ڿ���
};

typedef struct
{
	u8 sensor_imu_ok;
	u8 mems_temperature_ok;
	
	u8 motionless;
	
	u32 remote_ctrl_timestamp;
	u8 robot_sta; // ������״̬
	u8 low_power; // ����
	
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
	//У׼���� 
	u8 acc_CALIBRATE;
	u8 gyr_CALIBRATE;
	u8 acc_z_auto_CALIBRATE;
	
	//ԭʼ����
	s16 Acc_Original[VEC_XYZ];
	s16 Gyro_Original[VEC_XYZ];	
	
	s16 Acc[VEC_XYZ];
	s32 Acc_cmss[VEC_XYZ];			//��λ��cm/s2
	float Gyro[VEC_XYZ];
	float Gyro_deg[VEC_XYZ];    //��λ���Ƕ�
	float Gyro_rad[VEC_XYZ];		//��λ������
	float gyro_rps[3];
	float accel_mpss[3];

	s16 Tempreature;
	float Tempreature_C;				//��λ����
	
}_sensor_st;//__attribute__((packed)) 

extern _sensor_st sensor;

//==��������

//public
void Sensor_Data_Prepare(u8 dT_ms);
void Sensor_Get(void);





#endif
