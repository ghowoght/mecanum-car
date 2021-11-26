#include "Sensor_Basic.h"
#include "Math.h"
#include "motor.h"

_sensor_st sensor;
_flag_st flag;

s16 g_old[VEC_XYZ];
float g_d_sum[VEC_XYZ] = {500,500,500};
void motionless_check(u8 dT_ms)
{
	static u32 cnt = 0;
	u8 t = 0;
	for(u8 i = 0;i < 3; i++)
	{
		g_d_sum[i] += 3*ABS(sensor.Gyro_Original[i] - g_old[i]) ;
		g_d_sum[i] -= dT_ms ;	
		g_d_sum[i] = LIMIT(g_d_sum[i], 0, 200);
		if( g_d_sum[i] > 5)
		{
			t++;
		}
		g_old[i] = sensor.Gyro_Original[i];
	}
	if(t >= 1)
		cnt = 0;
	else
		cnt += dT_ms;
	if(cnt > 100)
		flag.motionless = 1;
	else
		flag.motionless = 0;
}


void Sensor_Data_Prepare(u8 dT_ms)
{	
	float hz = 0 ;
	if(dT_ms != 0) hz = 1000/dT_ms;
	
	/*静止检测*/
	motionless_check(dT_ms);

	/*转换单位*/
	for(u8 i =0 ;i<3;i++)
	{
		/*陀螺仪转换到度每秒，量程+-2000度*/
		sensor.Gyro_deg[i] = sensor.Gyro_Original[i] *0.061036f ;//  /65535 * 4000; +-2000度 0.061

		/*陀螺仪转换到弧度度每秒，量程+-2000度*/
		sensor.Gyro_rad[i] = sensor.Gyro_deg[i] *0.01745f;//sensor.Gyro[i] *RANGE_PN2000_TO_RAD ;//  0.001065264436f //微调值 0.0010652f
		sensor.gyro_rps[i] = sensor.Gyro_rad[i];
		/*加速度计转换到厘米每平方秒，量程+-8G*/
		sensor.accel_mpss[i] = (sensor.Acc_Original[i] * RANGE_PN16G_TO_CMSS ) / 100.0f;
	}
}

#include "icm20602.h"
void Sensor_Get()//1ms
{
	/*读取陀螺仪加速度计数据*/
	ICM20602_Get6AxisRawData(sensor.Acc_Original, sensor.Gyro_Original);
                             
} 
