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
	
	/*��ֹ���*/
	motionless_check(dT_ms);

	/*ת����λ*/
	for(u8 i =0 ;i<3;i++)
	{
		/*������ת������ÿ�룬����+-2000��*/
		sensor.Gyro_deg[i] = sensor.Gyro_Original[i] *0.061036f ;//  /65535 * 4000; +-2000�� 0.061

		/*������ת�������ȶ�ÿ�룬����+-2000��*/
		sensor.Gyro_rad[i] = sensor.Gyro_deg[i] *0.01745f;//sensor.Gyro[i] *RANGE_PN2000_TO_RAD ;//  0.001065264436f //΢��ֵ 0.0010652f
		sensor.gyro_rps[i] = sensor.Gyro_rad[i];
		/*���ٶȼ�ת��������ÿƽ���룬����+-8G*/
		sensor.accel_mpss[i] = (sensor.Acc_Original[i] * RANGE_PN16G_TO_CMSS ) / 100.0f;
	}
}

#include "icm20602.h"
void Sensor_Get()//1ms
{
	/*��ȡ�����Ǽ��ٶȼ�����*/
	ICM20602_Get6AxisRawData(sensor.Acc_Original, sensor.Gyro_Original);
                             
} 
