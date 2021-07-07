/**
 * @file dt.c
 * @brief ���ݴ������
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020  WHU-EIS
 * 
 */
#include "dt.h"
#include "Imu.h"
#include "data.h"
#include "motor.h"
#include "Sensor_Basic.h"
#include "stdio.h"

/**
 ************************ 
 ******* �������� ********
 ************************
 */

#define USE_USB

/**
 * @brief ���ݷ��ͺ���
 * @param  data_to_send     �����͵��ֽ�����
 * @param  cnt              ���������ݳ���
 */
void SendData(u8 *data_to_send, u8 cnt)
{
#ifdef USE_USB
	/* �رմ��ڽ����ж� */
	HAL_NVIC_DisableIRQ(LPUART1_IRQn);
	/* �������� */
	HAL_UART_Transmit(&hlpuart1, data_to_send, cnt, 0xFFFF); 
	/* �������ڽ����ж� */
	HAL_NVIC_EnableIRQ(LPUART1_IRQn);
#else
	/* �رմ��ڽ����ж� */
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	/* �������� */
	HAL_UART_Transmit(&huart1, data_to_send, cnt, 0xFFFF); 
	/* �������ڽ����ж� */
	HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
}

// �����壬����float�ͺ��ֽ�����Ļ���
typedef union
{
	float data;
	uint8_t data8[4];
} data_u;

/**
 * @brief ���ݷ�������
 * @param  dT_ms            ִ������
 */
void DataTrans_Task(u32 dT_ms)
{
	static u32 cnt = 0;
	const u32 sent_imu_cnt 	 = 20;
	const u32 sent_odom_cnt 	 = 50;
	const u32 sent_userdata_cnt 	 = 40;
	const u32 sent_wheel_cnt = 30;
	
	cnt += dT_ms;

	if((cnt % sent_odom_cnt) == sent_odom_cnt - 1)
	{
		DataTrans_Odom();
	}
//	else if((cnt % sent_userdata_cnt) == sent_userdata_cnt - 1)
//	{
//		DataTrans_UserData();
//	}
//		else if((cnt % sent_wheel_cnt) == sent_wheel_cnt - 1)
//	{
//		DataTrans_Wheel();
//	}
//	else if((cnt % sent_imu_cnt) == sent_imu_cnt - 1)
//	{
//		DataTrans_IMU();
//	}
	
	if(cnt>1200) cnt = 0;
	
}

/**
 * @brief ����IMU����
 */
void DataTrans_IMU(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // ����һ��������ʵ����ʹ����������������ת��Ϊ�ֽ�����
	uint8_t data_to_send[100] = {0}; // �����͵��ֽ�����
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x55;
	data_to_send[_cnt++]=0x01; 	// ����
	data_to_send[_cnt++]=12;		// ����
	
	uint8_t _start = _cnt;
		
	// ��Ҫ���͵����ݸ�ֵ���������float��Ա
	// ��Ӧ�ľ��ܸ����ֽ������Ա��ֵ
	_temp.data = imu_data.rol;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = imu_data.pit;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = imu_data.yaw;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  // ���ڷ���
	SendData(data_to_send, _cnt); 
	
}

/**
 * @brief ����С���ٶ�����
 */
void DataTrans_Vel(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // ����һ��������ʵ����ʹ����������������ת��Ϊ�ֽ�����
	uint8_t data_to_send[100] = {0}; // �����͵��ֽ�����
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xCC;
		
	// ��Ҫ���͵����ݸ�ֵ���������float��Ա
	// ��Ӧ�ľ��ܸ����ֽ������Ա��ֵ
	_temp.data = kinematics.fb_vel.linear_x;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_vel.linear_y;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.exp_vel.linear_x;//sensor.Gyro_rad[Z];//kinematics.fb_vel.angular_z;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	uint8_t checkout = 0;
	for(int i = 2; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  // ���ڷ���
	SendData(data_to_send, _cnt); 
}

/**
 * @brief ���ͳ���ת��
 */
void DataTrans_Wheel(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // ����һ��������ʵ����ʹ����������������ת��Ϊ�ֽ�����
	uint8_t data_to_send[100] = {0}; // �����͵��ֽ�����
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x55;
	data_to_send[_cnt++]=0x03; 	// ����
	data_to_send[_cnt++]=16;		// ����
	
	uint8_t _start = _cnt;
		
	// ��Ҫ���͵����ݸ�ֵ���������float��Ա
	// ��Ӧ�ľ��ܸ����ֽ������Ա��ֵ
	_temp.data = kinematics.fb_wheel_rpm.motor_1;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_wheel_rpm.motor_2;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_wheel_rpm.motor_3;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_wheel_rpm.motor_4;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  // ���ڷ���
	SendData(data_to_send, _cnt); 
	
}
/**
 * @brief �����û��Զ�������
 */
void DataTrans_UserData(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // ����һ��������ʵ����ʹ����������������ת��Ϊ�ֽ�����
	uint8_t data_to_send[100] = {0}; // �����͵��ֽ�����
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x55;
	data_to_send[_cnt++]=0x04; 	// ����
	data_to_send[_cnt++]=16;		// ����
	
	uint8_t _start = _cnt;
		
	// ��Ҫ���͵����ݸ�ֵ���������float��Ա
	// ��Ӧ�ľ��ܸ����ֽ������Ա��ֵ
	_temp.data = kinematics.fb_wheel_rpm.motor_1;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_wheel_rpm.motor_2;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_wheel_rpm.motor_3;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	_temp.data = kinematics.fb_wheel_rpm.motor_4;
	data_to_send[_cnt++]=_temp.data8[0];
	data_to_send[_cnt++]=_temp.data8[1];
	data_to_send[_cnt++]=_temp.data8[2];
	data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	
	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  // ���ڷ���
	SendData(data_to_send, _cnt); 
	
}

/**
 * @brief ������̼�����
 */
void DataTrans_Odom(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // ����һ��������ʵ����ʹ����������������ת��Ϊ�ֽ�����
	uint8_t data_to_send[100] = {0}; // �����͵��ֽ�����
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x55;
	
	uint8_t _start = _cnt;
	
	float datas[] = {	kinematics.odom.vel.linear_x, 
										kinematics.odom.vel.linear_y, 
										kinematics.odom.vel.angular_z, 
										kinematics.odom.pose.theta
									};
	
	for(int i = 0; i < sizeof(datas) / sizeof(float); i++)
	{
		// ��Ҫ���͵����ݸ�ֵ���������float��Ա
		// ��Ӧ�ľ��ܸ����ֽ������Ա��ֵ
		_temp.data = datas[i];
		data_to_send[_cnt++]=_temp.data8[0];
		data_to_send[_cnt++]=_temp.data8[1];
		data_to_send[_cnt++]=_temp.data8[2];
		data_to_send[_cnt++]=_temp.data8[3]; // ���λ
	}
	
	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  // ���ڷ���
	SendData(data_to_send, _cnt); 
	
}

/**
 ************************ 
 ******* �������� ********
 ************************
 */

uint8_t data_receive[100];
uint8_t data_one_byte[1];
/**
 * @brief ���ڻص�����
 * @param  huart            ���ھ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		GetOneByte(data_one_byte[0]);
		// �����һ�ν��պ󣬴����жϻᱻ�رգ���Ҫ�ٴδ�
		HAL_UART_Receive_IT(&huart1, data_one_byte, 1);
	}	
	
	if(huart->Instance == LPUART1)
	{
		GetOneByte(data_one_byte[0]);
		// �����һ�ν��պ󣬴����жϻᱻ�رգ���Ҫ�ٴδ�
		HAL_UART_Receive_IT(&hlpuart1, data_one_byte, 1);
	}	
	
}

/**
 * @brief �Ӵ��ڶ�ȡ�����ֽ�
 * @param  data             ��ȡ���ֽ�����
 */
void GetOneByte(uint8_t data)
{
	static u8 state = 0;
	static u8 cnt = 0;
	if(state == 0 && data == 0xAA)
	{
		state++;
	}
	else if(state == 1 && data == 0x55)
	{
		state++;
	}
	else if(state == 2)
	{
		data_receive[cnt++] = data;
		if(cnt >= 13)
		{
			// У��
			u8 checkout = 0;
			for(int i = 0; i < cnt - 1; i++)
			{
				checkout += data_receive[i];
			}
			if(checkout == data_receive[cnt - 1])
			{
				// У��ͨ�������н���
				DataDecoder(data_receive);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
			
			state = 0;
			cnt = 0;
		}
	}
	else state = 0;
}

// ��Ƭ����λָ��
u8 reset_checkout[12] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC};
/**
 * @brief ���ݽ���
 * @param  data             ����������
 */
void DataDecoder(u8 *data)
{	
	if(flag.robot_sta == MODE_UART_CTRL)
	{
		data_u temp;
		
		temp.data8[0] = data[0];
		temp.data8[1] = data[1];
		temp.data8[2] = data[2];
		temp.data8[3] = data[3];
		kinematics.exp_vel.linear_x = temp.data;
		
		temp.data8[0] = data[4 + 0];
		temp.data8[1] = data[4 + 1];
		temp.data8[2] = data[4 + 2];
		temp.data8[3] = data[4 + 3];
		kinematics.exp_vel.linear_y = temp.data;
		
		temp.data8[0] = data[8 + 0];
		temp.data8[1] = data[8 + 1];
		temp.data8[2] = data[8 + 2];
		temp.data8[3] = data[8 + 3];
		kinematics.exp_vel.angular_z = temp.data;	
	}
	
	// ��λָ��У��
	for(int i = 0; i < 12; i++)
	{
		if(data[i] != reset_checkout[i])
			return;
	}
	HAL_NVIC_SystemReset();
}

