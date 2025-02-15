/**
 * @file dt.c
 * @brief 数据传输程序
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020  WHU-EIS
 * 
 */
#include "dt.h"
#include "Imu.h"
#include "Sensor_Basic.h"
#include "motor.h"
#include "Sensor_Basic.h"
#include "stdio.h"

/**
 ************************ 
 ******* 发送数据 ********
 ************************
 */

#define USE_USB

/**
 * @brief 数据发送函数
 * @param  data_to_send     待发送的字节数组
 * @param  cnt              待发送数据长度
 */
void SendData(u8 *data_to_send, u8 cnt)
{
#ifdef USE_USB
	/* 关闭串口接收中断 */
	HAL_NVIC_DisableIRQ(LPUART1_IRQn);
	/* 发送数据 */
	HAL_UART_Transmit(&hlpuart1, data_to_send, cnt, 0xFFFF); 
	/* 开启串口接收中断 */
	HAL_NVIC_EnableIRQ(LPUART1_IRQn);
#else
	/* 关闭串口接收中断 */
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	/* 发送数据 */
	HAL_UART_Transmit(&huart1, data_to_send, cnt, 0xFFFF); 
	/* 开启串口接收中断 */
	HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
}

// 联合体，用于float型和字节数组的互换
typedef union
{
	float data;
	uint8_t data8[4];
} data_u;

/**
 * @brief 数据发送任务
 * @param  dT_ms            执行周期
 */
void DataTrans_Task(u32 dT_ms)
{
	static u32 cnt = 0;
	const u32 sent_imu_cnt 	 = 1;
	const u32 sent_odom_cnt 	 = 20;
	const u32 sent_userdata_cnt 	 = 40;
	const u32 sent_wheel_cnt = 30;
	
	cnt += dT_ms;

	// if((cnt % sent_odom_cnt) == sent_odom_cnt - 1)
	// {
	// 	DataTrans_Odom();
	// }
	// else if((cnt % sent_userdata_cnt) == sent_userdata_cnt - 1)
	// {
	// 	DataTrans_UserData();
	// }
	if((cnt % sent_imu_cnt) == sent_imu_cnt - 1)
	{
		DataTrans_IMU_Raw();
	}
	
	if(cnt>1200) cnt = 0;
}

/**
 * @brief 发送IMU原始数据, 包含里程计数据
 */
void DataTrans_IMU_Raw(void)
{
	uint8_t _cnt = 0;
	data_u _temp;
	uint8_t data_to_send[100] = {0};
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x55;
	
	uint8_t _start = _cnt;

	// IMU数据序号
	for(int i = 0; i < 4; i++){
		data_to_send[_cnt++] = *((uint8_t*)(&imu_data_cnt) + i);
	}
	
	// IMU数据打包
	for(int i = 0; i < 6; i++){
		data_to_send[_cnt++] = *((uint8_t*)sensor.Gyro_Original + i);
	}
	for(int i = 0; i < 6; i++){
		data_to_send[_cnt++] = *((uint8_t*)sensor.Acc_Original + i);
	}
	
	// 里程计数据打包
	for(int i = 0; i < 8; i++){
		data_to_send[_cnt++] = *((uint8_t*)&sensor.encoder_incre + i);
	}

	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
	// 串口发送
	SendData(data_to_send, _cnt); 
}
/**
 * @brief 发送用户自定义数据
 */
void DataTrans_UserData(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // 声明一个联合体实例，使用它将待发送数据转换为字节数组
	uint8_t data_to_send[100] = {0}; // 待发送的字节数组
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x55;
	data_to_send[_cnt++]=0x04; 	// 类型
	data_to_send[_cnt++]=16;		// 长度
	
	uint8_t _start = _cnt;

	float datas[] = {	kinematics.fb_wheel_rpm.motor_1,
						kinematics.fb_wheel_rpm.motor_2,
						kinematics.fb_wheel_rpm.motor_3,
						kinematics.fb_wheel_rpm.motor_4
					};
	
	for(int i = 0; i < sizeof(datas) / sizeof(float); i++)
	{
		// 将要发送的数据赋值给联合体的float成员
		// 相应的就能更改字节数组成员的值
		_temp.data = datas[i];
		data_to_send[_cnt++]=_temp.data8[0];
		data_to_send[_cnt++]=_temp.data8[1];
		data_to_send[_cnt++]=_temp.data8[2];
		data_to_send[_cnt++]=_temp.data8[3]; // 最高位
	}
		
	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  // 串口发送
	SendData(data_to_send, _cnt); 
	
}

/**
 * @brief 发送里程计数据
 */
void DataTrans_Odom(void)
{
	uint8_t _cnt = 0;
	data_u _temp; // 声明一个联合体实例，使用它将待发送数据转换为字节数组
	uint8_t data_to_send[100] = {0}; // 待发送的字节数组
	
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
		// 将要发送的数据赋值给联合体的float成员
		// 相应的就能更改字节数组成员的值
		_temp.data = datas[i];
		data_to_send[_cnt++]=_temp.data8[0];
		data_to_send[_cnt++]=_temp.data8[1];
		data_to_send[_cnt++]=_temp.data8[2];
		data_to_send[_cnt++]=_temp.data8[3]; // 最高位
	}
	
	uint8_t checkout = 0;
	for(int i = _start; i < _cnt; i++)
	{
		checkout += data_to_send[i];
	}
	data_to_send[_cnt++] = checkout;
  	// 串口发送
	SendData(data_to_send, _cnt); 
}

/**
 ************************ 
 ******* 接收数据 ********
 ************************
 */

uint8_t data_receive[100];
uint8_t data_one_byte[1];
/**
 * @brief 串口回调函数
 * @param  huart            串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		GetOneByte(data_one_byte[0]);
		// 在完成一次接收后，串口中断会被关闭，需要再次打开
		HAL_UART_Receive_IT(&huart1, data_one_byte, 1);
	}	
	
	if(huart->Instance == LPUART1)
	{
		GetOneByte(data_one_byte[0]);
		// 在完成一次接收后，串口中断会被关闭，需要再次打开
		HAL_UART_Receive_IT(&hlpuart1, data_one_byte, 1);
	}	
	
}

/**
 * @brief 从串口读取单个字节
 * @param  data             读取的字节数据
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
			// 校验
			u8 checkout = 0;
			for(int i = 0; i < cnt - 1; i++)
			{
				checkout += data_receive[i];
			}
			if(checkout == data_receive[cnt - 1])
			{
				// 校验通过，进行解码
				DataDecoder(data_receive);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
			
			state = 0;
			cnt = 0;
		}
	}
	else state = 0;
}

// 单片机复位指令
u8 reset_checkout[12] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC};
/**
 * @brief 数据解码
 * @param  data             待解码数组
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
	
	// 复位指令校验
	for(int i = 0; i < 12; i++)
	{
		if(data[i] != reset_checkout[i])
			return;
	}
	HAL_NVIC_SystemReset();
}

