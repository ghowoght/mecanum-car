#include "Scheduler.h"
#include "IMU.h"
#include "Sensor_Basic.h"
#include "dt.h"
#include "motor.h"
#include "stdio.h"
#include "UserCtrl.h"
#include "LED.h"
#include "ppm.h"
#include "battery.h"

static u8 lt0_run_flag;

/**
 * @brief ICM20602的1ms中断函数
 */
void INT_1ms_Task()
{	
	//标记1ms执行
	lt0_run_flag ++;
}

/**
 * @brief 1ms任务函数
 */
static void Loop_Task_0()
{
	/* 传感器数据读取 */
	Sensor_Get();
	
	/* 惯性传感器数据准备 */
	Sensor_Data_Prepare(1);
	
	/* 姿态解算更新 */
	ImuUpdate_Task(1);
	
	/* 数据传输任务 */
	DataTrans_Task(1);
	
	/* LED驱动任务 */
	LED_1ms_DRV();
	
	
	
}

/**
 * @brief 2ms任务函数
 * @param  dT_us            执行周期
 */
static void Loop_Task_1(u32 dT_us)
{
	
}

/**
 * @brief 6ms任务函数
 * @param  dT_us            执行周期
 */
static void Loop_Task_2(u32 dT_us)
{
	/* 获取姿态角(ROLL PITCH YAW) */
//	calculate_RPY();
}

/**
 * @brief 11ms任务函数
 * @param  dT_us            执行周期
 */
static void Loop_Task_5(u32 dT_us)
{	
	/* LED任务 */
	LED_Task(11);
	
}

/**
 * @brief 20ms任务函数
 * @param  dT_us            执行周期
 */
uint64_t start = 0;
static void Loop_Task_8(u32 dT_us)
{
	/* 电机控制任务 */
	Motor_Task(20e3);
	
//	printf("rol: %10f pit: %10f yaw: %10f\r\n", imu_data.rol, imu_data.pit, imu_data.yaw);
}

/**
 * @brief 50ms任务函数
 * @param  dT_us            执行周期
 */
static void Loop_Task_9(u32 dT_us)	//50ms执行一次
{
	u32 dT_ms = 50; //dT_us * 1e03;
	
	/* 用户控制任务 */
//	Ctrl_Task(dT_ms);
	
	/* 遥控任务 */
	RemoteCtrl_Task(50);
	
	/* 发送里程计数据 */
//	DataTrans_Odom();
	
	/* 电池电压测量任务 */
	Battery_Task(50);
}

/**
 ************************ 
 ***** 任务调度系统 *****
 ************************
 */

//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] = 
{
	//任务n,    周期us,   上次时间us
//	{Loop_Task_0 ,  1000,  0 },
	{Loop_Task_1 ,  2000,  0 },
	{Loop_Task_2 ,  6000,  0 },
//	{Loop_Task_2 ,  2500,  0 },
//	{Loop_Task_3 ,  2500,  0 },
//	{Loop_Task_4 ,  2500,  0 },
	{Loop_Task_5 ,  11000,  0 },
//	{Loop_Task_6 ,  9090,  0 },
//	{Loop_Task_7 ,  9090,  0 },
	{Loop_Task_8 , 20000,  0 },
	{Loop_Task_9 , 50000,  0 },
//	{Loop_Task_10,100000,  0 },
};

//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

/**
 * @brief 任务调度函数
 * @return u8 
 */
u8 Main_Task(void)
{
	uint8_t index = 0;
	
	//查询1ms任务是否需要执行
	if(lt0_run_flag!=0)
	{
		lt0_run_flag--;
		Loop_Task_0();
	}
	
	//循环判断其他所有线程任务，是否应该执行
	uint64_t time_now;
	int delta_time_us;
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位us
		time_now = GetSysRunTimeUs();
		delta_time_us = (int)(time_now - sched_tasks[index].last_run);
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(delta_time_us >= sched_tasks[index].interval_ticks)
		{
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = time_now;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func(delta_time_us);
		}	 
	}
	return 0;
}

volatile uint64_t SysRunTimeMs = 0; // 当前系统运行时间/ms

/**
 * @brief 滴答定时器回调函数，触发周期：1ms
 */
void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
	SysRunTimeMs++;
}
/**
 * @brief 获取当前系统运行时间
 * @return uint32_t 
 */
uint32_t GetSysRunTimeUs(void)
{
		return SysRunTimeMs * 1000U + (SysTick->LOAD - SysTick->VAL) * 1000U / SysTick->LOAD;
}	

