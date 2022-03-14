#include "LED.h"
#include "MATH.h"

u16 led_accuracy = 20;//该时间应与LED_Duty()调用周期相同
float LED_Brightness = 20;


//LED的1ms驱动，在1ms定时中断里调用。
void LED_1ms_DRV(void)
{
	static u16 led_cnt;
	{
			
		if( led_cnt < LED_Brightness )
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
		}
		
		if(++led_cnt >= led_accuracy)
		{
			led_cnt = 0;
		}
	}
	

}

static void ledOnOff(u8 led)
{
	if(led)
		LED_Brightness = 20;
	else
		LED_Brightness = 0;
}


#include "Sensor_Basic.h"
void LED_Task(u8 dT_ms)
{
	static u16 timtmp = 0;
	
	u8 flashtims;
	static u8 cnttmp = 0;
	
	static u8  statmp = 0;
	static u8  modtmp = 0;
	
	if(statmp == 0)			//显示机器人状态
	{
		if(modtmp < 3 - flag.low_power)//flag.robot_sta) // 闪1次：停止状态 闪2次：工作状态
		{
			if(timtmp < 60)
			{
				ledOnOff(1);
			}
			else
				ledOnOff(0);
			timtmp += dT_ms;
			
			if(timtmp > 200)
			{
				timtmp = 0;
				modtmp++;
			}
		}
		else
		{
			modtmp = 0;
			statmp = 1;
		}
	}
	
	else		//每一轮循环后，加一个灭灯长延时
	{
		ledOnOff(0);
		timtmp += dT_ms;
		if(timtmp > 1000)
		{
			timtmp = 0;
			statmp = 0;
			modtmp = 0;
		}
	}
}

