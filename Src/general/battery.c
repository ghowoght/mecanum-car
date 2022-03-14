#include "battery.h"
#include "Sensor_Basic.h"

#define V_RATE (2.5 + 10) / 2.5 // ½µÑ¹±ÈÀý

uint32_t ADC_Value[50]; 

void Battery_Init(){
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)&ADC_Value,50); 
}



void Battery_Task(uint32_t dT_ms){
	
	uint32_t v = 0;
	for(int i = 0; i < 50; i++){
		v += ADC_Value[i];
	}
	
	float voltage = v / 4096.0f * 3.3f / 50.0f * V_RATE;
	
	if(voltage < 10.5f)
		flag.low_power = 1;
	else
		flag.low_power = 0;
	
//	printf("%f\r\n", voltage);
	
}
