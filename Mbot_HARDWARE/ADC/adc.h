#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h" 

//Í¨µÀ6
#define Battery_Ch 6

void MY_ADC_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
void Get_battery_volt_average(int *Voltage,int time);

#endif 















