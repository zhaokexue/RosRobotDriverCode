#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h" 

#define Battery_Ch 6

void MY_ADC_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   

#endif 















