#ifndef __KEY_H
#define __KEY_H	 

#include "stm32f10x.h"

#define KEY GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)

void KEY_Init(void);          //按键初始化

u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);          //长按扫描  

void key(void);

#endif  
