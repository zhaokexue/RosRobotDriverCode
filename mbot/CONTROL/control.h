#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "sys.h"

#define PI 3.14159265

int EXTI15_10_IRQHandler(void);
void Set_Pwm(int moto1,int moto2);
void Xianfu_Pwm(void);
u8 Turn_Off(int voltage);
void key(void);
void getAngle(void);
int myabs(int a);
void App_main(void);

#endif
