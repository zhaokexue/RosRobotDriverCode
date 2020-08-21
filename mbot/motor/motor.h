#ifndef __MOTOR_H
#define __MOTOR_H

#include <sys.h>	 
#include "pid.h"
#include "control.h"

extern int    motorLeft,motorRight;                      //电机PWM变量
//速度
extern int    leftSpeedNow; 
extern int    rightSpeedNow; 
//乘以1000之后的速度设定值
extern int    leftSpeedSet; 
extern int    rightSpeedSet; 

extern u8 Flag_Qian;
extern u8 Flag_Hou;
extern u8 Flag_Left;
extern u8 Flag_Right;
extern u8 Flag_sudu;
extern u8 Flag_useApp;       //蓝牙遥控相关的变量

#define PWMA   TIM1->CCR1    //PA8
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define BIN1   PBout(13)
#define BIN2   PBout(12)
#define PWMB   TIM1->CCR4    //PA11

void Set_Pwm(int moto1,int moto2);
u8 Turn_Off(int voltage);
void App_main(void);
int myabs(int a);


#endif

