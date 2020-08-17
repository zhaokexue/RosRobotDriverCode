#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "stm32f10x.h"
#include "pid.h"

#define PI 3.14159265

extern int leftWheelEncoder;
extern int rightWheelEncoder;
       
extern int leftWheelEncoderNow;
extern int rightWheelEncoderNow;
extern int leftWheelEncoderLast;
extern int rightWheelEncoderLast;

extern int Voltage; //电池电压采样相关的变量

extern float yaw;         //转向陀螺仪

extern u8 Flag_Qian;
extern u8 Flag_Hou;
extern u8 Flag_Left;
extern u8 Flag_Right;
extern u8 Flag_sudu;
extern u8 Flag_useApp;    //蓝牙遥控相关的变量


void App_main(void);

#endif
