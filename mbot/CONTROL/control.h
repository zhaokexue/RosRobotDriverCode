#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "stm32f10x.h"

extern int Voltage;                 //电池电压采样相关的变量
extern float yaw;                   //转向陀螺仪
extern float yaw_acc_error;         //yaw累积误差

#endif
