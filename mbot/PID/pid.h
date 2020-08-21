#ifndef __PID_H__
#define __PID_H__

#include "stm32f10x.h"

struct pid_uint
{
	s32 U_kk;    	    //上一次的输出量
	s32 ekk;		 	//上一次的输入偏差
	s32 ekkk;			//前一次的输入偏差
	s32 Ur;				//限幅输出值,需初始化
	s32 Kp;				//比例
	s32 Ki;				//积分
	s32 Kd;				//微分
	
	u8  En;             //开关
	s16 Adjust;         //调节量
	s16 speedSet;       //速度设置
	s16 speedNow;       //当前速度
};
/****************************外接函数***************************/

extern struct pid_uint pid_Task_Letf;
extern struct pid_uint pid_Task_Right;

void  PID_Init(void);
void  reset_Uk(struct pid_uint *p);
s32   PID_common(int set,int jiance,struct pid_uint *p);
void Pid_Ctrl(int *leftMotor,int  *rightMotor);

#endif //__PID_H__
