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
};

/****************************外接函数***************************/

void  PID_Init(void);
void  reset_Uk(struct pid_uint *p);
s32   PID_common(int set,int jiance,struct pid_uint *p);
void  Pid_Ctrl(void);

/*****************************外接变量***************************/
extern u8	  g_Pid_En;
extern s16    g_Pid_Left_Adjust;		 //左轮速度调节量
extern s16    g_Pid_Right_Adjust;		 //右轮速度调节量

#endif //__PID_H__
