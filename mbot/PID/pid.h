#ifndef __PID_H__
#define __PID_H__

#include <sys.h>	
#define PID_Uint struct pid_uint

PID_Uint
{
	s32 U_kk;    	    //上一次的输出量
	s32 ekk;		 	//上一次的输入偏差
	s32 ekkk;			//前一次的输入偏差
	s32 Ur;				//限幅输出值,需初始化
	s32 Kp;				//比例
	s32 Ki;				//积分
	s32 Kd;				//微分
	s32 Umax;
	s32 Umin;
};

/****************************外接函数***************************/

void  PID_Init(void);
void  reset_Uk(PID_Uint *p);
s32   PID_common(int set,int jiance,PID_Uint *p);
void  Pid_Ctrl(void);

/*****************************外接变量***************************/
extern s32       GYRO_Ang;
extern u8	     g_Pid_En;
extern u8	     g_Pid_Angle_En;             //角度PID开关
extern s16       g_Pid_Left_Adjust;		     //左轮速度调节量
extern s16       g_Pid_Right_Adjust;		 //右轮速度调节量
extern s16       g_Pid_Angle_Adjust;		 //角度调节量，作用就是一边加一边减
extern PID_Uint  pid_Task_A_1_1;
//速度
extern int    leftSpeedNow; 
extern int    rightSpeedNow; 
//乘以1000之后的速度设定值
extern int       leftSpeedSet; 
extern int       rightSpeedSet; 
extern int       angleSet;//角度设定值
#endif //__PID_H__
