#include "pid.h"

s32      GYRO_Ang              = 0;

u8	     g_Pid_En  		       = 1;          //pid开关
u8	     g_Pid_Angle_En  	   = 0;          //角度PID开关
s16      g_Pid_Left_Adjust     = 0;		     //左轮速度调节量
s16      g_Pid_Right_Adjust    = 0;		     //右轮速度调节量
s16      g_Pid_Angle_Adjust    = 0;		     //角度调节量，作用就是一边加一边减

PID_Uint    pid_Task_Letf;
PID_Uint    pid_Task_Right;
PID_Uint    pid_Task_A_1_1;		

//乘以1000之后的速度实时值
int      leftSpeedNow  = 0; 
int      rightSpeedNow = 0; 
//乘以1000之后的速度设定值
int      leftSpeedSet  = 0; 
int      rightSpeedSet = 0; 
int      angleSet      = 0;//角度设定值

/****************************************************************************
*函数名称：PID_Init(void)
*函数功能：初始化PID结构体参数
*输入值	 ：无
*返回值  ：无
****************************************************************************/
void PID_Init(void)
{
//乘以1024原因避免出现浮点数运算，全部是整数运算，这样PID控制器运算速度会更快
/***********************左轮速度pid****************************/
	pid_Task_Letf.Kp = 1024 * 0.5;//0.4
 	pid_Task_Letf.Ki = 1024 * 0;	
	pid_Task_Letf.Kd = 1024 * 0.08; 
	pid_Task_Letf.Ur = 1024 * 4000;
	pid_Task_Letf.Umax=1024 * 400;
	pid_Task_Letf.Umin=1024 *(-200);
	reset_Uk(&pid_Task_Letf);		
/***********************右轮速度pid****************************/
	pid_Task_Right.Kp = 1024 * 0.35;//0.2
 	pid_Task_Right.Ki = 1024 * 0;	//不使用积分
	pid_Task_Right.Kd = 1024 * 0.06; 
	pid_Task_Right.Ur = 1024 * 4000;
	pid_Task_Right.Umax=1024 *(400);
	pid_Task_Right.Umin=1024 *(-200);
	reset_Uk(&pid_Task_Right);
	
/***********************角度pid****************************/
	pid_Task_A_1_1.Kp = 1024 * 3.0;
 	pid_Task_A_1_1.Ki = 1024 * 100000;	
	pid_Task_A_1_1.Kd = 1024 * 2.0; 
	pid_Task_A_1_1.Ur = 1024 * 45;

	reset_Uk(&pid_Task_A_1_1);
}

/***********************************************************************************************
 函 数 名：void reset_Uk(PID_Uint *p)
 功    能：初始化U_kk,ekk,ekkk
 说    明：在初始化时调用，改变PID参数时有可能需要调用
 入口参数：PID单元的参数结构体 地址
 返 回 值：无
************************************************************************************************/
void reset_Uk(PID_Uint *p)
{
	p->U_kk=0;
	p->ekk=0;
	p->ekkk=0;
}
/***********************************************************************************************
 函 数 名：s32 PID_commen(int set,int jiance,PID_Uint *p)
 功    能：PID计算函数
 说    明：求任意单个PID的控制量
 入口参数：期望值，实测值，PID单元结构体
 返 回 值：PID控制量
************************************************************************************************/
s32 PID_common(int set,int jiance,PID_Uint *p)
{
	int ek=0,U_k=0;

	ek=jiance - set;                                        //差值
	
	U_k=p->U_kk + p->Kp*(ek - p->ekk) + p->Ki*ek + p->Kd*(ek - 2*p->ekk + p->ekkk);
	
	p->U_kk=U_k;
    p->ekkk=p->ekk;
	p->ekk=ek;
	
	if(U_k>(p->Ur))		                                    
		U_k=p->Ur;
	if(U_k<-(p->Ur))
		U_k=-(p->Ur);
	
	return U_k>>10; //
}

/***********************************************************************************
** 函数名称 ：void Pid_Which(PID_TYPE *px, PID_TYPE *py, PID_TYPE *pa)
** 函数功能 ：pid选择函数	      
** 入口参数 ：角度pid参数、				      
** 出口参数 ：无						     
** 说明     ：
***********************************************************************************/
void Pid_Which(PID_Uint *px, PID_Uint *py, PID_Uint *pa)
{
	/**********************左轮速度pid*************************/
	if(g_Pid_En == 1)
	{									
		g_Pid_Left_Adjust = -PID_common(leftSpeedSet, leftSpeedNow, px);		
	}	
	else
	{
		g_Pid_Left_Adjust = 0;
		reset_Uk(px);
		g_Pid_En = 2; 
	}
	/***********************右轮速度pid*************************/
	if(g_Pid_En == 1)
	{
		g_Pid_Right_Adjust = -PID_common(rightSpeedSet, rightSpeedNow, py);		
	}	
	else
	{
		g_Pid_Right_Adjust = 0;
		reset_Uk(py);
		g_Pid_En = 2; 
	}
	/***********************角度pid*************************/
	if(g_Pid_Angle_En == 1)
	{
		g_Pid_Angle_Adjust = -PID_common(angleSet, (int)Gyro_Turn, pa);		
	}	
	else
	{
		g_Pid_Angle_Adjust = 0;
		reset_Uk(pa);
		g_Pid_Angle_En = 2; 
	}
}

/*******************************************************************************
 *
 * 函数名：Pid_Ctrl(void)
 * 描述  ：Pid控制
 * 输入  ：无
 * 输出  ：无
 * 说明  ：pid计算，并向2个轮子发速度
 *
 *******************************************************************************/
void Pid_Ctrl(void)
{
	if(g_Pid_En == 1)
	{
		Pid_Which(&pid_Task_Letf, &pid_Task_Right, &pid_Task_A_1_1); 
		Moto1 += g_Pid_Left_Adjust;
		Moto2 += g_Pid_Right_Adjust;
	}
}


