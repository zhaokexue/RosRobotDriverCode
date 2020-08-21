#include "pid.h"

/*===================================================================
程序功能：双路电机速度PID
程序编写：公众号：小白学移动机器人
其他    ：如果对代码有任何疑问，可以私信小编，一定会回复的。
=====================================================================
------------------关注公众号，获得更多有趣的分享---------------------
===================================================================*/

struct pid_uint pid_Task_Letf;
struct pid_uint pid_Task_Right;

/****************************************************************************
*函数名称：PID_Init(void)
*函数功能：初始化PID结构体参数
****************************************************************************/

void PID_Init(void)
{
//乘以1024原因避免出现浮点数运算，全部是整数运算，这样PID控制器运算速度会更快
/***********************左轮速度pid****************************/
	pid_Task_Letf.Kp = 1024 * 0.5;//0.4
 	pid_Task_Letf.Ki = 1024 * 0;	
	pid_Task_Letf.Kd = 1024 * 0.08; 
	pid_Task_Letf.Ur = 1024 * 4000;
	pid_Task_Letf.Adjust   = 0;
	pid_Task_Letf.En       = 1;
	pid_Task_Letf.speedSet = 0;
	pid_Task_Letf.speedNow = 0;
	reset_Uk(&pid_Task_Letf);		
/***********************右轮速度pid****************************/
	pid_Task_Right.Kp = 1024 * 0.35;//0.2
 	pid_Task_Right.Ki = 1024 * 0;	//不使用积分
	pid_Task_Right.Kd = 1024 * 0.06; 
	pid_Task_Right.Ur = 1024 * 4000;
	pid_Task_Right.Adjust   = 0;
	pid_Task_Right.En       = 1;
	pid_Task_Right.speedSet = 0;
	pid_Task_Right.speedNow = 0;
	reset_Uk(&pid_Task_Right);
}

/***********************************************************************************************
 函 数 名：void reset_Uk(PID_Uint *p)
 功    能：初始化U_kk,ekk,ekkk
 说    明：在初始化时调用，改变PID参数时有可能需要调用
 入口参数：PID单元的参数结构体 地址
************************************************************************************************/

void reset_Uk(struct pid_uint *p)
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

s32 PID_common(int set,int jiance,struct pid_uint *p)
{
	int ek=0,U_k=0;

	ek=jiance - set;                                                               
	
	U_k=p->U_kk + p->Kp*(ek - p->ekk) + p->Ki*ek + p->Kd*(ek - 2*p->ekk + p->ekkk);
	
	p->U_kk=U_k;
    p->ekkk=p->ekk;
	p->ekk=ek;
	
	if(U_k>(p->Ur))		                                    
		U_k=p->Ur;
	if(U_k<-(p->Ur))
		U_k=-(p->Ur);
	
	return U_k>>10; 
}

/***********************************************************************************
** 函数名称 ：void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
** 函数功能 ：pid选择函数	      
***********************************************************************************/

void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
{
	/**********************左轮速度pid*************************/
	if(pl->En == 1)
	{									
		pl->Adjust = -PID_common(pl->speedSet, pl->speedNow, pl);		
	}	
	else
	{
		pl->Adjust = 0;
		reset_Uk(pl);
		pl->En = 2; 
	}
	/***********************右轮速度pid*************************/
	if(pr->En == 1)
	{
		pr->Adjust = -PID_common(pr->speedSet, pr->speedNow, pr);		
	}	
	else
	{
		pr->Adjust = 0;
		reset_Uk(pr);
		pr->En = 2; 
	}
}

/*******************************************************************************
 * 函数名：Pid_Ctrl(int *leftMotor,int  *rightMotor)
 * 描述  ：Pid控制
 *******************************************************************************/

void Pid_Ctrl(int *leftMotor,int  *rightMotor)
{
	Pid_Which(&pid_Task_Letf, &pid_Task_Right); 
	*leftMotor  += pid_Task_Letf.Adjust;
	*rightMotor += pid_Task_Right.Adjust;
}

