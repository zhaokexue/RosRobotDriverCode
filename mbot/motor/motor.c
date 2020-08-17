#include "motor.h"

int motorLeft   =0;
int motorRight  =0;         //左右轮电机PWM变量

//乘以1000之后的速度实时值
int leftSpeedNow  = 0; 
int rightSpeedNow = 0; 
//乘以1000之后的速度设定值
int leftSpeedSet  = 0; 
int rightSpeedSet = 0; 

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motorLeft,int motorRight)
{
	if(motorLeft>0)     AIN2=0, AIN1=1;
	else 	            AIN2=1,	AIN1=0;
	PWMA=myabs(motorLeft);
	if(motorRight>0)	BIN1=0,	BIN2=1;
	else                BIN1=1,	BIN2=0;
	PWMB=myabs(motorRight);	
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(int voltage)
{
	u8 temp=0;
	if(voltage<1110)//电池电压低于11.1V关闭电机
	{	                                             
		temp=1;                                        
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
	}
	else
		temp=0;
	return temp;			
}

/**************************************************************************
函数功能：APP指令判断
入口参数：
返回  值：
Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0
**************************************************************************/
void App_main()
{
	if(Flag_useApp==1)
	{
		if(Flag_Qian==1)
		{
			if(Flag_sudu==2)//默认低速
			{
				leftSpeedSet  = 200; 
				rightSpeedSet = 200; 			
			}
			else if(Flag_sudu==1)//高速
			{
				leftSpeedSet  = 350; 
				rightSpeedSet = 350; 			
			}		
		}
		else if(Flag_Hou==1)
		{

			if(Flag_sudu==2)//默认低速
			{
				leftSpeedSet  = -200; 
				rightSpeedSet = -200; 			
			}
			else if(Flag_sudu==1)//高速
			{
				leftSpeedSet  = -350; 
				rightSpeedSet = -350; 				
			}	
		}
		else if(Flag_Left==1)
		{

			if(Flag_sudu==2||Flag_sudu==1)//默认低速
			{
				leftSpeedSet  = -100; 
				rightSpeedSet = 100;			
			}
		}
		else if(Flag_Right==1)
		{

			if(Flag_sudu==2||Flag_sudu==1)//默认低速
			{
				leftSpeedSet  = 100; 
				rightSpeedSet = -100;			
			}
		}
		else
		{
			leftSpeedSet  = 0; 
			rightSpeedSet = 0; 
		}	
	}
}


/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  
	  temp=-a;  
	else 
	  temp=a;
	return temp;
}

