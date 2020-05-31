#include "control.h"	
#include "pid.h"

#define GYRO_Z_OFFSET 9

u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All;

/**************************************************************************
函数功能：所有的控制代码都在这里面
          5ms定时中断由MPU6050的INT引脚触发
          严格保证采样和数据处理的时间同步		
		//计算左右车轮线速度，正向速度为正值 ，反向速度为负值，速度为乘以1000之后的速度 mm/s
		//一定时间内的编码器变化值*转化率（转化为直线上的距离m）*100s（10ms计算一次） 得到 m/s *1000转化为int数据

		一圈的脉冲数：
			左：1560
			右：1040
		轮子半径：0.03m
		轮子周长：2*pi*r
		一个脉冲的距离：
			左：0.000120830m
			右：0.000181245m
		速度分辨率：
			左：0.0120m/s 12.0mm/s
			右：0.0181m/s 18.1mm/s
						
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	if(INT==0)		
	{   
		EXTI->PR=1<<12;                                                         //清除中断标志位   
		
		Flag_Target=!Flag_Target;
		if(delayFlag==1)
		{
			if(++delayValue==3)	 delayValue=0,delayFlag=0;                //5 //给主函数提供15ms的精准延时
		}
		if(Flag_Target==1)                                                      //5ms读取一次陀螺仪和加速度计的值
		{		
			Voltage_Temp=Get_battery_volt();		                            //=====读取电池电压		
			Voltage_Count++;                                                    //=====平均值计数器
			Voltage_All+=Voltage_Temp;                                          //=====多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====求平均值		
			return 0;	                                               
		}                                                                       //10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
		leftWheelEncoder += getTIMx_DetaCnt(TIM4);
		rightWheelEncoder+= getTIMx_DetaCnt(TIM2);
		
		//记录本次左右编码器数据
		leftWheelEncoderNow   = leftWheelEncoder;
		rightWheelEncoderNow  = rightWheelEncoder;
			
		//10ms测速    	
		leftSpeedNow          = (leftWheelEncoderNow - leftWheelEncoderLast)*1000*100*0.000120830;//
		rightSpeedNow         = (rightWheelEncoderNow - rightWheelEncoderLast)*1000*100*0.000181245;//

		//记录上次编码器数据
		leftWheelEncoderLast  = leftWheelEncoderNow;                    //===读取编码器的值
		rightWheelEncoderLast = rightWheelEncoderNow;                   //===读取编码器的值
		
		getAngle();                                                     //===更新姿态	
		key();
		Led_Flash(200);                                                 //===LED闪烁;常规模式 1s改变一次指示灯的状态		
		if((int)yaw==0) 
			Led_Flash(30); 				

		App_main();
		Pid_Ctrl();
		Xianfu_Pwm();

		if(Turn_Off(Voltage)==0)                                         //===如果电压异常
		{
			Set_Pwm(motorLeft,motorRight);                                 //===赋值给PWM寄存器 		
		}
		else                                                             //这里不能上来就关pid因为，上来adc还没采集好数值
		{
			motorLeft=0;
			motorRight=0;
			AIN1=0;                                            
			AIN2=0;
			BIN1=0;
			BIN2=0;
		}
	}       	
	 return 0;	  
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
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motorLeft,int motorRight)
{
	if(motorLeft>0) AIN2=0,			AIN1=1;
	else 	    AIN2=1,			AIN1=0;
	PWMA=myabs(motorLeft);
	if(motorRight>0)	BIN1=0,			BIN2=1;
	else        BIN1=1,			BIN2=0;
	PWMB=myabs(motorRight);	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Amplitude=6900;  //===PWM满幅是7200 限制在6900
	if(motorLeft<-Amplitude) motorLeft=-Amplitude;	
	if(motorLeft>Amplitude)  motorLeft=Amplitude;	
	if(motorRight<-Amplitude) motorRight=-Amplitude;	
	if(motorRight>Amplitude)  motorRight=Amplitude;		
}
/**************************************************************************
函数功能：key给定速度，测试PID
入口参数：无
返回  值：无
**************************************************************************/
void key(void)
{	
	u8 tmp=0;
	tmp=click_N_Double(50); //单击控制小车的启停
	if(tmp==1)
	{
		leftSpeedSet  = 500; 
		rightSpeedSet = 500; 
		
	}
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
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：无
返回  值：无
**************************************************************************/
void getAngle()
{
		Read_DMP();                      //===读取加速度、角速度、倾角
		if(Yaw<-GYRO_Z_OFFSET)
			Yaw=Yaw+360;
		yaw=GYRO_Z_OFFSET+Yaw;     //===更新转向角度
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



