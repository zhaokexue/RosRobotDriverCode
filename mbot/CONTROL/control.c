#include "control.h"	
#include "pid.h"
#include "MPU6050.h"
#include "motor.h"
#include "adc.h"
#include "encoder.h"
#include "key.h"
#include "led.h"
#include "usart3.h"

int leftWheelEncoder       = 0;
int rightWheelEncoder      = 0;

int leftWheelEncoderNow    = 0;
int rightWheelEncoderNow   = 0;
int leftWheelEncoderLast   = 0;
int rightWheelEncoderLast  = 0;


int Voltage     = 0;    //电池电压采样相关的变量
int Voltage_Temp,Voltage_Count,Voltage_All;

float yaw  =0;         //转向陀螺仪

u8 Flag_Qian     =0;
u8 Flag_Hou      =0;
u8 Flag_Left     =0;
u8 Flag_Right    =0;
u8 Flag_sudu     =2;
u8 Flag_useApp   =0;    //蓝牙遥控相关的变量

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
	EXTI_ClearITPendingBit(EXTI_Line12);  //清除LINE12线路挂起位		
	
	leftWheelEncoder += getTIMx_DetaCnt(TIM4);
	rightWheelEncoder+= getTIMx_DetaCnt(TIM2);
	
	//记录本次左右编码器数据
	leftWheelEncoderNow   = leftWheelEncoder;
	rightWheelEncoderNow  = rightWheelEncoder;
		
	//10ms测速    	
	leftSpeedNow          = (leftWheelEncoderNow - leftWheelEncoderLast)*1000*100*0.000120830;  //
	rightSpeedNow         = (rightWheelEncoderNow - rightWheelEncoderLast)*1000*100*0.000181245;//

	//记录上次编码器数据
	leftWheelEncoderLast  = leftWheelEncoderNow;                    //===读取编码器的值
	rightWheelEncoderLast = rightWheelEncoderNow;                   //===读取编码器的值
	
//	Voltage_Temp=Get_battery_volt();		                            //=====读取电池电压		
//	Voltage_Count++;                                                    //=====平均值计数器
//	Voltage_All+=Voltage_Temp;                                          //=====多次采样累积
//	if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====求平均值	

//	key();
//	Led_Flash(200);                                                 //===LED闪烁;常规模式 1s改变一次指示灯的状态		
			
	App_main();
	Pid_Ctrl();
	Set_Pwm(motorLeft,motorRight); 
	
//	if(Turn_Off(Voltage)==0)                                         //===如果电压异常
//	{
//		Set_Pwm(motorLeft,motorRight);                                 //===赋值给PWM寄存器 		
//	}
//	else                                                             //这里不能上来就关pid因为，上来adc还没采集好数值
//	{
//		motorLeft=0;
//		motorRight=0;
//		AIN1=0;                                            
//		AIN2=0;
//		BIN1=0;
//		BIN2=0;
//	}   	
	return 0;	  
} 






