#include "control.h"	
#include "pid.h"
#include "motor.h"
#include "adc.h"
#include "encoder.h"
#include "led.h"

int   Voltage    =0;       //电池电压采样相关的变量
float yaw        =0;       //转向陀螺仪

/**************************************************************************
函数功能：所有的控制代码都在这里面
          5ms定时中断由MPU6050的INT引脚触发		
**************************************************************************/
void EXTI15_10_IRQHandler(void) 
{                                                         
	EXTI_ClearITPendingBit(EXTI_Line12);                            //===清除LINE12线路挂起位		
	
	Led_Flash(200);                                                 //===LED闪烁，证明程序正常运行	
	
	Get_battery_volt_average(&Voltage,100);		                    //===电池电压测量，单位mv，100次取一次平均
	
	Get_Motor_Speed(&leftSpeedNow,&rightSpeedNow);                  //===获取左右轮子真实速度
	
	App_main();                                                     //===手机app控制
	
	Pid_Ctrl();                                                     //===轮子速度PID控制

	if(Turn_Off(Voltage)==0)                                        //===如果电压异常
	{
		Set_Pwm(motorLeft,motorRight);                              //===赋值给PWM寄存器 		
	} 	
} 








