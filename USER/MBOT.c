#include "stm32f10x.h"
#include "sys.h"
#include "mbotLinuxUsart.h"
#include "pid.h"	

u8 Way_Angle     =1;    //获取角度的算法，1：四元数
u8 Flag_Qian     =0;
u8 Flag_Hou      =0;
u8 Flag_Left     =0;
u8 Flag_Right    =0;
u8 Flag_sudu     =2;
u8 Flag_useApp   =0;    //蓝牙遥控相关的变量
u8 Flag_Stop     =1;    //停止标志位和默认停止
u8 Flag_Show     =0;    //显示标志位  显示打开

int Encoder_Left =0;
int Encoder_Right=0;    //左右编码器的脉冲计数
int Moto1        =0;
int Moto2        =0;    //电机PWM变量	

int Voltage      =0;    //电池电压采样相关的变量
float Gyro_Turn  =0;    //转向陀螺仪

u8 delay_value   =0;
u8 delay_flag    =0;    //延时和调参等变量 

int main(void)
{ 
	delay_init();	    	        //=====延时函数初始化	
	uart_init(115200);	            //=====串口初始化为  树莓派
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁用JTAG 启用 SWD
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	MY_NVIC_PriorityGroupConfig(2);	//=====设置中断分组
	MBOT_PWM_Init(7199,0);          //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口 
	uart3_init(9600);               //=====串口3初始化
	Encoder_Init_TIM2();            //=====编码器接口
	Encoder_Init_TIM4();            //=====初始化编码器2
	Adc_Init();                     //=====adc初始化
	IIC_Init();                     //=====IIC初始化   读取MPU6050数据
	MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //=====初始化DMP 
	PID_Init();                     //=====PID初始化
	KEY_Init();
	MBOT_EXTI_Init();               //=====MPU6050 5ms定时中断初始化

	while(1)
	{
		//蓝牙调试时用，不调试注释
		//APP_Show();
		//给树莓派发送速度，角度,这里速度已经乘以1000
		usartSendSpeed((float)leftSpeedNow,(float)rightSpeedNow,(float)(int)Gyro_Turn);
		delay_flag=1;	
		delay_value=0;
		while(delay_flag);	        //通过MPU6050的INT中断实现的15ms精准延时	
	} 
}

