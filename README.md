### 2.6 ROS小车STM32底层控制代码

经过之前几篇文章，我相信大家一定对下面这些模块都已经有些了解了。今天，我们将编写ROS小车STM32单片机底层控制的最后一篇文章。我们将一起按照之前的软件流程，将所有的代码整合到一个工程中去。

**在公众号：小白学移动机器人，发送：ROS小车底层**，即可获得源码工程文件。

![image-20200821000807464](https://imgconvert.csdnimg.cn/aHR0cHM6Ly9pLmxvbGkubmV0LzIwMjAvMDgvMjEvemRPRGs1QWpXVnFKbjc5LnBuZw?x-oss-process=image/format,png#pic_center)

相信大家对配置代码都很清楚了，这里直接描述软件执行流程，以及部分代码展示

**注意：该工程文件中的STM32与ROS通信的程序和前面写的文章提供的程序，有一点出入，之前是为了方便理解。大家调用的时候注意即可。**

#### 2.6.1 软件执行流程

实际上大家都知道，我们的程序中，除了main.c的while外，还有串口1接收中断服务函数，外部触发中断服务函数。他们分别执行了一些软件流程。

**（1）while中的流程功能**：完成向ROS发送左右轮速、航向角、控制位；调试；获取MPU6050的yaw值。

**（2）串口1中断服务函数流程功能**：接收从ROS发送过来的，对左右电机的控制数据、控制位。

**（3）外部触发中断服务函数流程功能**：程序灯、yaw温漂累积、电池电压测量（可以没有）、电机测速、电机PID速度控制、设置PWM

整体的软件流程就是这个样子，感觉已经描述的十分清楚了，具体的看代码。

#### 2.6.2 代码

（1）主程序以及串口1的中断服务函数

```c
#include "sys.h"

//====================自己加入的头文件===============================
#include "delay.h"
#include "led.h"
#include "myexti.h"
#include "adc.h"
#include "pwm.h"
#include "encoder.h"
#include "usart1.h"	
#include "usart3.h"
#include "ioi2c.h"
#include "mpu6050.h"

#include "show.h"					
#include "mbotLinuxUsart.h"
#include "pid.h"
#include "control.h"
#include "motor.h"

/*===================================================================
程序功能：ROS小车底层代码（全部）
程序编写：公众号：小白学移动机器人
其他    ：如果对代码有任何疑问，可以私信小编，一定会回复的。
=====================================================================
------------------关注公众号，获得更多有趣的分享---------------------
===================================================================*/
int main(void)
{ 
	//发送计数
	char sendCount=0;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁用JTAG 启用 SWD
	
	MY_NVIC_PriorityGroupConfig(2);	//=====设置中断分组
	
	delay_init();	    	        //=====延时函数初始化
	
	LED_Init();                     //=====LED初始化    程序灯	
	
	usart1_init(115200);	        //=====串口1初始化  树莓派
	usart3_init(9600);              //=====串口3初始化  蓝牙

	IIC_Init();                     //=====IIC初始化    读取MPU6050数据
	MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //=====初始化DMP 

	Encoder_Init_TIM2();            //=====初始化编码器1接口
	Encoder_Init_TIM4();            //=====初始化编码器2接口
	
	MY_ADC_Init();                  //=====adc初始化    电池电量检测
	
	Motor_Init(7199,0);             //=====初始化PWM 10KHZ，用于驱动电机 如需初始化驱动器接口
	PID_Init();                     //=====PID初始化
	
	MBOT_EXTI_Init();               //=====MPU6050 5ms定时中断初始化

	while(1)
	{
		//给树莓派发送速度，角度,这里速度已经乘以1000
		if(sendCount==0)//发送  14.4ms  发送一次数据 70Hz 左右
		{
			//发送需要一定的延时
			usartSendData(USART1,(short)leftSpeedNow,(short)rightSpeedNow,(short)yaw,sendCtrlFlag);  //1ms
			//蓝牙调试时用，不调试注释
			pcShow();                                                           //2.2ms单个float数据
			sendCount++;
		}
		else
		{
			sendCount++;
			if(sendCount==25)
				sendCount=0;
		}
		//获取角度		
		getAngle(&yaw,&yaw_acc_error);                                                      
	} 
}

//中断服务函数
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//首先清除中断标志位
		 usartReceiveOneData(USART1,&leftSpeedSet,&rightSpeedSet,&receCtrlFlag);
	 }
}

```

（2）外部中断服务函数

```c
#include "control.h"	
#include "pid.h"
#include "motor.h"
#include "adc.h"
#include "encoder.h"
#include "led.h"

int   Voltage          =0;           //电池电压采样相关的变量
float yaw              =0;           //转向陀螺仪
float yaw_acc_error    =0;           //yaw累积误差
#define FIVE_MS_ERROR   0.00002115   //yaw每5ms的向上漂移的度数，这里近似线性，可以做到半小时偏1度，每个人的这个值可能有所不同，可以自行计算


/**************************************************************************
函数功能：所有的控制代码都在这里面
          5ms定时中断由MPU6050的INT引脚触发		
**************************************************************************/
void EXTI15_10_IRQHandler(void) 
{                                                         
	EXTI_ClearITPendingBit(EXTI_Line12);                            //===清除LINE12线路挂起位		
	
	Led_Flash(200);                                                 //===LED闪烁，证明程序正常运行	
	
	yaw_acc_error += FIVE_MS_ERROR;								    //===yaw漂移误差累加
	
	Get_battery_volt_average(&Voltage,100);		                    //===电池电压测量，单位mv，100次取一次平均
	
	Get_Motor_Speed(&leftSpeedNow,&rightSpeedNow);                  //===获取左右轮子真实速度
	
	pid_Task_Letf.speedSet  = leftSpeedSet;	                        //===给速度设定值和实时值赋值
	pid_Task_Right.speedSet = rightSpeedSet;
	pid_Task_Letf.speedNow  = leftSpeedNow;
	pid_Task_Right.speedNow = rightSpeedNow;
	
	Pid_Ctrl(&motorLeft,&motorRight);                               //===执行PID控制函数

	if(Turn_Off(Voltage)==0)                                        //===如果电压异常
	{
		Set_Pwm(motorLeft,motorRight);                              //===赋值给PWM寄存器 		
	} 	
} 

```

#### 2.6.3 总结

ROS小车的底层控制代码就到这里了，之后还会做优化，可能会更新。如果有不懂的地方，都可以找小编解答。系列文章中，可能一些地方代码的使用并不是很好，也希望大家多多指正。

之后就是ROS层的文章了，希望大家提前完成linux系统的安装。网上的linux系统安装的教程十分多并且详细，这里就不在重复了。Linux系统镜像以及参考教程的。**在公众号：小白学移动机器人，发送：系统镜像**，即可获得下面镜像以及教程。
**树莓派（Ubuntu mate 16.04）、PC（Ubuntu 16.04）的镜像文件，以及TX2刷机的参考教程。**

下面陆续会有ROS安装、ROS小车模型文件编写、ROS小车启动文件编写、建图算法安装与测试、导航算法安装与测试。敬请期待。
### 系列文章
[搭建ROS小车真的难吗？](https://blog.csdn.net/zhao_ke_xue/article/details/107922037)
[ROS小车软件结构以及控制流程](https://blog.csdn.net/zhao_ke_xue/article/details/107963681)
[STM32电机PWM控制](https://blog.csdn.net/zhao_ke_xue/article/details/108111850)
[STM32电机测速（正交\霍尔编码器）](https://blog.csdn.net/zhao_ke_xue/article/details/108112420)
[STM32电机PID速度控制](https://blog.csdn.net/zhao_ke_xue/article/details/108112694)
[STM32 MPU6050 数据获取、数据处理](https://blog.csdn.net/zhao_ke_xue/article/details/108136979)
[STM32与ROS通信教程](https://blog.csdn.net/zhao_ke_xue/article/details/105493907)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200821004807250.gif#pic_center)




