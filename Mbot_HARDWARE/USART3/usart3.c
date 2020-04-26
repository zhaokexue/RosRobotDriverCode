#include "usart3.h"

u8 Usart3_Receive;

/**************************************************************************
函数功能：串口3初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	   //使能UGPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	   //使能USART3时钟
	//USART3_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	           //复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART3_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;      //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;   //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		   //子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                           //根据指定的参数初始化VIC寄存器
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;                //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;     //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;        //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure);                  //初始化串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);             //开启串口接受中断
	USART_Cmd(USART3, ENABLE);                                 //使能串口3 
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	
		//USART_ClearITPendingBit(USART3,USART_IT_RXNE);  //清除中断标志位	串口读取DR寄存器以后会硬件清除中断标志位	
		static	int uart_receive=0;                       //蓝牙接收相关变量
		uart_receive=USART_ReceiveData(USART3); 

		if(uart_receive==0x4A)//速度切换
		{
			if(Flag_sudu==2) //低速挡（默认值）
				Flag_sudu=1; //高速档
			else
				Flag_sudu=2;  
		}	
		
		if(uart_receive==0x4B)
		{
			if(Flag_useApp==0)//默认不使用APP
				Flag_useApp=1;
			else
				Flag_useApp=0;
		}
		
		if(uart_receive>10)  //默认使用
		{			
			if(uart_receive==0x5A)	
				Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
			else if(uart_receive==0x41||uart_receive==0x42||uart_receive==0x48)	
				Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
			else if(uart_receive==0x44||uart_receive==0x45||uart_receive==0x46)	
				Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
			else if(uart_receive==0x43)	
				Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;//////////////右
			else if(uart_receive==0x47)
				Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;//////////////左
			else 
				Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
		}
	}  											 
} 



