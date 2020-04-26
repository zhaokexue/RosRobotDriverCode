#include "mbotLinuxUsart.h"
#include <math.h>

/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/
//数据接收暂存区
unsigned char  receiveBuff[16] = {0};         
//通信协议常量
const unsigned char ender[2]   = {0x0d, 0x0a};
const unsigned char header[2]  = {0x55, 0xaa};
const int SPEED_INFO           = 0xa55a; 
const int GET_SPEED            = 0xaaaa;
//发送数据（左轮速、右轮速、角度）共用体
union sendData
{
	float d;
	unsigned char data[4];
}leftdata,rightdata,angle;
//发送接收校验值共用体
union checkSum
{
	short d;
	unsigned char data[1];
}SendCheckSum, ReceiveCheckSum;
//接收信息命名、接收头共用体
union receiveHeader
{
	int d;
	unsigned char data[2];
}receive_command;
//发送命令的共用体
union sendCommand
{
	int d;
	unsigned char data[2];
}send_command;
//接收里程计信息左右轮速共用体
union odometry
{
	int odoemtry_int;
	unsigned char odometry_char[4];
}vel_left, vel_right;

/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的左右轮控制速度，分别存入参数中
入口参数：左轮轮速控制地址、右轮轮速控制地址
返回  值：无特殊意义
**************************************************************************/
int receiveTo103(int *p_leftSpeedSet,int *p_rightSpeedSet)
{
	u8 USART_Receiver               = 0;          //接收数据
	static u8 checkSum              = 0;
	static u8  USARTBufferIndex     = 0;
	static s16 i=0,j=0,k=0;
	static u8  USARTReceiverFront   = 0;
	static u8  Start_Flag           = START;      //一帧数据传送开始标志位
	static s16 dataLength           = 0;
	const int V_XIN_FU              =400;         //速度限幅，可以根据自己情况设置
	USART_Receiver = USART_ReceiveData(USART1);   //@@@@@#####如果你使用不是USART1更改成相应的，比如USART3
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)       
		{  
			if(USARTReceiverFront == 0x55)        //数据头两位
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				receiveBuff[0]= 0x55;             //存储数据头信息
				receiveBuff[1]= 0xaa;
				//printf("ok\n");
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  //数据头1 0x55
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收消息类型
				if(i==0)
				{
					receiveBuff[2] = USART_Receiver;         //buf[2]
					receive_command.data[0] = receiveBuff[2];
					i++;
				}
				else if(i==1)
				{
					receiveBuff[3] = USART_Receiver;         //buf[3]
					receive_command.data[1] = receiveBuff[3];
					//检查消息类型，失败的话直接跳出
					if(receive_command.d != GET_SPEED)
					{
						printf("Received command error!");
						return 0;
					}
					USARTBufferIndex++;
				}
				break;
			case 1://接收左右轮速度数据的长度
				receiveBuff[4] = USART_Receiver;            //buf[4]	
				dataLength     = receiveBuff[4];
				USARTBufferIndex++;
				break;
			case 2://接收速度数据，并赋值处理 
				receiveBuff[j + 5] = USART_Receiver;        //buf[5] - buf[12]					
				j++;
				if(j >= dataLength)                         //修改此处将接收发送一致
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 3://接收校验值信息
				receiveBuff[5 + dataLength] = USART_Receiver;   //buf[13]
				ReceiveCheckSum.data[0] = receiveBuff[5 + dataLength];
				checkSum = getCrc8(receiveBuff, 5 + dataLength);
				  // 检查信息校验值
				if (checkSum != ReceiveCheckSum.d)
				{
					printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
			case 4://接收信息尾
				if(k==0)
				{
					receiveBuff[6 + dataLength] = USART_Receiver;
					k++;
				}
				else if (k==1)
				{
					receiveBuff[6 + dataLength + 1] = USART_Receiver;
					//检查信息尾
					if (receiveBuff[6 + dataLength] != ender[0] || receiveBuff[6 + dataLength + 1] != ender[1])
					{
						printf("Received message header error!");
						return 0;
					}
					//通过测试进行速度赋值操作					
					 for(k = 0; k < 4; k++)
					{
						vel_left.odometry_char[k]  = receiveBuff[k + 5];
						vel_right.odometry_char[k] = receiveBuff[k + 9];
					}				
					//速度赋值操作
					*p_leftSpeedSet  = vel_left.odoemtry_int;
					*p_rightSpeedSet = vel_right.odoemtry_int;
					/*=========================速度限制在V_XIN_FU====================*/
					if(*p_leftSpeedSet>V_XIN_FU)
						*p_leftSpeedSet=V_XIN_FU;
					if(*p_leftSpeedSet<-V_XIN_FU)
						*p_leftSpeedSet=-V_XIN_FU;
					if(*p_rightSpeedSet>V_XIN_FU)
						*p_rightSpeedSet=V_XIN_FU;
					if(*p_rightSpeedSet<-V_XIN_FU)
						*p_rightSpeedSet=-V_XIN_FU;
					/*==============================================================*/
					USARTBufferIndex++;
				}
				break;
			case 5:
				//printf("ok\n");
				USARTBufferIndex   = 0;
				USARTReceiverFront = 0;
				Start_Flag         = START;
				checkSum           = 0;
				dataLength         =0;
				i = 0;
				j = 0;
				k = 0;
				break;
			 default:break;
		}		
	}
	return 0;
}
/**************************************************************************
函数功能：将左右轮速和角度数据进行打包，通过串口发送给Linux
入口参数：实时左轮轮速、实时右轮轮速、实时角度（如果没有角度也可以不发）
返回  值：无
**************************************************************************/
void usartSendSpeed(float Left_V, float Right_V,float Angle)
{
	//协议数据缓存数组
	unsigned char buf[20] = {0};
	int i, length = 0;

	// 计算左右轮期望速度

	leftdata.d  = Left_V;
	rightdata.d = Right_V;
	angle.d     = Angle;
	
	// 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];
	
	// 设置消息类型
	send_command.d = SPEED_INFO;
	for(i = 0; i < 2; i++)
		buf[i + 2] = send_command.data[i];
	
	// 设置机器人左右轮速度、角度
	length = 12;
	buf[4] = length;
	for(i = 0; i < 4; i++)
	{
		buf[i + 5] = leftdata.data[i];
		buf[i + 9] = rightdata.data[i];
		buf[i + 13]= angle.data[i];
	}
	
	// 设置校验值、消息尾
	buf[5 + length] = getCrc8(buf, 5 + length);
	buf[6 + length] = ender[0];
	buf[6 + length + 1] = ender[1];
	
	//发送字符串数据
	USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
函数功能：发送指定大小的字符数组，被usartSendSpeed函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		//@@@@@#####如果你使用不是USART1更改成相应的，比如USART3，这里有两处修改
		while( !(USART1->SR&(0x01<<7)) );//发送缓冲区为空
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendSpeed和receiveTo103函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/

////中断服务函数
//void USART1_IRQHandler()
//{
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
// 	 {
//		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//首先清除中断标志位
//		 receiveTo103(&leftSpeedSet,&rightSpeedSet);
//	 }
//}



