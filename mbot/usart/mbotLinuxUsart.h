#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11
#define FILTER_N 10

//从linux接收并解析数据到参数地址中
extern int receiveTo103(int *p_leftSpeedSet,int *p_rightSpeedSet);   
//封装数据，调用USART1_Send_String将数据发送给linux
extern void usartSendSpeed(float Left_V, float Right_V,float Angle); 
//发送指定字符数组的函数
void USART_Send_String(u8 *p,u16 sendSize);     
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 

#endif
