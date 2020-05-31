#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED 端口定义
#define LED PAout(4)  
//IO初始化
void LED_Init(void);  
//LED闪烁函数
void Led_Flash(u16 time);

#endif
