#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"	 

void Motor_Init(u16 arr,u16 psc);
void PWM_Init(u16 arr,u16 psc);

#endif
