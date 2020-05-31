#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。

extern int leftWheelEncoder;
extern int rightWheelEncoder;
       
extern int leftWheelEncoderNow;
extern int rightWheelEncoderNow;
extern int leftWheelEncoderLast;
extern int rightWheelEncoderLast;

s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx);
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);

#endif
