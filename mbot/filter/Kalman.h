#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "stm32f10x.h"

typedef struct KALMAN
{
    double Q;       //过程噪声的协方差
    double R;       //观测噪声的协方差
    
    double x_last;  //上次的最优值，本次测量值，本次最优值
    double p_last;  //上次协方差，本次预测协方差，本次协方差
    
    double kg;      //kalman增益
    
    double A;       //系统转移矩阵，
    double H;       //观测转移矩阵   
		
    double best;

}Kalman_filter;

extern Kalman_filter   kalman_V_Left;
extern Kalman_filter   kalman_V_Right;

void  kalman_init(Kalman_filter* kalman);
double kalman_filter(Kalman_filter *kalman,double input);

#endif
