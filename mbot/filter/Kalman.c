#include "kalman.h"
#include "stdlib.h"

Kalman_filter   kalman_V_Left;
Kalman_filter   kalman_V_Right;

void kalman_init(Kalman_filter* kalman)
{
    kalman->x_last=1;
    kalman->p_last=5;     //这两个数值初始化无所谓，但P不要为0
    
    kalman->Q=0.0001;     //过程噪声
    kalman->R=0.03; 	  //观测噪声,即原始速度数据的方差？？？？
    kalman->A=1;          //简化单模型
    kalman->H=1;
}

double kalman_filter(Kalman_filter *kalman,double input)
{
    kalman->x_last=kalman->A*kalman->x_last;                //根据上一次的值预测本次的值    
    kalman->p_last=kalman->p_last+kalman->Q;                //根据上一次的协方差计算本次的协方差    
    kalman->kg=kalman->p_last/(kalman->p_last+kalman->R);   //根据本次的协方差计算kalman增益    
    kalman->x_last=kalman->x_last+kalman->kg*(input-kalman->H*kalman->x_last);//求出本次的最有值    
    kalman->p_last=(1-kalman->kg)*kalman->p_last;           //求出本次的协方差	
	  kalman->best=kalman->x_last;    
    return kalman->best;
}

