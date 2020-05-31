#include "show.h"
#include "pid.h"

/**************************************************************************
函数功能：向PC发送调试数据，配合串口助手使用
入口参数：无
返回  值：无
**************************************************************************/
void pcShow(void)
{    
	//printf("V:%d.%dv:%lf:%lf:%lf:%lf:%lf\r\n",Voltage/100,Voltage%100,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Z);//打印到APP上面 显示波形
	//printf("Angle:%d V:%d.%d:%d:%d\r\n",(int)Gyro_Turn,Voltage/100,Voltage%100,leftWheelEncoder,rightWheelEncoder);//打印到APP上面 显示波形
	//printf("Angle:%d V:%d.%d:%d:%d | %d:%d\r\n",(int)Gyro_Turn,Voltage/100,Voltage%100,g_Pid_Left_Adjust,g_Pid_Right_Adjust,leftSpeedNow,rightSpeedNow);//打印到APP上面 显示波形
	printf("%d,%d,%d,%d,\r\n",leftSpeedNow,rightSpeedNow,g_Pid_Left_Adjust,g_Pid_Right_Adjust);
	//printf("%d,%d,\r\n",g_Pid_Left_Adjust,g_Pid_Right_Adjust);
	//printf("Angle:%d,%d,\r\n",(int)Gyro_Turn,g_Pid_Angle_Adjust);

}

