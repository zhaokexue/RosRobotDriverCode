#include "show.h"
#include "pid.h"

/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	//printf("V:%d.%dv:%lf:%lf:%lf:%lf:%lf\r\n",Voltage/100,Voltage%100,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Z);//��ӡ��APP���� ��ʾ����
	//printf("Angle:%d V:%d.%d:%d:%d\r\n",(int)Gyro_Turn,Voltage/100,Voltage%100,leftWheelEncoder,rightWheelEncoder);//��ӡ��APP���� ��ʾ����
	//printf("Angle:%d V:%d.%d:%d:%d | %d:%d\r\n",(int)Gyro_Turn,Voltage/100,Voltage%100,g_Pid_Left_Adjust,g_Pid_Right_Adjust,leftSpeedNow,rightSpeedNow);//��ӡ��APP���� ��ʾ����
	printf("%d,%d,%d,%d,\r\n",leftSpeedNow,rightSpeedNow,g_Pid_Left_Adjust,g_Pid_Right_Adjust);
	//printf("%d,%d,\r\n",g_Pid_Left_Adjust,g_Pid_Right_Adjust);
	//printf("Angle:%d,%d,\r\n",(int)Gyro_Turn,g_Pid_Angle_Adjust);

}
