#ifndef Imu_h
#define Imu_h

#include "headfile.h"

#define PI 3.141592653589793f//3.14159f

//Imu�ź���
extern rt_sem_t Imu_sem;

//������/�����/�����/ת����ĺ����
extern float pitch,roll,yaw,real_yaw;

void get_icm20602_value(void);//��̬������ٶȼ����ݻ�ȡ

float transform_yaw(unsigned char mode, float yaw, float expect_yaw); //yaw�Ƕ�ת��

void Imu_init(void); //Imu�߳�

#endif