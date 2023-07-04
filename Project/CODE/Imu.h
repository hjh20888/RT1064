#ifndef Imu_h
#define Imu_h

#include "headfile.h"

#define PI 3.141592653589793f//3.14159f

//Imu信号量
extern rt_sem_t Imu_sem;

//俯仰角/横滚角/航向角/转换后的航向角
extern float pitch,roll,yaw,real_yaw;

void get_icm20602_value(void);//姿态角与加速度计数据获取

float transform_yaw(unsigned char mode, float yaw, float expect_yaw); //yaw角度转换

void Imu_init(void); //Imu线程

#endif