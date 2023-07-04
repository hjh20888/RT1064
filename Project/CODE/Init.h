#ifndef Init_h
#define Init_h

#include "headfile.h"



void PID_Init(void);		//各种PID结构体成员的初始化

void KalmanFilter_Init(void);	//Kalman滤波器初始化

void Moving_average_filter_Init(void); //滑动平均滤波器结构体成员的初始化

void All_parameter_Init(void);   //各种参数的初始化

#endif