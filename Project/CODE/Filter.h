#ifndef Filter_h
#define Filter_h

#include "headfile.h"

//滑动平均滤波器的长度
#define MVF_LENGTH 5

typedef struct
{
    float kalmanGain;//Kalamn增益
    float A;//状态矩阵
    float H;//观测矩阵
    float Q;//状态矩阵的方差								Q小更平滑
    float R;//观测矩阵的方差								R对滤波效果的影响比较大	R较小时滞后少，但波动较大
    float P;//预测误差																							R较大时滞后多，但波动较小
    float B;
    float u;
		float filterOut;//滤波后的值
}KalmanInfo;

//Kalman结构体变量声明
extern KalmanInfo Imu_KalmanInfo;

extern KalmanInfo speed_L1_KalmanInfo;
extern KalmanInfo speed_R1_KalmanInfo;
extern KalmanInfo speed_L2_KalmanInfo;
extern KalmanInfo speed_R2_KalmanInfo;


typedef struct
{
		unsigned char index;	//新数据插入的位置
		unsigned char first_group;	  //判断第一组数据是否处理完毕
		short int buffer[MVF_LENGTH];	//用于存放需要处理的数据
		int sum;	//数组中所有数据之和
}Moving_average;

//Moving_average结构体变量声明
extern Moving_average speed_L1_Moving_average;
extern Moving_average speed_R1_Moving_average;
extern Moving_average speed_L2_Moving_average;
extern Moving_average speed_R2_Moving_average;

extern Moving_average gyro_z_Moving_average;

//滤波算法
float complementar_filter(float acc,float gyro);//互补滤波
float kalman_filter(KalmanInfo *info, float new_value); //Kalman滤波器
short int Moving_average_filter(Moving_average *info,short int new_value);	//滑动平均滤波器

#endif