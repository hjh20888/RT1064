#include "headfile.h"

//Kalman结构体变量定义
KalmanInfo Imu_KalmanInfo;

KalmanInfo speed_L1_KalmanInfo;
KalmanInfo speed_R1_KalmanInfo;
KalmanInfo speed_L2_KalmanInfo;
KalmanInfo speed_R2_KalmanInfo;

//Moving_average结构体变量声明
Moving_average speed_L1_Moving_average;
Moving_average speed_R1_Moving_average;
Moving_average speed_L2_Moving_average;
Moving_average speed_R2_Moving_average;

Moving_average gyro_z_Moving_average;


//-------------------------------------------------------------------------------------------------------------------
//  brief      互补滤波
//  return     float
//  since      v1.0
//  Sample usage:		放PIT中断  周期一般设置为5ms
//									complementar_filter(acc_X,gyro_X);
//-------------------------------------------------------------------------------------------------------------------
float acc_ratio = 0.6;      //加速度计比例
float gyro_ratio = 2.0;     //陀螺仪比例
float dt = 0.005;           //采样周期

float complementar_filter(float acc,float gyro)//互补滤波
{
		float temp_angle;
		float gyro_now;
		float error_angle;

		static float last_angle;
		static unsigned char first_angle;

		if (!first_angle) 
		{ 
				//判断是否为第一次运行本函数
				//如果是第一次运行，则将上次角度值设置为与加速度值一致
				first_angle = 1;
				last_angle = acc;
		}

		gyro_now = gyro * gyro_ratio;
		//根据测量到的加速度值转换为角度之后与上次的角度值求偏差
		error_angle = (acc - last_angle) * acc_ratio;

		//根据偏差与陀螺仪测量得到的角度值计算当前角度值
		temp_angle = last_angle + (error_angle + gyro_now) * dt;

		//保存当前角度值
		last_angle = temp_angle;

		return temp_angle;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      Kalman滤波器
//  return     float
//  since      v1.0
//  Sample usage:		new_value为新的测量值，函数返回滤波后的估计值
//									kalman_filter(&Imu_KalmanInfo, acc_X)
//-------------------------------------------------------------------------------------------------------------------

float kalman_filter(KalmanInfo *info, float new_value) //Kalman滤波器
{
		//计算预测值
    float predictValue = info->A*info->filterOut+info->B*info->u;
		//求协方差
    info->P = info->A*info->A*info->P + info->Q;
		//计算卡尔曼增益
    info->kalmanGain = info->P * info->H /(info->P * info->H * info->H + info->R);
		//计算输出的值
    info->filterOut = predictValue + (new_value - predictValue)*info->kalmanGain;
		//更新协方差
    info->P = (1 - info->kalmanGain* info->H)*info->P;
	
    return info->filterOut;
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      滑动平均滤波器
//  return     short int
//  since      v1.0
//  Sample usage:		可用此滤波器对编码器采集到的数据进行滤波
//									Moving_average_filter(&speed_L1_Moving_average, encoder_L1);
//-------------------------------------------------------------------------------------------------------------------

short int Moving_average_filter(Moving_average *info,short int new_value)	//滑动平均滤波器
{
		if(!info->first_group) 
		{ 
				//判断当前数据是否为第一组数据
				//如果是第一组，则返回值为 sum/index；
			
				info->buffer[info->index] = new_value;
				info->sum += new_value;
				info->index++;
				
				//第一组已存满
				if(info->index >= MVF_LENGTH)
					info->first_group = 1;
					
				return info->sum/info->index;
		}
		
		//如果数据已存满
		if(info->index >= MVF_LENGTH)
			info->index=0;
		
		if(info->first_group)
		{
				info->sum -= info->buffer[info->index];	//删去最早存进的数据
				info->buffer[info->index] = new_value;	//存入新数据
				info->sum += new_value;
				info->index++;	
		}	
		
		return info->sum/MVF_LENGTH;
}	