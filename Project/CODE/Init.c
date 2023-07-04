#include "headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  brief      各种PID结构体成员的初始化
//  return     void
//  since      v1.0
//  Sample usage:		所有参数默认为零，根据实际调试进行修改
//-------------------------------------------------------------------------------------------------------------------

void PID_Init(void)		//各种PID结构体成员的初始化
{
		//增量式pid	L1
		increase_L1_PID.P=500;//1050;
		increase_L1_PID.I=26;//40;
		increase_L1_PID.D=0;
		
		increase_L1_PID.error_last = 0;				//上次误差
		increase_L1_PID.error_last_last = 0;	//上上次误差
	
		increase_L1_PID.limit=30000;					//限幅
	
		//增量式pid	R1
		increase_R1_PID.P=500;//1050;
		increase_R1_PID.I=26;//40;
		increase_R1_PID.D=0;
	
		increase_R1_PID.error_last = 0;				//上次误差
		increase_R1_PID.error_last_last = 0;	//上上次误差
	
		increase_R1_PID.limit=30000;					//限幅
		
		//增量式pid	L2
		increase_L2_PID.P=500;//1060;
		increase_L2_PID.I=26;//40;
		increase_L2_PID.D=0;
		
		increase_L2_PID.error_last = 0;				//上次误差
		increase_L2_PID.error_last_last = 0;	//上上次误差
		
		increase_L2_PID.limit=30000;					//限幅
		
		//增量式pid	R2
		increase_R2_PID.P=500;//1050;
		increase_R2_PID.I=26;//40;
		increase_R2_PID.D=0;
		
		increase_R2_PID.error_last = 0;				//上次误差
		increase_R2_PID.error_last_last = 0;	//上上次误差
		
		increase_R2_PID.limit=30000;					//限幅
		
		//角速度环
		angle_speed_PID.P=0.013;//0.02;//
		angle_speed_PID.I=0;		
		angle_speed_PID.D=0.036;//0.052;//
		angle_speed_PID.limit=0;							//限幅
		
		//角度环pid
		angle_PID.P=85;//70;
		angle_PID.I=0.1;;
		angle_PID.D=0;
		angle_PID.limit=5100;//5000;										//限幅
		
		//姿态环
		attitude_X_PID.P=0.12;//0.15
		attitude_X_PID.I=0;
		attitude_X_PID.D=0.2;//0.45
		attitude_X_PID.limit=0;
		
		attitude_Y_PID.P=0.12;
		attitude_Y_PID.I=0;
		attitude_Y_PID.D=0.2;
		attitude_Y_PID.limit=0;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      Kalman结构体成员的初始化
//  return     void
//  since      v1.0
//  Sample usage:		所有参数默认为零，根据实际调试进行修改	一般只需要对R进行修改
//-------------------------------------------------------------------------------------------------------------------

void KalmanFilter_Init(void)	//Kalman滤波器初始化
{
		//Imu
    Imu_KalmanInfo.A = 1;
    Imu_KalmanInfo.H = 1;
    Imu_KalmanInfo.P = 0.1;
    Imu_KalmanInfo.Q = 0.03;//0.05
    Imu_KalmanInfo.R = 0.6;	//R值较小时滞后很少，但有波动; R值较大时滞后多，比较平滑  //0.2	0.4
    Imu_KalmanInfo.B = 0.1;
    Imu_KalmanInfo.u = 0;
    Imu_KalmanInfo.filterOut = 0;
	
		//speed_L1
		speed_L1_KalmanInfo.A = 1;
    speed_L1_KalmanInfo.H = 1;
    speed_L1_KalmanInfo.P = 0.1;
    speed_L1_KalmanInfo.Q = 0.03;//0.05
    speed_L1_KalmanInfo.R = 0.6;	//R值较小时滞后很少，但有波动; R值较大时滞后多，比较平滑  //0.2
    speed_L1_KalmanInfo.B = 0.1;
    speed_L1_KalmanInfo.u = 0;
    speed_L1_KalmanInfo.filterOut = 0;
	
		//speed_R1
		speed_R1_KalmanInfo.A = 1;
    speed_R1_KalmanInfo.H = 1;
    speed_R1_KalmanInfo.P = 0.1;
    speed_R1_KalmanInfo.Q = 0.03;//0.05
    speed_R1_KalmanInfo.R = 0.6;	//R值较小时滞后很少，但有波动; R值较大时滞后多，比较平滑  //0.2
    speed_R1_KalmanInfo.B = 0.1;
    speed_R1_KalmanInfo.u = 0;
    speed_R1_KalmanInfo.filterOut = 0;
		
		//speed_L2
		speed_L2_KalmanInfo.A = 1;
    speed_L2_KalmanInfo.H = 1;
    speed_L2_KalmanInfo.P = 0.1;
    speed_L2_KalmanInfo.Q = 0.03;//0.05
    speed_L2_KalmanInfo.R = 0.6;	//R值较小时滞后很少，但有波动; R值较大时滞后多，比较平滑  //0.2
    speed_L2_KalmanInfo.B = 0.1;
    speed_L2_KalmanInfo.u = 0;
    speed_L2_KalmanInfo.filterOut = 0;
		
		//speed_R2
		speed_R2_KalmanInfo.A = 1;
    speed_R2_KalmanInfo.H = 1;
    speed_R2_KalmanInfo.P = 0.1;
    speed_R2_KalmanInfo.Q = 0.03;//0.05
    speed_R2_KalmanInfo.R = 0.6;	//R值较小时滞后很少，但有波动; R值较大时滞后多，比较平滑  //0.2
    speed_R2_KalmanInfo.B = 0.1;
    speed_R2_KalmanInfo.u = 0;
    speed_R2_KalmanInfo.filterOut = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      滑动平均滤波器结构体成员的初始化
//  return     short int
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void Moving_average_filter_Init(void)//滑动平均滤波器结构体成员的初始化
{
		//speed_L1
		speed_L1_Moving_average.index=0;
		speed_L1_Moving_average.sum=0;
		speed_L1_Moving_average.first_group=0;
	
		//speed_R1
		speed_R1_Moving_average.index=0;
		speed_R1_Moving_average.sum=0;
		speed_R1_Moving_average.first_group=0;
	
		//speed_L2
		speed_L2_Moving_average.index=0;
		speed_L2_Moving_average.sum=0;
		speed_L2_Moving_average.first_group=0;
	
		//speed_R2
		speed_R2_Moving_average.index=0;
		speed_R2_Moving_average.sum=0;
		speed_R2_Moving_average.first_group=0;
	
		//gyro_Z
		gyro_z_Moving_average.index=0;
		gyro_z_Moving_average.sum=0;
		gyro_z_Moving_average.first_group=0;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      各种参数的初始化
//  return     void
//  since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

void All_parameter_Init(void)   //各种参数的初始化
{
		//各种PID结构体成员变量的初始化
		PID_Init();	
		
		//Kalman滤波器初始化
		KalmanFilter_Init(); 	
		
		//滑动平均滤波器结构体成员的初始化
		Moving_average_filter_Init();
}	