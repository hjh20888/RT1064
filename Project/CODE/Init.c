#include "headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����PID�ṹ���Ա�ĳ�ʼ��
//  return     void
//  since      v1.0
//  Sample usage:		���в���Ĭ��Ϊ�㣬����ʵ�ʵ��Խ����޸�
//-------------------------------------------------------------------------------------------------------------------

void PID_Init(void)		//����PID�ṹ���Ա�ĳ�ʼ��
{
		//����ʽpid	L1
		increase_L1_PID.P=500;//1050;
		increase_L1_PID.I=26;//40;
		increase_L1_PID.D=0;
		
		increase_L1_PID.error_last = 0;				//�ϴ����
		increase_L1_PID.error_last_last = 0;	//���ϴ����
	
		increase_L1_PID.limit=30000;					//�޷�
	
		//����ʽpid	R1
		increase_R1_PID.P=500;//1050;
		increase_R1_PID.I=26;//40;
		increase_R1_PID.D=0;
	
		increase_R1_PID.error_last = 0;				//�ϴ����
		increase_R1_PID.error_last_last = 0;	//���ϴ����
	
		increase_R1_PID.limit=30000;					//�޷�
		
		//����ʽpid	L2
		increase_L2_PID.P=500;//1060;
		increase_L2_PID.I=26;//40;
		increase_L2_PID.D=0;
		
		increase_L2_PID.error_last = 0;				//�ϴ����
		increase_L2_PID.error_last_last = 0;	//���ϴ����
		
		increase_L2_PID.limit=30000;					//�޷�
		
		//����ʽpid	R2
		increase_R2_PID.P=500;//1050;
		increase_R2_PID.I=26;//40;
		increase_R2_PID.D=0;
		
		increase_R2_PID.error_last = 0;				//�ϴ����
		increase_R2_PID.error_last_last = 0;	//���ϴ����
		
		increase_R2_PID.limit=30000;					//�޷�
		
		//���ٶȻ�
		angle_speed_PID.P=0.013;//0.02;//
		angle_speed_PID.I=0;		
		angle_speed_PID.D=0.036;//0.052;//
		angle_speed_PID.limit=0;							//�޷�
		
		//�ǶȻ�pid
		angle_PID.P=85;//70;
		angle_PID.I=0.1;;
		angle_PID.D=0;
		angle_PID.limit=5100;//5000;										//�޷�
		
		//��̬��
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
//  brief      Kalman�ṹ���Ա�ĳ�ʼ��
//  return     void
//  since      v1.0
//  Sample usage:		���в���Ĭ��Ϊ�㣬����ʵ�ʵ��Խ����޸�	һ��ֻ��Ҫ��R�����޸�
//-------------------------------------------------------------------------------------------------------------------

void KalmanFilter_Init(void)	//Kalman�˲�����ʼ��
{
		//Imu
    Imu_KalmanInfo.A = 1;
    Imu_KalmanInfo.H = 1;
    Imu_KalmanInfo.P = 0.1;
    Imu_KalmanInfo.Q = 0.03;//0.05
    Imu_KalmanInfo.R = 0.6;	//Rֵ��Сʱ�ͺ���٣����в���; Rֵ�ϴ�ʱ�ͺ�࣬�Ƚ�ƽ��  //0.2	0.4
    Imu_KalmanInfo.B = 0.1;
    Imu_KalmanInfo.u = 0;
    Imu_KalmanInfo.filterOut = 0;
	
		//speed_L1
		speed_L1_KalmanInfo.A = 1;
    speed_L1_KalmanInfo.H = 1;
    speed_L1_KalmanInfo.P = 0.1;
    speed_L1_KalmanInfo.Q = 0.03;//0.05
    speed_L1_KalmanInfo.R = 0.6;	//Rֵ��Сʱ�ͺ���٣����в���; Rֵ�ϴ�ʱ�ͺ�࣬�Ƚ�ƽ��  //0.2
    speed_L1_KalmanInfo.B = 0.1;
    speed_L1_KalmanInfo.u = 0;
    speed_L1_KalmanInfo.filterOut = 0;
	
		//speed_R1
		speed_R1_KalmanInfo.A = 1;
    speed_R1_KalmanInfo.H = 1;
    speed_R1_KalmanInfo.P = 0.1;
    speed_R1_KalmanInfo.Q = 0.03;//0.05
    speed_R1_KalmanInfo.R = 0.6;	//Rֵ��Сʱ�ͺ���٣����в���; Rֵ�ϴ�ʱ�ͺ�࣬�Ƚ�ƽ��  //0.2
    speed_R1_KalmanInfo.B = 0.1;
    speed_R1_KalmanInfo.u = 0;
    speed_R1_KalmanInfo.filterOut = 0;
		
		//speed_L2
		speed_L2_KalmanInfo.A = 1;
    speed_L2_KalmanInfo.H = 1;
    speed_L2_KalmanInfo.P = 0.1;
    speed_L2_KalmanInfo.Q = 0.03;//0.05
    speed_L2_KalmanInfo.R = 0.6;	//Rֵ��Сʱ�ͺ���٣����в���; Rֵ�ϴ�ʱ�ͺ�࣬�Ƚ�ƽ��  //0.2
    speed_L2_KalmanInfo.B = 0.1;
    speed_L2_KalmanInfo.u = 0;
    speed_L2_KalmanInfo.filterOut = 0;
		
		//speed_R2
		speed_R2_KalmanInfo.A = 1;
    speed_R2_KalmanInfo.H = 1;
    speed_R2_KalmanInfo.P = 0.1;
    speed_R2_KalmanInfo.Q = 0.03;//0.05
    speed_R2_KalmanInfo.R = 0.6;	//Rֵ��Сʱ�ͺ���٣����в���; Rֵ�ϴ�ʱ�ͺ�࣬�Ƚ�ƽ��  //0.2
    speed_R2_KalmanInfo.B = 0.1;
    speed_R2_KalmanInfo.u = 0;
    speed_R2_KalmanInfo.filterOut = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����ƽ���˲����ṹ���Ա�ĳ�ʼ��
//  return     short int
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void Moving_average_filter_Init(void)//����ƽ���˲����ṹ���Ա�ĳ�ʼ��
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
//  brief      ���ֲ����ĳ�ʼ��
//  return     void
//  since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

void All_parameter_Init(void)   //���ֲ����ĳ�ʼ��
{
		//����PID�ṹ���Ա�����ĳ�ʼ��
		PID_Init();	
		
		//Kalman�˲�����ʼ��
		KalmanFilter_Init(); 	
		
		//����ƽ���˲����ṹ���Ա�ĳ�ʼ��
		Moving_average_filter_Init();
}	