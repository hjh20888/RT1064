#include "headfile.h"

//Kalman�ṹ���������
KalmanInfo Imu_KalmanInfo;

KalmanInfo speed_L1_KalmanInfo;
KalmanInfo speed_R1_KalmanInfo;
KalmanInfo speed_L2_KalmanInfo;
KalmanInfo speed_R2_KalmanInfo;

//Moving_average�ṹ���������
Moving_average speed_L1_Moving_average;
Moving_average speed_R1_Moving_average;
Moving_average speed_L2_Moving_average;
Moving_average speed_R2_Moving_average;

Moving_average gyro_z_Moving_average;


//-------------------------------------------------------------------------------------------------------------------
//  brief      �����˲�
//  return     float
//  since      v1.0
//  Sample usage:		��PIT�ж�  ����һ������Ϊ5ms
//									complementar_filter(acc_X,gyro_X);
//-------------------------------------------------------------------------------------------------------------------
float acc_ratio = 0.6;      //���ٶȼƱ���
float gyro_ratio = 2.0;     //�����Ǳ���
float dt = 0.005;           //��������

float complementar_filter(float acc,float gyro)//�����˲�
{
		float temp_angle;
		float gyro_now;
		float error_angle;

		static float last_angle;
		static unsigned char first_angle;

		if (!first_angle) 
		{ 
				//�ж��Ƿ�Ϊ��һ�����б�����
				//����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
				first_angle = 1;
				last_angle = acc;
		}

		gyro_now = gyro * gyro_ratio;
		//���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
		error_angle = (acc - last_angle) * acc_ratio;

		//����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
		temp_angle = last_angle + (error_angle + gyro_now) * dt;

		//���浱ǰ�Ƕ�ֵ
		last_angle = temp_angle;

		return temp_angle;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      Kalman�˲���
//  return     float
//  since      v1.0
//  Sample usage:		new_valueΪ�µĲ���ֵ�����������˲���Ĺ���ֵ
//									kalman_filter(&Imu_KalmanInfo, acc_X)
//-------------------------------------------------------------------------------------------------------------------

float kalman_filter(KalmanInfo *info, float new_value) //Kalman�˲���
{
		//����Ԥ��ֵ
    float predictValue = info->A*info->filterOut+info->B*info->u;
		//��Э����
    info->P = info->A*info->A*info->P + info->Q;
		//���㿨��������
    info->kalmanGain = info->P * info->H /(info->P * info->H * info->H + info->R);
		//���������ֵ
    info->filterOut = predictValue + (new_value - predictValue)*info->kalmanGain;
		//����Э����
    info->P = (1 - info->kalmanGain* info->H)*info->P;
	
    return info->filterOut;
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����ƽ���˲���
//  return     short int
//  since      v1.0
//  Sample usage:		���ô��˲����Ա������ɼ��������ݽ����˲�
//									Moving_average_filter(&speed_L1_Moving_average, encoder_L1);
//-------------------------------------------------------------------------------------------------------------------

short int Moving_average_filter(Moving_average *info,short int new_value)	//����ƽ���˲���
{
		if(!info->first_group) 
		{ 
				//�жϵ�ǰ�����Ƿ�Ϊ��һ������
				//����ǵ�һ�飬�򷵻�ֵΪ sum/index��
			
				info->buffer[info->index] = new_value;
				info->sum += new_value;
				info->index++;
				
				//��һ���Ѵ���
				if(info->index >= MVF_LENGTH)
					info->first_group = 1;
					
				return info->sum/info->index;
		}
		
		//��������Ѵ���
		if(info->index >= MVF_LENGTH)
			info->index=0;
		
		if(info->first_group)
		{
				info->sum -= info->buffer[info->index];	//ɾȥ������������
				info->buffer[info->index] = new_value;	//����������
				info->sum += new_value;
				info->index++;	
		}	
		
		return info->sum/MVF_LENGTH;
}	