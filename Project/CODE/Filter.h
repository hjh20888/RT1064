#ifndef Filter_h
#define Filter_h

#include "headfile.h"

//����ƽ���˲����ĳ���
#define MVF_LENGTH 5

typedef struct
{
    float kalmanGain;//Kalamn����
    float A;//״̬����
    float H;//�۲����
    float Q;//״̬����ķ���								QС��ƽ��
    float R;//�۲����ķ���								R���˲�Ч����Ӱ��Ƚϴ�	R��Сʱ�ͺ��٣��������ϴ�
    float P;//Ԥ�����																							R�ϴ�ʱ�ͺ�࣬��������С
    float B;
    float u;
		float filterOut;//�˲����ֵ
}KalmanInfo;

//Kalman�ṹ���������
extern KalmanInfo Imu_KalmanInfo;

extern KalmanInfo speed_L1_KalmanInfo;
extern KalmanInfo speed_R1_KalmanInfo;
extern KalmanInfo speed_L2_KalmanInfo;
extern KalmanInfo speed_R2_KalmanInfo;


typedef struct
{
		unsigned char index;	//�����ݲ����λ��
		unsigned char first_group;	  //�жϵ�һ�������Ƿ������
		short int buffer[MVF_LENGTH];	//���ڴ����Ҫ���������
		int sum;	//��������������֮��
}Moving_average;

//Moving_average�ṹ���������
extern Moving_average speed_L1_Moving_average;
extern Moving_average speed_R1_Moving_average;
extern Moving_average speed_L2_Moving_average;
extern Moving_average speed_R2_Moving_average;

extern Moving_average gyro_z_Moving_average;

//�˲��㷨
float complementar_filter(float acc,float gyro);//�����˲�
float kalman_filter(KalmanInfo *info, float new_value); //Kalman�˲���
short int Moving_average_filter(Moving_average *info,short int new_value);	//����ƽ���˲���

#endif