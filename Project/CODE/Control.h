#ifndef Control_h
#define Control_h

#include "headfile.h"

//�궨�������������pwm����
#define DIR_1 D14
#define DIR_2 D12
#define DIR_3	D0
#define DIR_4	D2

#define PWM_1 PWM1_MODULE1_CHB_D15
#define PWM_2 PWM1_MODULE0_CHB_D13

#define PWM_3 PWM1_MODULE3_CHB_D1
#define PWM_4 PWM2_MODULE3_CHB_D3

//�궨���������ʱ����AB������
#define ENCODER1_QTIMER		QTIMER_3
#define ENCODER2_QTIMER		QTIMER_2
#define ENCODER3_QTIMER		QTIMER_1
#define ENCODER4_QTIMER		QTIMER_1

#define ENCODER1_A	QTIMER3_TIMER2_B18
#define ENCODER1_B	QTIMER3_TIMER3_B19

#define ENCODER2_A	QTIMER2_TIMER0_C3		
#define ENCODER2_B	QTIMER2_TIMER3_C25

#define ENCODER3_A	QTIMER1_TIMER0_C0
#define ENCODER3_B	QTIMER1_TIMER1_C1

#define ENCODER4_A	QTIMER1_TIMER2_C2
#define ENCODER4_B	QTIMER1_TIMER3_C24


//����������
#define S_MOTOR_PIN   PWM4_MODULE2_CHA_C30       

//�궨���������ز���
#define ENCORDER_PRCISION	512.f	//512�ߡ�1024�ߵ�
#define ENCORDER_D 		19.2f		  //����������ֱ��   ����30
#define WHEEL_D				63.0f			//����ֱ��
#define	WHEEL_GEAR_D 	43.2f			//����ֱ������ֱ�� ����70
#define GET_DISTANCE_MM(val) ((((val*1.0f/ENCORDER_PRCISION)*ENCORDER_D*PI )*WHEEL_D)/WHEEL_GEAR_D/1.0f )	//val��������ֵ
#define GET_DISTANCE_CM(val) ((((val*1.0f/ENCORDER_PRCISION)*ENCORDER_D*PI )*WHEEL_D)/WHEEL_GEAR_D/1000.0f) //X���� ((L1+R2)-(L2+R1))/4.0f
																																																  //Y���� (L1+R1+L2+R2)/4.0f
																																																			
//Control�ź���
extern rt_sem_t Control_sem;

//rt_sem_t�ź���
extern rt_sem_t Mode_select_sem;

//Parameter_calculation�ź���
extern rt_sem_t Parameter_calculation_sem;

//Control���������ƿ�ָ��
extern rt_mutex_t Control_mutex;

typedef struct 	//pid
{
		float P;
		float I;
		float D;
		
		short int error;				//�������
		short int error_last;			//�ϴ����
		short int error_last_last;		//���ϴ����
		short int error_add;			//������
	
		short int error_rate;			//������-�ϴ����
		short int error_rate_pro;		//������-2*�ϴ����+���ϴ����
	
		short int limit;					//�޷�
		short int out;						//���
	
}PID;	

//��������PID�ṹ�����
extern PID increase_L1_PID;		//����ʽpid	L1
extern PID increase_R1_PID;		//����ʽpid	R1
extern PID increase_L2_PID;		//����ʽpid	L2
extern PID increase_R2_PID;		//����ʽpid	R2	

extern PID angle_PID;					//�ǶȻ�pid
extern PID angle_speed_PID;		//���ٶȻ�pid

extern PID attitude_X_PID;			//��̬����PID X
extern PID attitude_Y_PID;			//��̬����PID Y

//�����ķ��ȫ���˶�����
extern short int Mecanum_X, Mecanum_Y, Mecanum_Z;	

//��������ٶ�
extern short int expect_speed_L1, expect_speed_R1, expect_speed_L2, expect_speed_R2;

//��������
extern float key_P,key_I,key_D;

//��������ȡ����ֵ
extern short int encoder_L1,encoder_R1;
extern short int encoder_L2,encoder_R2;
//�������˲����ֵ
extern short int now_speed_L1, now_speed_R1;	
extern short int now_speed_L2, now_speed_R2;

//X Y�����ƽ�ƾ���
extern float DISTANCE_X, DISTANCE_Y;

//�����Ƕ������
extern float expect_angle, Length;

//X Y���������ٶ�
extern short int Expect_X,Expect_Y;

//ת��������־λ
extern short int turn_flag;	//ֵΪ1ʱ��ʼת��0ʱ����ת��

//����ģʽѡ��
extern unsigned char control_mode;	 //0 ǰ�������		1 ������̬		2 ����ͼƬ

extern unsigned char num;//��¼��ǰ�����

//Control�������ͷź��־λ
extern unsigned char Control_mutex_release_flag;// 1��ʾ��Ҫ�ͷ�

extern unsigned char picture_data[2];

void encoder_deal(void);	//������ֵ�Ĵ���

short int limit_value(short int now , short int high , short int low);	//����޷�

short int speed_PID(PID *increase_PID , short int now_speed , short int expect_speed);	//�ٶȻ� ����ʽpid

short int location_PID(PID *location_PID , short int now_speed , short int expect_speed); //λ��ʽpid

short int Angle_PID(PID *angle_PID , float now_angle , float expect_angle); //�ǶȻ�pid
short int Angle_PID2(PID *angle_PID , float now_angle , float expect_angle);//�ǶȻ�pid2

short int Angle_speed_PID(PID *angle_speed_PID , short int now_angle_speed , short int expect_angle_speed);	//���ٶȻ�pid

short int Attitude_PID(PID *attitude_PID , short int now_attitude , short int expect_attitude);	//��̬����pid

void Mecanum_Solution(short int Mecanum_X, short int Mecanum_Y, short int Mecanum_Z); //�����ķȫ�����

void Parameter_calculation(void);	//��������	���������Ƕ������

void get_picture(void); //��ȡͼƬ����

void Picture_handling(void);	//����ͼƬ

void Direction_control(short int expect_X, short int expect_Y, float expect_angle); //�˶��������

void speed_PID_Transformation(void); //�ٶȻ������任

void speed_control(short int expect_speed_L1,short int expect_speed_R1,short int expect_speed_L2,short int expect_speed_R2);		//ת�ٿ���

void Total_control(void); //�ܿ���

void Mode_select_init(void); //Mode_select�߳�

void Control_init(void); //Control�߳�
	


#endif