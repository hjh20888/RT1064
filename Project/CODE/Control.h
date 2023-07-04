#ifndef Control_h
#define Control_h

#include "headfile.h"

//宏定义电机方向输出与pwm引脚
#define DIR_1 D14
#define DIR_2 D12
#define DIR_3	D0
#define DIR_4	D2

#define PWM_1 PWM1_MODULE1_CHB_D15
#define PWM_2 PWM1_MODULE0_CHB_D13

#define PWM_3 PWM1_MODULE3_CHB_D1
#define PWM_4 PWM2_MODULE3_CHB_D3

//宏定义编码器定时器与AB相引脚
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


//定义舵机引脚
#define S_MOTOR_PIN   PWM4_MODULE2_CHA_C30       

//宏定义编码器相关参数
#define ENCORDER_PRCISION	512.f	//512线、1024线等
#define ENCORDER_D 		19.2f		  //编码器齿轮直径   齿数30
#define WHEEL_D				63.0f			//车轮直径
#define	WHEEL_GEAR_D 	43.2f			//车轮直连齿轮直径 齿数70
#define GET_DISTANCE_MM(val) ((((val*1.0f/ENCORDER_PRCISION)*ENCORDER_D*PI )*WHEEL_D)/WHEEL_GEAR_D/1.0f )	//val编码器的值
#define GET_DISTANCE_CM(val) ((((val*1.0f/ENCORDER_PRCISION)*ENCORDER_D*PI )*WHEEL_D)/WHEEL_GEAR_D/1000.0f) //X方向 ((L1+R2)-(L2+R1))/4.0f
																																																  //Y方向 (L1+R1+L2+R2)/4.0f
																																																			
//Control信号量
extern rt_sem_t Control_sem;

//rt_sem_t信号量
extern rt_sem_t Mode_select_sem;

//Parameter_calculation信号量
extern rt_sem_t Parameter_calculation_sem;

//Control互斥量控制块指针
extern rt_mutex_t Control_mutex;

typedef struct 	//pid
{
		float P;
		float I;
		float D;
		
		short int error;				//本次误差
		short int error_last;			//上次误差
		short int error_last_last;		//上上次误差
		short int error_add;			//误差积累
	
		short int error_rate;			//这次误差-上次误差
		short int error_rate_pro;		//这次误差-2*上次误差+上上次误差
	
		short int limit;					//限幅
		short int out;						//输出
	
}PID;	

//声明各种PID结构体变量
extern PID increase_L1_PID;		//增量式pid	L1
extern PID increase_R1_PID;		//增量式pid	R1
extern PID increase_L2_PID;		//增量式pid	L2
extern PID increase_R2_PID;		//增量式pid	R2	

extern PID angle_PID;					//角度环pid
extern PID angle_speed_PID;		//角速度环pid

extern PID attitude_X_PID;			//姿态调整PID X
extern PID attitude_Y_PID;			//姿态调整PID Y

//麦克纳姆轮全向运动参数
extern short int Mecanum_X, Mecanum_Y, Mecanum_Z;	

//最终输出速度
extern short int expect_speed_L1, expect_speed_R1, expect_speed_L2, expect_speed_R2;

//按键调参
extern float key_P,key_I,key_D;

//编码器读取到的值
extern short int encoder_L1,encoder_R1;
extern short int encoder_L2,encoder_R2;
//编码器滤波后的值
extern short int now_speed_L1, now_speed_R1;	
extern short int now_speed_L2, now_speed_R2;

//X Y方向的平移距离
extern float DISTANCE_X, DISTANCE_Y;

//期望角度与距离
extern float expect_angle, Length;

//X Y方向期望速度
extern short int Expect_X,Expect_Y;

//转向环启动标志位
extern short int turn_flag;	//值为1时开始转向，0时结束转向

//控制模式选择
extern unsigned char control_mode;	 //0 前往坐标点		1 调整姿态		2 搬运图片

extern unsigned char num;//记录当前坐标号

//Control互斥量释放后标志位
extern unsigned char Control_mutex_release_flag;// 1表示需要释放

extern unsigned char picture_data[2];

void encoder_deal(void);	//编码器值的处理

short int limit_value(short int now , short int high , short int low);	//输出限幅

short int speed_PID(PID *increase_PID , short int now_speed , short int expect_speed);	//速度环 增量式pid

short int location_PID(PID *location_PID , short int now_speed , short int expect_speed); //位置式pid

short int Angle_PID(PID *angle_PID , float now_angle , float expect_angle); //角度环pid
short int Angle_PID2(PID *angle_PID , float now_angle , float expect_angle);//角度环pid2

short int Angle_speed_PID(PID *angle_speed_PID , short int now_angle_speed , short int expect_angle_speed);	//角速度环pid

short int Attitude_PID(PID *attitude_PID , short int now_attitude , short int expect_attitude);	//姿态调整pid

void Mecanum_Solution(short int Mecanum_X, short int Mecanum_Y, short int Mecanum_Z); //麦克纳姆全向控制

void Parameter_calculation(void);	//参数计算	计算期望角度与距离

void get_picture(void); //获取图片数据

void Picture_handling(void);	//搬运图片

void Direction_control(short int expect_X, short int expect_Y, float expect_angle); //运动方向控制

void speed_PID_Transformation(void); //速度环参数变换

void speed_control(short int expect_speed_L1,short int expect_speed_R1,short int expect_speed_L2,short int expect_speed_R2);		//转速控制

void Total_control(void); //总控制

void Mode_select_init(void); //Mode_select线程

void Control_init(void); //Control线程
	


#endif