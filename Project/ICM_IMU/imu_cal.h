#ifndef _imu_cal_H_
#define _imu_cal_H_
#include "headfile.h"
#include "Butterwarth_filter.h"

#define Imu_Sampling_Dt  0.003
#define Imu_Sampling_Freq  (1/Imu_Sampling_Dt)	//Ö´ÐÐÖÜÆÚ

#define M_PI_F 3.141592653589793f
#define RtA         57.324841
#define AtR    	    0.0174533
#define Acc_G 	    0.0000610351
#define Gyro_G 	    0.0610351
#define Gyro_Gr	    0.0010653
#define DEG2RAD (M_PI_F / 180.0f)
#define RAD2DEG (180.0f / M_PI_F)
//#define GYRO_CALIBRATION_COFF 0.060976f  //2000
//#define GYRO_CALIBRATION_COFF 0.030488f;  //1000
#define GYRO_CALIBRATION_COFF 0.0152672f    //500
//#define GYRO_CALIBRATION_COFF 0.0076336f    //250
#define AcceMax_1G      4096
#define GRAVITY_MSS     9.80665f
#define ACCEL_TO_1G     GRAVITY_MSS/AcceMax_1G
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS
#define sq2(sq) (((float)sq) * ((float)sq))
	
void imu_filter_init(void);
void AHRS_Update_IMU(float gx, float gy, float gz,float ax, float ay, float az,float mx, float my, float mz,float gyro_mold);
typedef struct
{
	float x;
	float y;
	float z;
}Vector3f;
typedef struct
{
	float x;
	float y;
}Vector2f;

typedef struct 
{
	float L1,R1,L2,R2;
	Vector2f speed;
	Vector2f distance;
}Chassis;

void get_angle(void);
extern float Pitch,Roll,Yaw;
extern Vector3f accel_filter,acc_offset,gyro_filter;	
extern Vector2f imu_speed,imu_pos,SINS_Accel_Body,encorder_pos;
extern Chassis chassis;
#endif