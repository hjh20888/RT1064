#include "imu_cal.h"
#include "headfile.h"
Butter_Parameter Ins_Accel_Parameter;
Butter_Parameter Accel_Parameter;
Butter_Parameter Gyro_Parameter;
Butter_Parameter Calibrate_Parameter;
Butter_BufferData ins_accel_filter_buf[3], gyro_filter_buf_bug[3], accel_filter_buf[3], accel_cal_filter_buf[3], accel_for_cal_filter_buf[3];
float Pitch,Roll,Yaw;
Vector3f gyro_filter = { 0 };	//滤波后的角速度
Vector3f gyro_offset = { 0 }, acc_offset = { 0 };
Vector3f raw_accel_filter = { 0 };	//滤波后的角速度(原始)
Vector3f accel_filter = { 0 };	//滤波后的角速度(姿态)
Vector3f ins_accel_filter = { 0 }; //滤波后的角速度(惯导)

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float constrain_float(float amt, float low, float high) 
{
  // the check for NaN as a float prevents propogation of
  // floating point errors through any function that uses
  // constrain_float(). The normal float semantics already handle -Inf
  // and +Inf
  if (isnan(amt)) {
    return (low+high)*0.5f;
  }
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
#define PI_2 (1.5707963267948966192313216916398f)
#define PI_3 (1.0471975511965977461542144610932f)
#define PI_4 (0.78539816339744830961566084581988f)
#define PI_6 (0.52359877559829887307710723054658f)
#define TWO_MINUS_ROOT3 (0.26794919243112270647255365849413f)
#define SQRT3_MINUS_1 (0.73205080756887729352744634150587f)
#define SQRT3 (1.7320508075688772935274463415059f)
#define EPS_FLOAT (+3.452669830012e-4f)
//Coefficients used for atan/atan2
#define ATANP_COEF0 (-1.44008344874f)
#define ATANP_COEF1 (-7.20026848898e-1f)
#define ATANQ_COEF0 (+4.32025038919f)
#define ATANQ_COEF1 (+4.75222584599f)
//Coefficients used for asin/acos
#define ASINP_COEF1 (-2.7516555290596f)
#define ASINP_COEF2 (+2.9058762374859f)
#define ASINP_COEF3 (-5.9450144193246e-1f)
#define ASINQ_COEF0 (-1.6509933202424e+1f)
#define ASINQ_COEF1 (+2.4864728969164e+1f)
#define ASINQ_COEF2 (-1.0333867072113e+1f)
float FastAtan2(float y, float x)
{
  float f, g;
  float num, den;
  float result;
  int n;
  
  static const float a[4] = {0, (float)PI_6, (float)PI_2, (float)PI_3};
  
  if (x == (float)0.0){
    if (y == (float)0.0){
      result = 0.0;
      return result;
    }
    
    result = (float)PI_2;
    if (y > (float)0.0){
      return result;
    }
    if (y < (float)0.0){
      result = -result;
      return result;
    }
  }
  n = 0;
  num = y;
  den = x;
  
  if (num < (float)0.0){
    num = -num;
  }
  if (den < (float)0.0){
    den = -den;
  }
  if (num > den){
    f = den;
    den = num;
    num = f;
    n = 2;
  }
  f = num / den;
  
  if (f > (float)TWO_MINUS_ROOT3){
    num = f * (float)SQRT3_MINUS_1 - 1.0f + f;
    den = (float)SQRT3 + f;
    f = num / den;
    n = n + 1;
  }
  
  g = f;
  if (g < (float)0.0){
    g = -g;
  }
  
  if (g < (float)EPS_FLOAT){
    result = f;
  }
  else{
    g = f * f;
    num = (ATANP_COEF1 * g + ATANP_COEF0) * g;
    den = (g + ATANQ_COEF1) * g + ATANQ_COEF0;
    result = num / den;
    result = result * f + f;
  }
  if (n > 1){
    result = -result;
  }
  result = result + a[n];
  
  if (x < (float)0.0){
    result = M_PI_F - result;
  }
  if (y < (float)0.0){
    result = -result;
  }
  return result;
}
// Quake inverse square root
float FastSqrtI(float x)
{
  //////////////////////////////////////////////////////////////////////////
  //less accuracy, more faster
  /*
  L2F l2f;
  float xhalf = 0.5f * x;
  l2f.f = x;
  
  l2f.i = 0x5f3759df - (l2f.i >> 1);
  x = l2f.f * (1.5f - xhalf * l2f.f * l2f.f);
  return x;
  */
  //////////////////////////////////////////////////////////////////////////
  union { unsigned int i; float f;} l2f;
  l2f.f = x;
  l2f.i = 0x5F1F1412 - (l2f.i >> 1);
  return l2f.f * (1.69000231f - 0.714158168f * x * l2f.f * l2f.f);
}

float FastSqrt(float x)
{
  return x * FastSqrtI(x);
}
float FastAsin(float x)
{
  float y, g;
  float num, den, result;
  long i;
  float sign = 1.0;
  
  y = x;
  if (y < (float)0.0){
    y = -y;
    sign = -sign;
  }
  
  if (y > (float)0.5){
    i = 1;
    if (y > (float)1.0){
      result = 0.0;
      return result;
    }    
    g = (1.0f - y) * 0.5f;
    y = -2.0f * FastSqrt(g);
  }
  else{
    i = 0;
    if (y < (float)EPS_FLOAT){
      result = y;
      if (sign < (float)0.0){
        result = -result;
      }
      return result;
    }
    g = y * y;
  }
  num = ((ASINP_COEF3 * g + ASINP_COEF2) * g + ASINP_COEF1) * g;
  den = ((g + ASINQ_COEF2) * g + ASINQ_COEF1) * g + ASINQ_COEF0;
  result = num / den;
  result = result * y + y;
  if (i == 1){
    result = result + (float)PI_2;
  }
  if (sign < (float)0.0){
    result = -result;
  }
  return result;
}
float Gyro_Length=0;//陀螺仪模长
void imu_calibration(void)
{
	Vector3f acc_offset_temp;
	int32 gyro_xoffset = 0, gyro_yoffset = 0, gyro_zoffset = 0;
	int32 acc_xoffset = 0, acc_yoffset = 0, acc_zoffset = 0;
	for(int i = 0; i<200;i++)
	{
		get_icm20602_accdata_spi();
		acc_offset_temp.x=LPButterworth(icm_acc_x,&accel_cal_filter_buf[0],&Calibrate_Parameter);
		acc_offset_temp.y=LPButterworth(icm_acc_y,&accel_cal_filter_buf[1],&Calibrate_Parameter);
		acc_offset_temp.z=LPButterworth(icm_acc_z,&accel_cal_filter_buf[2],&Calibrate_Parameter);
		systick_delay_ms(1);
	}
	for (int i = 0; i < 200; i++)			//连续采样30次，一共耗时30*3=90ms
	{
		get_icm20602_accdata_spi();
		get_icm20602_gyro_spi();
		gyro_xoffset += icm_gyro_x;
		gyro_yoffset += icm_gyro_y;
		gyro_zoffset += icm_gyro_z;
		acc_xoffset += LPButterworth(icm_acc_x,&accel_cal_filter_buf[0],&Calibrate_Parameter);
		acc_yoffset += LPButterworth(icm_acc_y,&accel_cal_filter_buf[1],&Calibrate_Parameter);
		acc_zoffset += LPButterworth(icm_acc_z,&accel_cal_filter_buf[2],&Calibrate_Parameter);
		systick_delay_ms(3);
	}
	gyro_offset.x = (gyro_xoffset / 200.f);//得到标定偏移
	gyro_offset.y = (gyro_yoffset / 200.f);
	gyro_offset.z = (gyro_zoffset / 200.f);
	acc_offset.x = (acc_xoffset / 200.f)*ACCEL_TO_1G;
	acc_offset.y = (acc_yoffset / 200.f)*ACCEL_TO_1G;
	acc_offset.z = (acc_zoffset / 200.f)*ACCEL_TO_1G-sqrtf(sq2(GRAVITY_MSS)-sq2(acc_offset.x)-sq2(acc_offset.y));
	acc_offset.x *= One_G_TO_Accel;
	acc_offset.y *= One_G_TO_Accel;
	acc_offset.z *= One_G_TO_Accel;

}

void imu_filter_init()
{
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 50, &Gyro_Parameter);//姿态角速度反馈滤波参数
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 30, &Accel_Parameter);//姿态解算加计修正滤波值 
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 2, &Calibrate_Parameter);//传感器校准加计滤波值
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 60, &Ins_Accel_Parameter);//INS加计滤波值 
	imu_calibration();
}

void imu_get_filter_value()
{
	get_icm20602_accdata_spi();
	get_icm20602_gyro_spi();

	Vector3f gyro, accel;
	gyro.x = icm_gyro_x - gyro_offset.x;
	gyro.y = icm_gyro_y - gyro_offset.y;
	gyro.z = icm_gyro_z - gyro_offset.z;

	accel.x = icm_acc_x - acc_offset.x;
	accel.y = icm_acc_y - acc_offset.y;
	accel.z = icm_acc_z - acc_offset.z;

	gyro_filter.x = LPButterworth(gyro.x, &gyro_filter_buf_bug[0], &Gyro_Parameter);
	gyro_filter.y = LPButterworth(gyro.y, &gyro_filter_buf_bug[1], &Gyro_Parameter);
	gyro_filter.z = LPButterworth(gyro.z, &gyro_filter_buf_bug[2], &Gyro_Parameter);

	accel_filter.x = LPButterworth(accel.x, &accel_filter_buf[0], &Accel_Parameter);
	accel_filter.y = LPButterworth(accel.y, &accel_filter_buf[1], &Accel_Parameter);
	accel_filter.z = LPButterworth(accel.z, &accel_filter_buf[2], &Accel_Parameter);

	ins_accel_filter.x = LPButterworth(accel.x, &ins_accel_filter_buf[0], &Ins_Accel_Parameter);
	ins_accel_filter.y = LPButterworth(accel.y, &ins_accel_filter_buf[1], &Ins_Accel_Parameter);
	ins_accel_filter.z = LPButterworth(accel.z, &ins_accel_filter_buf[2], &Ins_Accel_Parameter);
}

float Yaw_Gyro_Earth_Frame = 0;
#define sampleFreq 200
float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;
volatile float beta = 0.01f;//0.01//0.0075f;
float gx_delta = 0, gy_delta = 0, gz_delta = 0;
float kp = 0;

void AHRS_Update_IMU(float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float gyro_mold)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float beta_temp = 0;

    //IMU_Dt=0.0025f;
    gx *= GYRO_CALIBRATION_COFF;
    gy *= GYRO_CALIBRATION_COFF;
    gz *= GYRO_CALIBRATION_COFF;


    //  gx+=gx_delta;
    //  gy+=gy_delta;
    //  gz+=gz_delta;

      // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * DEG2RAD;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) * DEG2RAD;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) * DEG2RAD;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) * DEG2RAD;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (my * halfwz - mz * halfwy);
        halfey = (mz * halfwx - mx * halfwz);
        halfez = (mx * halfwy - my * halfwx);

        gx_delta = kp * halfex;
        gy_delta = kp * halfey;
        gz_delta = kp * halfez;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        beta_temp = beta + 0.025f * Imu_Sampling_Dt * constrain_float(gyro_mold, 0, 500);//0.035
        beta_temp = constrain_float(beta_temp, beta, 0.06f);

        // Apply feedback step
        qDot1 -= beta_temp * s0;
        qDot2 -= beta_temp * s1;
        qDot3 -= beta_temp * s2;
        qDot4 -= beta_temp * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * Imu_Sampling_Dt;
    q1 += qDot2 * Imu_Sampling_Dt;
    q2 += qDot3 * Imu_Sampling_Dt;
    q3 += qDot4 * Imu_Sampling_Dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

volatile float rMat[3][3];
float Sin_Pitch;
float Cos_Pitch;
float Sin_Roll;
float Cos_Roll;
float Sin_Yaw;
float Cos_Yaw;
void imuComputeRotationMatrix(void)
{
	Sin_Pitch=sin(Pitch* DEG2RAD);
	Cos_Pitch=cos(Pitch* DEG2RAD);
	Sin_Roll=sin(Roll* DEG2RAD);
	Cos_Roll=cos(Roll* DEG2RAD);
	Sin_Yaw=sin(Yaw* DEG2RAD);
  Cos_Yaw=cos(Yaw* DEG2RAD);
	//Inclination_Rate=(rMat[2][2]-Cos_Pitch * Cos_Roll);
	
  rMat[0][0]=Cos_Yaw* Cos_Roll;
  rMat[0][1]=Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw;
  rMat[0][2]=Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw;
  
  rMat[1][0]=Sin_Yaw * Cos_Roll;
  rMat[1][1]=Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw;
  rMat[1][2]=Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;
  
  rMat[2][0]=-Sin_Roll;
  rMat[2][1]= Sin_Pitch * Cos_Roll;
  rMat[2][2]= Cos_Pitch * Cos_Roll;
}

/***********************************************************
@函数名：Vector_From_BodyFrame2EarthFrame
@入口参数：Vector3f *bf,Vector3f *ef
@出口参数：无
功能描述：载体系向导航系转换
*************************************************************/
void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef)
{
  ef->x=rMat[0][0]*bf->x+rMat[0][1]*bf->y+rMat[0][2]*bf->z;
  ef->y=rMat[1][0]*bf->x+rMat[1][1]*bf->y+rMat[1][2]*bf->z;
  ef->z=rMat[2][0]*bf->x+rMat[2][1]*bf->y+rMat[2][2]*bf->z;
}

/***********************************************************
@函数名：Vector_From_EarthFrame2BodyFrame
@入口参数：Vector3f *ef,Vector3f *bf
@出口参数：无
功能描述：导航系向载体系转换
*************************************************************/
void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf)
{
  bf->x=rMat[0][0]*ef->x+rMat[1][0]*ef->y+rMat[2][0]*ef->z;
  bf->y=rMat[0][1]*ef->x+rMat[1][1]*ef->y+rMat[2][1]*ef->z;
  bf->z=rMat[0][2]*ef->x+rMat[1][2]*ef->y+rMat[2][2]*ef->z;
}
Vector2f SINS_Accel_Earth,SINS_Accel_Body;
#define _YAW 0
#define _PITCH 1
#define _ROLL 2
void  SINS_Prepare(void)
{
  Vector2f SINS_Accel_Earth={0};
  Vector3f Body_Frame,Earth_Frame;
	float origin_acc[3];
  /*Z-Y-X欧拉角转方向余弦矩阵
  //Pitch Roll  Yaw 分别对应Φ θ Ψ  
  X轴旋转矩阵
  R（Φ）
  {1      0        0    }
  {0      cosΦ    sinΦ}
  {0    -sinΦ    cosΦ }
  
  Y轴旋转矩阵
  R（θ）
  {cosθ     0        -sinθ}
  {0         1        0     }
  {sinθ     0        cosθ}
  
  Z轴旋转矩阵
  R（θ）
  {cosΨ      sinΨ       0}
  {-sinΨ     cosΨ       0}
  {0          0           1 }
  
  由Z-Y-X顺规有:
  载体坐标系到导航坐标系下旋转矩阵R(b2n)
  R(b2n) =R(Ψ)^T*R(θ)^T*R(Φ)^T
  
  R=
  {cosΨ*cosθ     -cosΦ*sinΨ+sinΦ*sinθ*cosΨ        sinΨ*sinΦ+cosΦ*sinθ*cosΨ}
  {cosθ*sinΨ     cosΦ*cosΨ +sinΦ*sinθ*sinΨ       -cosΨ*sinΦ+cosΦ*sinθ*sinΨ}
  {-sinθ          cosθsin Φ                          cosθcosΦ                   }
  */
  Body_Frame.x=ins_accel_filter.x;
  Body_Frame.y=ins_accel_filter.y;
  Body_Frame.z=ins_accel_filter.z;
	

	imuComputeRotationMatrix();
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  origin_acc[_YAW]=Earth_Frame.z;
  origin_acc[_PITCH]=Earth_Frame.x;
  origin_acc[_ROLL]=Earth_Frame.y;
  
  origin_acc[_YAW]*=GRAVITY_MSS/AcceMax_1G;
  origin_acc[_YAW]-=GRAVITY_MSS;//减去重力加速度
  origin_acc[_YAW]*=100;//加速度cm/s^2
  
  origin_acc[_PITCH]*=GRAVITY_MSS/AcceMax_1G;
  origin_acc[_PITCH]*=100;//加速度cm/s^2
  
  origin_acc[_ROLL]*=GRAVITY_MSS/AcceMax_1G;
  origin_acc[_ROLL]*=100;//加速度cm/s^2
  
  /******************************************************************************/
  //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚
  
  SINS_Accel_Earth.x=origin_acc[_PITCH];//沿地理坐标系，正东方向运动加速度,单位为CM
  SINS_Accel_Earth.y=origin_acc[_ROLL];//沿地理坐标系，正北方向运动加速度,单位为CM
  
  SINS_Accel_Body.x= SINS_Accel_Earth.x*Cos_Yaw+SINS_Accel_Earth.y*Sin_Yaw;  //横滚正向运动加速度  X轴正向
  SINS_Accel_Body.y=-SINS_Accel_Earth.x*Sin_Yaw+SINS_Accel_Earth.y*Cos_Yaw; //机头正向运动加速度  Y轴正向
  
}
Vector2f imu_speed={0},imu_pos={0};
Vector2f encorder_pos={0};

/*#define ENCORDER_PRCISION	512.f	//512线、1024线等
#define ENCORDER_D 		20.8f		//编码器齿轮直径
#define WHEEL_D				55.0f		//车轮直径
#define	WHEEL_GEAR_D 	42.8f		//车轮直连齿轮直径
#define GET_DISTANCE_MM(val) ((((val/ENCORDER_PRCISION)*ENCORDER_D*M_PI_F)*WHEEL_D)/WHEEL_GEAR_D/1 )
#define GET_DISTANCE_CM(val) ((((val/ENCORDER_PRCISION)*ENCORDER_D*M_PI_F)*WHEEL_D)/WHEEL_GEAR_D/10)*/

Chassis chassis={0};
void get_imu_position()
{
	Vector2f speed_delta,speed_temp;
	Vector2f encorder_speed,speed_err,pos_err;
	
	speed_temp.x = GET_DISTANCE_CM(((chassis.L1+chassis.R2)-(chassis.R1+chassis.L2))/4.f) / Imu_Sampling_Dt ;
	speed_temp.y = GET_DISTANCE_CM((chassis.L1+chassis.R1+chassis.L2+chassis.R2)/4.f)/ Imu_Sampling_Dt ;
	
	encorder_speed.x = speed_temp.x*Cos_Yaw + speed_temp.y*Sin_Yaw;	
	encorder_speed.y = speed_temp.x*Sin_Yaw + speed_temp.y*Cos_Yaw;
	
	
	chassis.distance.x += speed_temp.x * Imu_Sampling_Dt;
	chassis.distance.y += speed_temp.y * Imu_Sampling_Dt;

	encorder_pos.x += encorder_speed.x*0.003f;
	encorder_pos.y += encorder_speed.y*0.003f;
	
	speed_err.x = encorder_speed.x - imu_speed.x;
	speed_err.y = encorder_speed.y - imu_speed.y;
	pos_err.x = encorder_pos.x - imu_pos.x;
	pos_err.y = encorder_pos.y - imu_pos.y;
	
	speed_delta.x=SINS_Accel_Body.x*0.003f;		//往右+
  speed_delta.y=SINS_Accel_Body.y*0.003f;		//往前+
	
	imu_pos.x+=imu_speed.x*0.003f+0.5f*speed_delta.x*0.003f + pos_err.x*0;
	imu_pos.y+=imu_speed.y*0.003f+0.5f*speed_delta.y*0.003f + pos_err.y*0;
	
	imu_speed.x+=speed_delta.x + speed_err.x*0.02; //0.02
	imu_speed.y+=speed_delta.y + speed_err.y*0.02;
	
}

volatile float Yaw_Gyro=0,Pitch_Gyro=0,Roll_Gyro=0;//俯仰角速度、横滚角速度、偏航角速度

void get_angle()
{
	imu_get_filter_value();
	AHRS_Update_IMU(gyro_filter.x,gyro_filter.y,gyro_filter.z
                          ,accel_filter.x,accel_filter.y,accel_filter.z,
    0, 0, 0,
    Gyro_Length);
	Pitch= FastAtan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f *q1 *q1 - 2.0f * q2* q2 + 1.0f) * RAD2DEG;		// Pitch
	Roll= FastAsin(2.0f * q0* q2-2.0f * q1 * q3) * RAD2DEG;	
	Yaw = FastAtan2(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q3 *q3 - 2.0f * q2 * q2 + 1.0f) * RAD2DEG;		// Yaw
	
	if(Yaw<0.0f)   Yaw+=360.0f;
    if(Yaw>360.0f) Yaw-=360.0f;	
	Pitch_Gyro=gyro_filter.x*GYRO_CALIBRATION_COFF;
    Roll_Gyro=gyro_filter.y*GYRO_CALIBRATION_COFF;
    Yaw_Gyro=gyro_filter.z*GYRO_CALIBRATION_COFF;
    Gyro_Length=FastSqrt(Yaw_Gyro*Yaw_Gyro+Pitch_Gyro*Pitch_Gyro+Roll_Gyro*Roll_Gyro);//单位deg/s
	
	SINS_Prepare();
	get_imu_position();
}