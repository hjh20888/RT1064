#include "headfile.h"

//Control信号量
rt_sem_t Control_sem;

//Mode_select信号量
rt_sem_t Mode_select_sem;

//Parameter_calculation信号量
rt_sem_t Parameter_calculation_sem;

//Control互斥量控制块指针
rt_mutex_t Control_mutex = RT_NULL;	

//各种PID结构体变量
PID increase_L1_PID;		//增量式pid	L1
PID increase_R1_PID;		//增量式pid	R1
PID increase_L2_PID;		//增量式pid	L2
PID increase_R2_PID;		//增量式pid	R2

PID angle_PID;					//角度环pid
PID angle_speed_PID;		//角速度环pid

PID attitude_X_PID;			//姿态调整PID X
PID attitude_Y_PID;			//姿态调整PID Y

//麦克纳姆轮全向运动参数
short int Mecanum_X, Mecanum_Y, Mecanum_Z;	

//最终输出速度
short int expect_speed_L1, expect_speed_R1, expect_speed_L2, expect_speed_R2;

//按键调参
float key_P=0,key_I=0,key_D=0;

//编码器读取到的值
short int encoder_L1 = 0,encoder_R1 = 0;
short int encoder_L2 = 0,encoder_R2 = 0;
//编码器滤波后的值
short int now_speed_L1 = 0, now_speed_R1 = 0;	
short int now_speed_L2 = 0, now_speed_R2 = 0;

//X Y方向的平移距离
float DISTANCE_X = 0.f, DISTANCE_Y = 0.f;

//期望角度与距离
float expect_angle =0.f, Length = 0.f;

//X Y方向期望速度
short int Expect_X = 0,Expect_Y = 0;

//转向环启动标志位
short int turn_flag = 1;	//值为1时开始转向，0时结束转向

//控制模式选择
unsigned char control_mode = 0;	 //0 前往坐标点		1 调整姿态		2 搬运图片

//记录当前坐标号
unsigned char num = 0;

//Control互斥量释放后标志位	搬运完成后释放互斥量 进行参数计算 前往下个坐标点
unsigned char Control_mutex_release_flag = 0;// 1表示需要释放	

//存储图片信息
unsigned char picture_data[2];	//存储图片大小类

//图片信息
unsigned char picture_message[10] = {2,3,1,1,1,0,0,0,0,0};
unsigned char picture_flag = 0;


//-------------------------------------------------------------------------------------------------------------------
//  brief      编码器值的处理
//  return     void
//  since      v1.0
//  Sample usage:	5ms执行一次
//-------------------------------------------------------------------------------------------------------------------

void encoder_deal(void)	//编码器值的处理
{
		//static float old_speed_L1 = 0, old_speed_R1 = 0;
		//static float old_speed_L2 = 0, old_speed_R2 = 0;
	
		//读取编码器计数值	这里需要注意第二个参数务必填写A相引脚
	  	chassis.L1 = -qtimer_quad_get(ENCODER1_QTIMER,ENCODER1_A);
	  	chassis.R1 =  qtimer_quad_get(ENCODER2_QTIMER,ENCODER2_A);
		chassis.L2 = -qtimer_quad_get(ENCODER3_QTIMER,ENCODER3_A); 
		chassis.R2 =  qtimer_quad_get(ENCODER4_QTIMER,ENCODER4_A); 
	
		//对编码器值进行滑动平均滤波
		now_speed_L1 = Moving_average_filter(&speed_L1_Moving_average,chassis.L1);
		now_speed_R1 = Moving_average_filter(&speed_R1_Moving_average,chassis.R1);
		now_speed_L2 = Moving_average_filter(&speed_L2_Moving_average,chassis.L2);
		now_speed_R2 = Moving_average_filter(&speed_R2_Moving_average,chassis.R2);
	
		/*now_speed_L1 = kalman_filter(&speed_L1_KalmanInfo,encoder_L1);
		now_speed_R1 = kalman_filter(&speed_R1_KalmanInfo,encoder_R1);
		now_speed_L2 = kalman_filter(&speed_L2_KalmanInfo,encoder_L2);
		now_speed_R2 = kalman_filter(&speed_R2_KalmanInfo,encoder_R2);*/
	
		//计数器清零
		qtimer_quad_clear(ENCODER1_QTIMER,ENCODER1_A);		//L1
		qtimer_quad_clear(ENCODER2_QTIMER,ENCODER2_A);		//R1
		qtimer_quad_clear(ENCODER3_QTIMER,ENCODER3_A);		//L2
		qtimer_quad_clear(ENCODER4_QTIMER,ENCODER4_A);		//R2
		
		
		if(turn_flag == 0)
		{
				DISTANCE_Y +=  fabs(GET_DISTANCE_CM(( chassis.L1+chassis.R1+chassis.L2+chassis.R2 )/4.0f)*0.95);
				DISTANCE_X +=  fabs(GET_DISTANCE_CM(( chassis.L1+chassis.R2-chassis.L2-chassis.R1 )/4.0f)*0.95);//*0.95;
		}	
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      限幅
//  return     short int
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

short int limit_value(short int now , short int high , short int low)	//输出限幅
{
		return (now <= low ? (low) : (now >= high ? high : now));
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      速度环 增量式pid
//  return     short int
//  since      v2.0
//  Sample usage:								 
//										比例P :    e(k)-e(k-1)   这次误差-上次误差
//										积分I :  	 e(i)    			 误差
//										微分D :  	 e(k) - 2e(k-1)+e(k-2)   这次误差-2*上次误差+上上次误差
//										(P可以消除I的抖动)
//
//															对应PID的地址			当前转速 		期望转速
//									speed_PID(&increase_L1_PID , now_speed_L1 , 30)//速度环 增量式pid
//-------------------------------------------------------------------------------------------------------------------

short int speed_PID(PID *increase_PID , short int now_speed , short int expect_speed)	//速度环 增量式pid
{
		short int duty_back = 0;	//需要调节的占空比
	
		//increase_PID->P = (float)key_P;
		//increase_PID->I = (float)key_I;
	
		increase_PID->error = expect_speed - now_speed;	//记录这次误差
	
		increase_PID->error_rate = increase_PID->error - increase_PID->error_last;//这次误差-上次误差
		increase_PID->error_rate_pro = increase_PID->error - 2*increase_PID->error_last + increase_PID->error_last_last;//这次误差-2*上次误差+上上次误差
	
		//对控制量进行累加
		increase_PID->error_add += (increase_PID->P * increase_PID->error_rate) + (increase_PID->I * increase_PID->error) + (increase_PID->D * increase_PID->error_rate_pro);	
		
		//限幅
		increase_PID->error_add = limit_value(increase_PID->error_add , increase_PID->limit , -increase_PID->limit);	//输出限幅
			
		increase_PID->error_last_last = increase_PID->error_last;	//上上次误差
		increase_PID->error_last = increase_PID->error;	// 上次误差
		
		duty_back = increase_PID->error_add;
		
		return duty_back;
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      位置式pid
//  return     short int
//  since      v2.0
//  Sample usage:		
//									比例P :   e(k)           误差
//                	积分I :   e(I)+=e(k)     误差的累加
//                	微分D :   e(k)-e(k-1)    这次误差-上次误差
//
//															对应PID的地址			当前转速 		期望转速
//								location_PID(&location_L1_PID , now_speed_L1 , 30)//位置式pid
//-------------------------------------------------------------------------------------------------------------------

short int location_PID(PID *location_PID , short int now_speed , short int expect_speed)//位置式pid
{
		//location_PID->P = (float)((p_Menu+0)->add_sub[4])*0.01;
		//location_PID->I = (float)((p_Menu+0)->add_sub[5])*0.01;
		
		location_PID->error = expect_speed - now_speed;
		location_PID->error_add += location_PID->error;	//误差的累加
		location_PID->error_add = limit_value(location_PID->error_add , 70 , -70);	//输出限幅
	
		location_PID->error_last = location_PID->error; //记录上次次误差
	
		
		location_PID->out = (location_PID->P * location_PID->error) + (location_PID->I * location_PID->error_add); 
		
		//location_PID->out = limit_value(location_PID->out , location_PID->limit , -location_PID->limit);	//输出限幅
	
		return location_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      角度环pid
//  return     short int
//  since      v1.0
//  Sample usage:			Mecanum_Z = Angle_PID(&angle_PID , yaw , 90);
//										在角速度环前面执行					 
//										角度环，角速度环，速度环（直立:角速度环，角度环，速度环）
//-------------------------------------------------------------------------------------------------------------------
 	

short int Angle_PID3(PID *angle_PID , float now_angle , float expect_angle)//角度环pid3
{
		//angle_PID->P = (float)key_P*1; ///86
		//angle_PID->I = (float)key_I*0.1;	//0.2
		
		angle_PID->error = expect_angle - now_angle;
		angle_PID->error_add += angle_PID->error;	//误差的累加
		angle_PID->error_add = limit_value(angle_PID->error_add , 1800 , -1800);	//输出限幅
		
		angle_PID->error_rate = angle_PID->error - angle_PID->error_last;//这次误差减上次误差
		
		angle_PID->out = (angle_PID->P * angle_PID->error) + (angle_PID->I * angle_PID->error_add); 
		
	  angle_PID->error_last = angle_PID->error; //记录上次误差
	
		return angle_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      角速度环pid
//  return     short int
//  since      v1.0
//  Sample usage:			在速度环前面执行					 
//										角度环，角速度环，速度环（直立:角速度环，角度环，速度环）
//-------------------------------------------------------------------------------------------------------------------

short int Angle_speed_PID(PID *angle_speed_PID , short int now_angle_speed , short int expect_angle_speed)	//角速度环pid
{
		//angle_speed_PID->P = (float)key_P*0.001;
		//angle_speed_PID->D = (float)key_D*0.001;
		
		angle_speed_PID->error = expect_angle_speed - now_angle_speed;
		angle_speed_PID->error_rate = angle_speed_PID->error - angle_speed_PID->error_last;//这次误差-上次误差
	
		angle_speed_PID->out = (angle_speed_PID->P * angle_speed_PID->error) + angle_speed_PID->D * (angle_speed_PID->error_rate);
		
		angle_speed_PID->error_last = angle_speed_PID->error;
		
		return angle_speed_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      姿态调整pid
//  return     short int
//  since      v1.0
//  Sample usage:														对应PID的地址		当前中心点纵坐标	期望纵坐标
//										Mecanum_Y =  Attitude_PID(&attitude_Y_PID , center_point[1] , 80);
//										Mecanum_X = -Attitude_PID(&attitude_X_PID , center_point[0] , 92);
//-------------------------------------------------------------------------------------------------------------------

short int Attitude_PID(PID *attitude_PID , short int now_attitude , short int expect_attitude)	//姿态调整pid
{
		//attitude_PID->P = (float)key_P*0.01;
		//attitude_PID->I = (float)key_I*0.001;
		//attitude_PID->D = (float)key_D*0.01;
		
		attitude_PID->error = expect_attitude - now_attitude;
		attitude_PID->error_add += attitude_PID->error;	//误差的累加
		attitude_PID->error_rate = attitude_PID->error - attitude_PID->error_last;//这次误差-上次误差
	
		attitude_PID->out = (attitude_PID->P * attitude_PID->error) + (attitude_PID->I * attitude_PID->error_add) + attitude_PID->D * (attitude_PID->error_rate);
		
		attitude_PID->error_last = attitude_PID->error;
		
		return attitude_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      麦克纳姆全向控制
//  return     void
//  since      v1.0
//  Sample usage:	在速度环前面执行	
//								Mecanum_X : X轴正向 	正右方
//							  Mecanum_Y :	Y轴正向		正前方
//								Mecanum_Z :	旋转轴		逆时针方向
//-------------------------------------------------------------------------------------------------------------------

void Mecanum_Solution(short int Mecanum_X, short int Mecanum_Y, short int Mecanum_Z)//麦克纳姆全向控制
{
	expect_speed_L1 = Mecanum_Y + Mecanum_X - Mecanum_Z;
    expect_speed_R1 = Mecanum_Y - Mecanum_X + Mecanum_Z;
    expect_speed_L2 = Mecanum_Y - Mecanum_X - Mecanum_Z;
    expect_speed_R2 = Mecanum_Y + Mecanum_X + Mecanum_Z;
}	


//-------------------------------------------------------------------------------------------------------------------
//  brief      速度控制
//  return     void
//  since      v2.0
//  Sample usage:	5ms执行一次
//-------------------------------------------------------------------------------------------------------------------

void speed_control(short int expect_speed_L1,short int expect_speed_R1,short int expect_speed_L2,short int expect_speed_R2)		//转速控制
{
		short int duty_L1=0,duty_R1=0;	//车头左右轮占空比
		short int duty_L2=0,duty_R2=0;	//车尾左右轮占空比
	
		//速度环 增量式pid	
		duty_L1 = speed_PID(&increase_L1_PID , now_speed_L1 , expect_speed_L1);
		duty_R1 = speed_PID(&increase_R1_PID , now_speed_R1 , expect_speed_R1);
		duty_L2 = speed_PID(&increase_L2_PID , now_speed_L2 , expect_speed_L2);
		duty_R2 = speed_PID(&increase_R2_PID , now_speed_R2 , expect_speed_R2);
		
		//pwm输出与方向控制
		if(duty_R2 >= 0)   {gpio_set(DIR_4,0); pwm_duty(PWM_4, duty_R2);}	
		else {gpio_set(DIR_4,1); pwm_duty(PWM_4,-duty_R2);}
		
		if (duty_L1 >= 0)  {gpio_set(DIR_1,1); pwm_duty(PWM_1, duty_L1);}	
		else  {gpio_set(DIR_1,0); pwm_duty(PWM_1,-duty_L1);}
		
		if (duty_R1 >= 0)  {gpio_set(DIR_2,0); pwm_duty(PWM_2, duty_R1);}
		else {gpio_set(DIR_2,1); pwm_duty(PWM_2,-duty_R1);}
		
		if (duty_L2 >= 0)  {gpio_set(DIR_3,1); pwm_duty(PWM_3, duty_L2);}
		else {gpio_set(DIR_3,0); pwm_duty(PWM_3,-duty_L2);}
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      运动方向控制
//  return     void
//  since      v1.0
//  Sample usage:		Direction_control(0, 20, 90);			
//									不是需要Z的原因是 Z轴速度 由角度环计算
//-------------------------------------------------------------------------------------------------------------------

void Direction_control(short int expect_X, short int expect_Y, float expect_angle) //运动方向控制
{	
	  if((abs(yaw) < abs(expect_angle)-1 || abs(yaw) > abs(expect_angle)+1) && turn_flag==1)
		{
				Mecanum_Z = Angle_PID3(&angle_PID , real_yaw , expect_angle);//角度环pid
				Mecanum_Z = Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
				Mecanum_Solution(0, 0, Mecanum_Z);//麦克纳姆全向控制	X Y Z
		}
		else 
		{		
				turn_flag=0;//值为1时开始转向，0时结束转向
				Mecanum_Z = Angle_PID3(&angle_PID , real_yaw , expect_angle);//角度环pid
				Mecanum_Z = Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
				Mecanum_Solution(expect_X, expect_Y, Mecanum_Z);//麦克纳姆全向控制	X Y Z
		}	
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      参数计算
//  return     void
//  since      v1.0
//  Sample usage:		计算期望角度与距离
//-------------------------------------------------------------------------------------------------------------------
float angle_error = 0.f;

void Parameter_calculation(void)	//参数计算	计算期望角度与距离
{
			//now_x = (unsigned char)BlackPoint[black_order[num+1]][0]*5 +1;
			//now_y = (unsigned char)BlackPoint[black_order[num+1]][1]*5 +1;
			Path_planning2();//路径规划2
	
			Length = pow(pow(current_location_X-next_location_X,2) + pow(current_location_Y-next_location_Y,2), 0.5) + 0.05;
			angle_error = atan2(next_location_Y-(current_location_Y-0.1), next_location_X-current_location_X)*180/PI;
			
			if(angle_error>=90.0f && angle_error<=180.0f)		//90 - 180
			{
					expect_angle = angle_error - 90.0f;
			}	
			
			if(angle_error<0.f && angle_error>=-90.0f) 	 		//0 - -90
			{
					expect_angle = 90.0f + angle_error;
			}	
			
			if(angle_error>=0.f && angle_error<90.0f)		 		//0-90
			{
					expect_angle = angle_error - 90.0f;
			}	
			
			if(angle_error>-180.0f && angle_error<-90.0f)	//-90 - -180 
			{
					expect_angle = angle_error + 90.0f;
			}	
			
			if(angle_error >= 0)
			{
					Expect_Y = 15;
			}	
			else if(angle_error < 0)
			{
					Expect_Y = -15;
			}
			
			
			if(num == black_num-1)
			{
					//Length = fabs(current_location_X - 0.1) - 0.1; //+ 0.1; 有误差 少跑0.1
					Length = fabs(current_location_X - 0.1);
					expect_angle = 90.f;
					if(current_location_X - 0.2 > 0)
						Expect_Y = 15;
					else
						Expect_Y = -15;
			}	
			
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      open art图片识别 获取图片数据	
//  return     void
//  since      v1.0
//  Sample usage:	 获取图片大类 小类
//-------------------------------------------------------------------------------------------------------------------
//unsigned char picture_data[2];	//存储图片大小类

void get_picture(void) //open art图片识别 获取图片数据
{
		//发送标志位，开始识别
		uart1_data[1] = '0';
		while(uart1_data[1] == '0')
				openart_send(); //串口通信 数据发送
		
		//识别后将数组内数据清零，并在此做出相应动作
		if(uart4_data[1]==1 || uart4_data[1]==2 || uart4_data[1]==3 || uart4_data[1]==4 || uart4_data[1]==5)
		{
				picture_data[0] = uart4_data[1];	//大类
				picture_data[1] = uart4_data[2];	//小类
			
				//信息发送
				//message[14] = uart4_data[1];	//大类
				//message[16] = uart4_data[2];	//小类
				//message_format_send();	//信息格式化发送
				//uart_putstr(USART_8,message);//信息发送 100ms一次	识别后立即发送一条
		}	
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      姿态调整
//  return     void
//  since      v2.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void Attitude_control(void) //姿态调整
{
		static int i = 0;
		
		if(i==0)
		{
				DISTANCE_Y = 0; //计步清零
				DISTANCE_X = 0;
				i++;
		}

		//调整完毕	执行下一步
		if( (center_point[0]>=160&&center_point[0]<=170) && (center_point[1]>=160&&center_point[1]<=170)) //172 134
		{
				i = 0;
			
				Expect_X = 0;
				Expect_Y = 0;
				
				openart_send();
				picture_data[0] = uart1_data[4];
				//rt_thread_mdelay(500);
				//picture_data[0] = picture_message[picture_flag++];
					
				if(picture_data[0] != 5)
				{
						//吸取图片
						gpio_set(B9,1);
						gpio_set(B10,1);
						pwm_duty(S_MOTOR_PIN,1550);		//1000最低	5000最高 //16600
						rt_thread_mdelay(1500);
						pwm_duty(S_MOTOR_PIN,2200);		//1000最低	5000最高
					
						DISTANCE_Y = 0;//姿态调整完毕 计步清零
						
						control_mode = 2;	 //姿态调整完毕	开始搬运			0 前往坐标点		1 调整姿态		2 搬运图片
				}	
				else
				{
						DISTANCE_Y = 0; //计步清零
		
						num++;
					
						//位置更新
						current_location_X = next_location_X;
						current_location_Y = next_location_Y;
						
						turn_flag=1;//值为1时开始转向，0时结束转向
						control_mode = 0;	 //搬运完毕 前往下个坐标点			0 前往坐标点		1 调整姿态		2 搬运图片
				}	
				
		}
	

}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      搬运图片
//  return     void
//  since      v2.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void Picture_handling(void)	//搬运图片
{
		switch(picture_data[0])
		{
			case 1://左边
				{
						angle_error = 180.0f;
						Length = next_location_X + 0.15;
						expect_angle = 90.0f;
					
						next_location_X = 0.f; //-0.15f;
				}
				break;
			case 2://右边
				{
						Length = 3.0f - next_location_X;
						expect_angle = -90.f;
				
						next_location_X = 3.0f; //+ 0.15f;
				}
				break;
			case 3://上边
				{
						Length = 3.0f - next_location_Y ;
						expect_angle = 0.f;
			
						next_location_Y = 3.0f ;
				}
				break;
				
			case 4://下边
				{
						Length = next_location_Y + 0.20f;
						expect_angle = 0.f;
			
						next_location_Y = -0.15f;
				}
				break;
		}	
		
		while(real_yaw<expect_angle-1 || real_yaw>expect_angle+1){gpio_set(BUZZER_PIN,1);}//等待转向完毕;关闭蜂鸣器
		gpio_set(BUZZER_PIN,0);//关闭蜂鸣器
		
		
		DISTANCE_Y = 0; //计步清零
		
		Expect_Y = 15;
		if(picture_data[0] == 4)
				Expect_Y = -15;
		//speed_PID_Transformation(); //速度环参数变换
		
		while(DISTANCE_Y < Length)
		{
				if(Length - DISTANCE_Y < 0.6)
				{
					if(Expect_Y > 0)
						Expect_Y = 15;
					else
						Expect_Y = -15;
			 }     
		}	
		
		Expect_X = 0;
		Expect_Y = 0;	
		
		//放下图片
		gpio_set(B9,0);
		gpio_set(B10,0);
		pwm_duty(S_MOTOR_PIN,4875);		//1000最低	5000最高
		
		//防止转向时车轮在图片上打滑
		if(picture_data[0]==1 || picture_data[0]==2)
		{
				DISTANCE_Y = 0; //计步清零
				while(DISTANCE_Y < 0.09)
				{
						Expect_Y = -15;    
				}	
				
				Expect_X = 0;
				Expect_Y = 0;	
		}	
		
		
		turn_flag=1;//值为1时开始转向，0时结束转向
		expect_angle = 0.f;
		while(yaw>1 || yaw<-1){gpio_set(BUZZER_PIN,1);}//等待转向完毕;关闭蜂鸣器
		gpio_set(BUZZER_PIN,0);//关闭蜂鸣器
		
		DISTANCE_Y = 0; //计步清零
		
		num++;
		
		//位置更新
		current_location_X = next_location_X;
		current_location_Y = next_location_Y;
		
		
		turn_flag=1;//值为1时开始转向，0时结束转向
		control_mode = 0;	 //搬运完毕 前往下个坐标点			0 前往坐标点		1 调整姿态		2 搬运图片
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      回车库
//  return     void
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------
void Back_garage(void)	//回车库
{
	
		Length = current_location_Y;
		DISTANCE_Y = 0; //计步清零
		
		while(DISTANCE_Y < Length)
		{
				Expect_Y = -20;
		}	
		
		Expect_X = 0;
		Expect_Y = 0;	
		DISTANCE_Y = 0;
		
		//如果车从场地下方入库 则将车往前走出库 再倒入库中
		/*if(current_location_Y < 0)
		{
				Length = 0.5f;
				while(DISTANCE_Y < Length)
				{
						Expect_Y = 20;
				}	
				Expect_X = 0;
				Expect_Y = 0;	
				DISTANCE_Y = 0;
				
				Length = 0.8f;
				while(DISTANCE_Y < Length)
				{
						Expect_Y = -20;
				}	
				Expect_X = 0;
				Expect_Y = 0;	
		}	*/
		
		while(1);
}	

//------------------------------------------------------------------------------------------------------------------
//  brief      总控制
//  return     void
//  since      v2.0
//  Sample usage:		Total_control();		
//------------------------------------------------------------------------------------------------------------------

void Total_control(void) //总控制
{
		if(num <= black_num-1)
				DISTANCE_Y = 0; //计步清零
		else
				while(1){Expect_Y = 0;}
				
		//speed_PID_Transformation(); //速度环参数变换		
		while(DISTANCE_Y < Length)
		{	
			
				if(Length - DISTANCE_Y < 0.5)
				{
					if(Expect_Y > 0)
						Expect_Y = 15;
					else
						Expect_Y = -15;
				}     
		}	

		Expect_X = 0;
		Expect_Y = 0;
		
		static short int yaw_is_zero_flag = 0; //判断当前角度是否为零 1为0度
		expect_angle = 0;
		if(yaw_is_zero_flag == 0)
		{
				while(yaw>1 || yaw<-1){gpio_set(BUZZER_PIN,1);}//等待转向完毕;关闭蜂鸣器
				gpio_set(BUZZER_PIN,0);//关闭蜂鸣器
		
				yaw_is_zero_flag = 1; //判断当前角度是否为零 1为0度
		}	
		
		yaw_is_zero_flag = 0; //判断当前角度是否为零 1为0度
		
		if(num < black_num-1)
				control_mode = 1;//0 前往坐标点		1 调整姿态		2 搬运图片
		else
				Back_garage();	//回车库
}

/*******************************************************************************************************************
*  brief      Mode_select线程
*  return     void
*  since      v1.0
*  Sample usage:	  
*******************************************************************************************************************/

void Mode_select_entry(void *parameter)
{
    while(1)
    {
				if(GoGoGo)
				{
						switch(control_mode)
						{
								case 0:	if(turn_flag==0){Total_control();} break;	//总控制	
								case 1:	Attitude_control(); break;  //姿态调整	
								case 2: Picture_handling(); break;	//搬运图片
						}
				}	
				else
				{
						rt_thread_mdelay(10);
				}	
			
    }
}

void Mode_select_init(void) //Mode_select线程
{
    rt_thread_t tid;
	
    //创建Mode_select的线程 优先级20
    tid = rt_thread_create("Mode_select", Mode_select_entry, RT_NULL, 1024, 20, 10);
    
    //启动线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}

/*******************************************************************************************************************
*  brief      Control线程
*  return     void
*  since      v1.0
*  Sample usage:	  获取到信号量后开始运行	5ms一次
*******************************************************************************************************************/

void Control_entry(void *parameter)
{
    while(1)
    {
        //获取Control信号量, 如果没有则持续等待并释放CPU控制权
        rt_sem_take(Control_sem,RT_WAITING_FOREVER);
			
				rt_enter_critical();    //API：进入临界区，退出前系统不会发生任务调度
			
				static short int count = -1;//计算当前点数
				if(count != num)
				{
						Parameter_calculation();	//参数计算	计算期望角度与距离
						count++;
				}	
				
				static unsigned char back_car_flag = 0;//姿态调整倒车标志位		
				
			  switch(control_mode)
				{
					case 0:	//前往坐标点
						{
								if(expect_angle < 0)	//右转
										real_yaw = transform_yaw(0, yaw, expect_angle);//yaw角度转换
								else
										real_yaw = transform_yaw(1, yaw, expect_angle);//yaw角度转换
								
								Direction_control(0, Expect_Y, expect_angle);//运动方向控制 
						}
						break;
					case 1:	//调整姿态
						{
								center_point[0] = uart1_data[1] + uart1_data[3];
								center_point[1] = uart1_data[2];
								
								if(center_point[1]!=0 && center_point[0]!=0)
								{
										back_car_flag++;//倒车标志位
									
										Mecanum_X = -Attitude_PID(&attitude_X_PID , center_point[0] , 164);
										Mecanum_Y =  Attitude_PID(&attitude_Y_PID , center_point[1] , 164);
										Mecanum_Z =  Angle_PID3(&angle_PID , yaw , 0);//角度环pid
										Mecanum_Z =  Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
										Mecanum_Solution(Mecanum_X, Mecanum_Y, Mecanum_Z);//麦克纳姆全向控制	X Y Z
								}	
								else ///if(back_car_flag == 0)
								{
										Mecanum_Z = Angle_PID3(&angle_PID , yaw , 0);//角度环pid
										Mecanum_Z = Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
										Mecanum_Solution(Mecanum_X, Mecanum_Y, Mecanum_Z);//麦克纳姆全向控制	X Y Z
								}	
						}
						break;
					case 2:	//搬运图片
						{
								back_car_flag = 0;//倒车标志位 清零
							
								if(expect_angle < 0)	//右转
										real_yaw = transform_yaw(0, yaw, expect_angle);//yaw角度转换
								else
										real_yaw = transform_yaw(1, yaw, expect_angle);//yaw角度转换
								
								Direction_control(0, Expect_Y, expect_angle);//运动方向控制 
						}
						break;
				}
				
				speed_control(expect_speed_L1,expect_speed_R1,expect_speed_L2,expect_speed_R2);	//转速控制
						
				rt_exit_critical();    //API：退出临界区
				
    }
}

void Control_init(void) //Control线程
{
    rt_thread_t tid;
	
	//灯
	gpio_init(B21, GPO, 1, GPIO_PIN_CONFIG);			// 初始化为GPIO浮空输入 默认上拉高电平
	
	//舵机初始化
	pwm_init(S_MOTOR_PIN,50,4875);
	
	//电磁铁
	gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);			// 初始化为GPIO浮空输入 默认上拉高电平
	gpio_init(B10, GPO, 0, GPIO_PIN_CONFIG);			// 初始化为GPIO浮空输入 默认上拉高电平
    
	//电机方向
	gpio_init(DIR_1, GPO, 1, GPIO_PIN_CONFIG); 		//B14 初始化DIR_1	GPIO
	gpio_init(DIR_2, GPO, 0, GPIO_PIN_CONFIG); 		//B15 
	gpio_init(DIR_3, GPO, 1, GPIO_PIN_CONFIG);    //B21
    gpio_init(DIR_4, GPO, 0, GPIO_PIN_CONFIG);    //B23
	//PWM引脚初始化
	pwm_init(PWM_1, 17000, 0);     //D2 初始化PWM_1 周期17K 占空比0
	pwm_init(PWM_2, 17000, 0);     //D3 
    pwm_init(PWM_3, 17000, 0);     //D12			
    pwm_init(PWM_4, 17000, 0);     //D13				
	
	//编码器初始化
	//一个QTIMER可以 创建两个正交解码，其定义在zf_qtimer.h文件中
    //这里需要注意一下，如果是带方向输出的编码器，编码器的LSB引脚应该与A相连接 DIR引脚应该与B相连接 不可交叉
	qtimer_quad_init(ENCODER1_QTIMER,ENCODER1_A,ENCODER1_B);			//L1
	qtimer_quad_init(ENCODER2_QTIMER,ENCODER2_A,ENCODER2_B);			//R1
    qtimer_quad_init(ENCODER3_QTIMER,ENCODER3_A,ENCODER3_B);			//L2
    qtimer_quad_init(ENCODER4_QTIMER,ENCODER4_A,ENCODER4_B);			//R2
	
	// 创建动态互斥量
	Control_mutex = rt_mutex_create("Control_mutex", RT_IPC_FLAG_FIFO);
	
	//创建Control的信号量，8ms中断释放一次
    Control_sem = rt_sem_create("Control", 0, RT_IPC_FLAG_FIFO);		
    
    //创建Control的线程 优先级19
    tid = rt_thread_create("Control", Control_entry, RT_NULL, 1024, 19, 3);
    
    //启动线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}


