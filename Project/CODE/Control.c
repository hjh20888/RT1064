#include "headfile.h"

//Control�ź���
rt_sem_t Control_sem;

//Mode_select�ź���
rt_sem_t Mode_select_sem;

//Parameter_calculation�ź���
rt_sem_t Parameter_calculation_sem;

//Control���������ƿ�ָ��
rt_mutex_t Control_mutex = RT_NULL;	

//����PID�ṹ�����
PID increase_L1_PID;		//����ʽpid	L1
PID increase_R1_PID;		//����ʽpid	R1
PID increase_L2_PID;		//����ʽpid	L2
PID increase_R2_PID;		//����ʽpid	R2

PID angle_PID;					//�ǶȻ�pid
PID angle_speed_PID;		//���ٶȻ�pid

PID attitude_X_PID;			//��̬����PID X
PID attitude_Y_PID;			//��̬����PID Y

//�����ķ��ȫ���˶�����
short int Mecanum_X, Mecanum_Y, Mecanum_Z;	

//��������ٶ�
short int expect_speed_L1, expect_speed_R1, expect_speed_L2, expect_speed_R2;

//��������
float key_P=0,key_I=0,key_D=0;

//��������ȡ����ֵ
short int encoder_L1 = 0,encoder_R1 = 0;
short int encoder_L2 = 0,encoder_R2 = 0;
//�������˲����ֵ
short int now_speed_L1 = 0, now_speed_R1 = 0;	
short int now_speed_L2 = 0, now_speed_R2 = 0;

//X Y�����ƽ�ƾ���
float DISTANCE_X = 0.f, DISTANCE_Y = 0.f;

//�����Ƕ������
float expect_angle =0.f, Length = 0.f;

//X Y���������ٶ�
short int Expect_X = 0,Expect_Y = 0;

//ת��������־λ
short int turn_flag = 1;	//ֵΪ1ʱ��ʼת��0ʱ����ת��

//����ģʽѡ��
unsigned char control_mode = 0;	 //0 ǰ�������		1 ������̬		2 ����ͼƬ

//��¼��ǰ�����
unsigned char num = 0;

//Control�������ͷź��־λ	������ɺ��ͷŻ����� ���в������� ǰ���¸������
unsigned char Control_mutex_release_flag = 0;// 1��ʾ��Ҫ�ͷ�	

//�洢ͼƬ��Ϣ
unsigned char picture_data[2];	//�洢ͼƬ��С��

//ͼƬ��Ϣ
unsigned char picture_message[10] = {2,3,1,1,1,0,0,0,0,0};
unsigned char picture_flag = 0;


//-------------------------------------------------------------------------------------------------------------------
//  brief      ������ֵ�Ĵ���
//  return     void
//  since      v1.0
//  Sample usage:	5msִ��һ��
//-------------------------------------------------------------------------------------------------------------------

void encoder_deal(void)	//������ֵ�Ĵ���
{
		//static float old_speed_L1 = 0, old_speed_R1 = 0;
		//static float old_speed_L2 = 0, old_speed_R2 = 0;
	
		//��ȡ����������ֵ	������Ҫע��ڶ������������дA������
	  	chassis.L1 = -qtimer_quad_get(ENCODER1_QTIMER,ENCODER1_A);
	  	chassis.R1 =  qtimer_quad_get(ENCODER2_QTIMER,ENCODER2_A);
		chassis.L2 = -qtimer_quad_get(ENCODER3_QTIMER,ENCODER3_A); 
		chassis.R2 =  qtimer_quad_get(ENCODER4_QTIMER,ENCODER4_A); 
	
		//�Ա�����ֵ���л���ƽ���˲�
		now_speed_L1 = Moving_average_filter(&speed_L1_Moving_average,chassis.L1);
		now_speed_R1 = Moving_average_filter(&speed_R1_Moving_average,chassis.R1);
		now_speed_L2 = Moving_average_filter(&speed_L2_Moving_average,chassis.L2);
		now_speed_R2 = Moving_average_filter(&speed_R2_Moving_average,chassis.R2);
	
		/*now_speed_L1 = kalman_filter(&speed_L1_KalmanInfo,encoder_L1);
		now_speed_R1 = kalman_filter(&speed_R1_KalmanInfo,encoder_R1);
		now_speed_L2 = kalman_filter(&speed_L2_KalmanInfo,encoder_L2);
		now_speed_R2 = kalman_filter(&speed_R2_KalmanInfo,encoder_R2);*/
	
		//����������
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
//  brief      �޷�
//  return     short int
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

short int limit_value(short int now , short int high , short int low)	//����޷�
{
		return (now <= low ? (low) : (now >= high ? high : now));
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      �ٶȻ� ����ʽpid
//  return     short int
//  since      v2.0
//  Sample usage:								 
//										����P :    e(k)-e(k-1)   ������-�ϴ����
//										����I :  	 e(i)    			 ���
//										΢��D :  	 e(k) - 2e(k-1)+e(k-2)   ������-2*�ϴ����+���ϴ����
//										(P��������I�Ķ���)
//
//															��ӦPID�ĵ�ַ			��ǰת�� 		����ת��
//									speed_PID(&increase_L1_PID , now_speed_L1 , 30)//�ٶȻ� ����ʽpid
//-------------------------------------------------------------------------------------------------------------------

short int speed_PID(PID *increase_PID , short int now_speed , short int expect_speed)	//�ٶȻ� ����ʽpid
{
		short int duty_back = 0;	//��Ҫ���ڵ�ռ�ձ�
	
		//increase_PID->P = (float)key_P;
		//increase_PID->I = (float)key_I;
	
		increase_PID->error = expect_speed - now_speed;	//��¼������
	
		increase_PID->error_rate = increase_PID->error - increase_PID->error_last;//������-�ϴ����
		increase_PID->error_rate_pro = increase_PID->error - 2*increase_PID->error_last + increase_PID->error_last_last;//������-2*�ϴ����+���ϴ����
	
		//�Կ����������ۼ�
		increase_PID->error_add += (increase_PID->P * increase_PID->error_rate) + (increase_PID->I * increase_PID->error) + (increase_PID->D * increase_PID->error_rate_pro);	
		
		//�޷�
		increase_PID->error_add = limit_value(increase_PID->error_add , increase_PID->limit , -increase_PID->limit);	//����޷�
			
		increase_PID->error_last_last = increase_PID->error_last;	//���ϴ����
		increase_PID->error_last = increase_PID->error;	// �ϴ����
		
		duty_back = increase_PID->error_add;
		
		return duty_back;
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      λ��ʽpid
//  return     short int
//  since      v2.0
//  Sample usage:		
//									����P :   e(k)           ���
//                	����I :   e(I)+=e(k)     �����ۼ�
//                	΢��D :   e(k)-e(k-1)    ������-�ϴ����
//
//															��ӦPID�ĵ�ַ			��ǰת�� 		����ת��
//								location_PID(&location_L1_PID , now_speed_L1 , 30)//λ��ʽpid
//-------------------------------------------------------------------------------------------------------------------

short int location_PID(PID *location_PID , short int now_speed , short int expect_speed)//λ��ʽpid
{
		//location_PID->P = (float)((p_Menu+0)->add_sub[4])*0.01;
		//location_PID->I = (float)((p_Menu+0)->add_sub[5])*0.01;
		
		location_PID->error = expect_speed - now_speed;
		location_PID->error_add += location_PID->error;	//�����ۼ�
		location_PID->error_add = limit_value(location_PID->error_add , 70 , -70);	//����޷�
	
		location_PID->error_last = location_PID->error; //��¼�ϴδ����
	
		
		location_PID->out = (location_PID->P * location_PID->error) + (location_PID->I * location_PID->error_add); 
		
		//location_PID->out = limit_value(location_PID->out , location_PID->limit , -location_PID->limit);	//����޷�
	
		return location_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      �ǶȻ�pid
//  return     short int
//  since      v1.0
//  Sample usage:			Mecanum_Z = Angle_PID(&angle_PID , yaw , 90);
//										�ڽ��ٶȻ�ǰ��ִ��					 
//										�ǶȻ������ٶȻ����ٶȻ���ֱ��:���ٶȻ����ǶȻ����ٶȻ���
//-------------------------------------------------------------------------------------------------------------------
 	

short int Angle_PID3(PID *angle_PID , float now_angle , float expect_angle)//�ǶȻ�pid3
{
		//angle_PID->P = (float)key_P*1; ///86
		//angle_PID->I = (float)key_I*0.1;	//0.2
		
		angle_PID->error = expect_angle - now_angle;
		angle_PID->error_add += angle_PID->error;	//�����ۼ�
		angle_PID->error_add = limit_value(angle_PID->error_add , 1800 , -1800);	//����޷�
		
		angle_PID->error_rate = angle_PID->error - angle_PID->error_last;//��������ϴ����
		
		angle_PID->out = (angle_PID->P * angle_PID->error) + (angle_PID->I * angle_PID->error_add); 
		
	  angle_PID->error_last = angle_PID->error; //��¼�ϴ����
	
		return angle_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      ���ٶȻ�pid
//  return     short int
//  since      v1.0
//  Sample usage:			���ٶȻ�ǰ��ִ��					 
//										�ǶȻ������ٶȻ����ٶȻ���ֱ��:���ٶȻ����ǶȻ����ٶȻ���
//-------------------------------------------------------------------------------------------------------------------

short int Angle_speed_PID(PID *angle_speed_PID , short int now_angle_speed , short int expect_angle_speed)	//���ٶȻ�pid
{
		//angle_speed_PID->P = (float)key_P*0.001;
		//angle_speed_PID->D = (float)key_D*0.001;
		
		angle_speed_PID->error = expect_angle_speed - now_angle_speed;
		angle_speed_PID->error_rate = angle_speed_PID->error - angle_speed_PID->error_last;//������-�ϴ����
	
		angle_speed_PID->out = (angle_speed_PID->P * angle_speed_PID->error) + angle_speed_PID->D * (angle_speed_PID->error_rate);
		
		angle_speed_PID->error_last = angle_speed_PID->error;
		
		return angle_speed_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      ��̬����pid
//  return     short int
//  since      v1.0
//  Sample usage:														��ӦPID�ĵ�ַ		��ǰ���ĵ�������	����������
//										Mecanum_Y =  Attitude_PID(&attitude_Y_PID , center_point[1] , 80);
//										Mecanum_X = -Attitude_PID(&attitude_X_PID , center_point[0] , 92);
//-------------------------------------------------------------------------------------------------------------------

short int Attitude_PID(PID *attitude_PID , short int now_attitude , short int expect_attitude)	//��̬����pid
{
		//attitude_PID->P = (float)key_P*0.01;
		//attitude_PID->I = (float)key_I*0.001;
		//attitude_PID->D = (float)key_D*0.01;
		
		attitude_PID->error = expect_attitude - now_attitude;
		attitude_PID->error_add += attitude_PID->error;	//�����ۼ�
		attitude_PID->error_rate = attitude_PID->error - attitude_PID->error_last;//������-�ϴ����
	
		attitude_PID->out = (attitude_PID->P * attitude_PID->error) + (attitude_PID->I * attitude_PID->error_add) + attitude_PID->D * (attitude_PID->error_rate);
		
		attitude_PID->error_last = attitude_PID->error;
		
		return attitude_PID->out;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      �����ķȫ�����
//  return     void
//  since      v1.0
//  Sample usage:	���ٶȻ�ǰ��ִ��	
//								Mecanum_X : X������ 	���ҷ�
//							  Mecanum_Y :	Y������		��ǰ��
//								Mecanum_Z :	��ת��		��ʱ�뷽��
//-------------------------------------------------------------------------------------------------------------------

void Mecanum_Solution(short int Mecanum_X, short int Mecanum_Y, short int Mecanum_Z)//�����ķȫ�����
{
	expect_speed_L1 = Mecanum_Y + Mecanum_X - Mecanum_Z;
    expect_speed_R1 = Mecanum_Y - Mecanum_X + Mecanum_Z;
    expect_speed_L2 = Mecanum_Y - Mecanum_X - Mecanum_Z;
    expect_speed_R2 = Mecanum_Y + Mecanum_X + Mecanum_Z;
}	


//-------------------------------------------------------------------------------------------------------------------
//  brief      �ٶȿ���
//  return     void
//  since      v2.0
//  Sample usage:	5msִ��һ��
//-------------------------------------------------------------------------------------------------------------------

void speed_control(short int expect_speed_L1,short int expect_speed_R1,short int expect_speed_L2,short int expect_speed_R2)		//ת�ٿ���
{
		short int duty_L1=0,duty_R1=0;	//��ͷ������ռ�ձ�
		short int duty_L2=0,duty_R2=0;	//��β������ռ�ձ�
	
		//�ٶȻ� ����ʽpid	
		duty_L1 = speed_PID(&increase_L1_PID , now_speed_L1 , expect_speed_L1);
		duty_R1 = speed_PID(&increase_R1_PID , now_speed_R1 , expect_speed_R1);
		duty_L2 = speed_PID(&increase_L2_PID , now_speed_L2 , expect_speed_L2);
		duty_R2 = speed_PID(&increase_R2_PID , now_speed_R2 , expect_speed_R2);
		
		//pwm����뷽�����
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
//  brief      �˶��������
//  return     void
//  since      v1.0
//  Sample usage:		Direction_control(0, 20, 90);			
//									������ҪZ��ԭ���� Z���ٶ� �ɽǶȻ�����
//-------------------------------------------------------------------------------------------------------------------

void Direction_control(short int expect_X, short int expect_Y, float expect_angle) //�˶��������
{	
	  if((abs(yaw) < abs(expect_angle)-1 || abs(yaw) > abs(expect_angle)+1) && turn_flag==1)
		{
				Mecanum_Z = Angle_PID3(&angle_PID , real_yaw , expect_angle);//�ǶȻ�pid
				Mecanum_Z = Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
				Mecanum_Solution(0, 0, Mecanum_Z);//�����ķȫ�����	X Y Z
		}
		else 
		{		
				turn_flag=0;//ֵΪ1ʱ��ʼת��0ʱ����ת��
				Mecanum_Z = Angle_PID3(&angle_PID , real_yaw , expect_angle);//�ǶȻ�pid
				Mecanum_Z = Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
				Mecanum_Solution(expect_X, expect_Y, Mecanum_Z);//�����ķȫ�����	X Y Z
		}	
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      ��������
//  return     void
//  since      v1.0
//  Sample usage:		���������Ƕ������
//-------------------------------------------------------------------------------------------------------------------
float angle_error = 0.f;

void Parameter_calculation(void)	//��������	���������Ƕ������
{
			//now_x = (unsigned char)BlackPoint[black_order[num+1]][0]*5 +1;
			//now_y = (unsigned char)BlackPoint[black_order[num+1]][1]*5 +1;
			Path_planning2();//·���滮2
	
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
					//Length = fabs(current_location_X - 0.1) - 0.1; //+ 0.1; ����� ����0.1
					Length = fabs(current_location_X - 0.1);
					expect_angle = 90.f;
					if(current_location_X - 0.2 > 0)
						Expect_Y = 15;
					else
						Expect_Y = -15;
			}	
			
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      open artͼƬʶ�� ��ȡͼƬ����	
//  return     void
//  since      v1.0
//  Sample usage:	 ��ȡͼƬ���� С��
//-------------------------------------------------------------------------------------------------------------------
//unsigned char picture_data[2];	//�洢ͼƬ��С��

void get_picture(void) //open artͼƬʶ�� ��ȡͼƬ����
{
		//���ͱ�־λ����ʼʶ��
		uart1_data[1] = '0';
		while(uart1_data[1] == '0')
				openart_send(); //����ͨ�� ���ݷ���
		
		//ʶ����������������㣬���ڴ�������Ӧ����
		if(uart4_data[1]==1 || uart4_data[1]==2 || uart4_data[1]==3 || uart4_data[1]==4 || uart4_data[1]==5)
		{
				picture_data[0] = uart4_data[1];	//����
				picture_data[1] = uart4_data[2];	//С��
			
				//��Ϣ����
				//message[14] = uart4_data[1];	//����
				//message[16] = uart4_data[2];	//С��
				//message_format_send();	//��Ϣ��ʽ������
				//uart_putstr(USART_8,message);//��Ϣ���� 100msһ��	ʶ�����������һ��
		}	
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      ��̬����
//  return     void
//  since      v2.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void Attitude_control(void) //��̬����
{
		static int i = 0;
		
		if(i==0)
		{
				DISTANCE_Y = 0; //�Ʋ�����
				DISTANCE_X = 0;
				i++;
		}

		//�������	ִ����һ��
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
						//��ȡͼƬ
						gpio_set(B9,1);
						gpio_set(B10,1);
						pwm_duty(S_MOTOR_PIN,1550);		//1000���	5000��� //16600
						rt_thread_mdelay(1500);
						pwm_duty(S_MOTOR_PIN,2200);		//1000���	5000���
					
						DISTANCE_Y = 0;//��̬������� �Ʋ�����
						
						control_mode = 2;	 //��̬�������	��ʼ����			0 ǰ�������		1 ������̬		2 ����ͼƬ
				}	
				else
				{
						DISTANCE_Y = 0; //�Ʋ�����
		
						num++;
					
						//λ�ø���
						current_location_X = next_location_X;
						current_location_Y = next_location_Y;
						
						turn_flag=1;//ֵΪ1ʱ��ʼת��0ʱ����ת��
						control_mode = 0;	 //������� ǰ���¸������			0 ǰ�������		1 ������̬		2 ����ͼƬ
				}	
				
		}
	

}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����ͼƬ
//  return     void
//  since      v2.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void Picture_handling(void)	//����ͼƬ
{
		switch(picture_data[0])
		{
			case 1://���
				{
						angle_error = 180.0f;
						Length = next_location_X + 0.15;
						expect_angle = 90.0f;
					
						next_location_X = 0.f; //-0.15f;
				}
				break;
			case 2://�ұ�
				{
						Length = 3.0f - next_location_X;
						expect_angle = -90.f;
				
						next_location_X = 3.0f; //+ 0.15f;
				}
				break;
			case 3://�ϱ�
				{
						Length = 3.0f - next_location_Y ;
						expect_angle = 0.f;
			
						next_location_Y = 3.0f ;
				}
				break;
				
			case 4://�±�
				{
						Length = next_location_Y + 0.20f;
						expect_angle = 0.f;
			
						next_location_Y = -0.15f;
				}
				break;
		}	
		
		while(real_yaw<expect_angle-1 || real_yaw>expect_angle+1){gpio_set(BUZZER_PIN,1);}//�ȴ�ת�����;�رշ�����
		gpio_set(BUZZER_PIN,0);//�رշ�����
		
		
		DISTANCE_Y = 0; //�Ʋ�����
		
		Expect_Y = 15;
		if(picture_data[0] == 4)
				Expect_Y = -15;
		//speed_PID_Transformation(); //�ٶȻ������任
		
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
		
		//����ͼƬ
		gpio_set(B9,0);
		gpio_set(B10,0);
		pwm_duty(S_MOTOR_PIN,4875);		//1000���	5000���
		
		//��ֹת��ʱ������ͼƬ�ϴ�
		if(picture_data[0]==1 || picture_data[0]==2)
		{
				DISTANCE_Y = 0; //�Ʋ�����
				while(DISTANCE_Y < 0.09)
				{
						Expect_Y = -15;    
				}	
				
				Expect_X = 0;
				Expect_Y = 0;	
		}	
		
		
		turn_flag=1;//ֵΪ1ʱ��ʼת��0ʱ����ת��
		expect_angle = 0.f;
		while(yaw>1 || yaw<-1){gpio_set(BUZZER_PIN,1);}//�ȴ�ת�����;�رշ�����
		gpio_set(BUZZER_PIN,0);//�رշ�����
		
		DISTANCE_Y = 0; //�Ʋ�����
		
		num++;
		
		//λ�ø���
		current_location_X = next_location_X;
		current_location_Y = next_location_Y;
		
		
		turn_flag=1;//ֵΪ1ʱ��ʼת��0ʱ����ת��
		control_mode = 0;	 //������� ǰ���¸������			0 ǰ�������		1 ������̬		2 ����ͼƬ
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      �س���
//  return     void
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------
void Back_garage(void)	//�س���
{
	
		Length = current_location_Y;
		DISTANCE_Y = 0; //�Ʋ�����
		
		while(DISTANCE_Y < Length)
		{
				Expect_Y = -20;
		}	
		
		Expect_X = 0;
		Expect_Y = 0;	
		DISTANCE_Y = 0;
		
		//������ӳ����·���� �򽫳���ǰ�߳��� �ٵ������
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
//  brief      �ܿ���
//  return     void
//  since      v2.0
//  Sample usage:		Total_control();		
//------------------------------------------------------------------------------------------------------------------

void Total_control(void) //�ܿ���
{
		if(num <= black_num-1)
				DISTANCE_Y = 0; //�Ʋ�����
		else
				while(1){Expect_Y = 0;}
				
		//speed_PID_Transformation(); //�ٶȻ������任		
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
		
		static short int yaw_is_zero_flag = 0; //�жϵ�ǰ�Ƕ��Ƿ�Ϊ�� 1Ϊ0��
		expect_angle = 0;
		if(yaw_is_zero_flag == 0)
		{
				while(yaw>1 || yaw<-1){gpio_set(BUZZER_PIN,1);}//�ȴ�ת�����;�رշ�����
				gpio_set(BUZZER_PIN,0);//�رշ�����
		
				yaw_is_zero_flag = 1; //�жϵ�ǰ�Ƕ��Ƿ�Ϊ�� 1Ϊ0��
		}	
		
		yaw_is_zero_flag = 0; //�жϵ�ǰ�Ƕ��Ƿ�Ϊ�� 1Ϊ0��
		
		if(num < black_num-1)
				control_mode = 1;//0 ǰ�������		1 ������̬		2 ����ͼƬ
		else
				Back_garage();	//�س���
}

/*******************************************************************************************************************
*  brief      Mode_select�߳�
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
								case 0:	if(turn_flag==0){Total_control();} break;	//�ܿ���	
								case 1:	Attitude_control(); break;  //��̬����	
								case 2: Picture_handling(); break;	//����ͼƬ
						}
				}	
				else
				{
						rt_thread_mdelay(10);
				}	
			
    }
}

void Mode_select_init(void) //Mode_select�߳�
{
    rt_thread_t tid;
	
    //����Mode_select���߳� ���ȼ�20
    tid = rt_thread_create("Mode_select", Mode_select_entry, RT_NULL, 1024, 20, 10);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}

/*******************************************************************************************************************
*  brief      Control�߳�
*  return     void
*  since      v1.0
*  Sample usage:	  ��ȡ���ź�����ʼ����	5msһ��
*******************************************************************************************************************/

void Control_entry(void *parameter)
{
    while(1)
    {
        //��ȡControl�ź���, ���û��������ȴ����ͷ�CPU����Ȩ
        rt_sem_take(Control_sem,RT_WAITING_FOREVER);
			
				rt_enter_critical();    //API�������ٽ������˳�ǰϵͳ���ᷢ���������
			
				static short int count = -1;//���㵱ǰ����
				if(count != num)
				{
						Parameter_calculation();	//��������	���������Ƕ������
						count++;
				}	
				
				static unsigned char back_car_flag = 0;//��̬����������־λ		
				
			  switch(control_mode)
				{
					case 0:	//ǰ�������
						{
								if(expect_angle < 0)	//��ת
										real_yaw = transform_yaw(0, yaw, expect_angle);//yaw�Ƕ�ת��
								else
										real_yaw = transform_yaw(1, yaw, expect_angle);//yaw�Ƕ�ת��
								
								Direction_control(0, Expect_Y, expect_angle);//�˶�������� 
						}
						break;
					case 1:	//������̬
						{
								center_point[0] = uart1_data[1] + uart1_data[3];
								center_point[1] = uart1_data[2];
								
								if(center_point[1]!=0 && center_point[0]!=0)
								{
										back_car_flag++;//������־λ
									
										Mecanum_X = -Attitude_PID(&attitude_X_PID , center_point[0] , 164);
										Mecanum_Y =  Attitude_PID(&attitude_Y_PID , center_point[1] , 164);
										Mecanum_Z =  Angle_PID3(&angle_PID , yaw , 0);//�ǶȻ�pid
										Mecanum_Z =  Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
										Mecanum_Solution(Mecanum_X, Mecanum_Y, Mecanum_Z);//�����ķȫ�����	X Y Z
								}	
								else ///if(back_car_flag == 0)
								{
										Mecanum_Z = Angle_PID3(&angle_PID , yaw , 0);//�ǶȻ�pid
										Mecanum_Z = Angle_speed_PID(&angle_speed_PID , GYRO[2]  , Mecanum_Z);
										Mecanum_Solution(Mecanum_X, Mecanum_Y, Mecanum_Z);//�����ķȫ�����	X Y Z
								}	
						}
						break;
					case 2:	//����ͼƬ
						{
								back_car_flag = 0;//������־λ ����
							
								if(expect_angle < 0)	//��ת
										real_yaw = transform_yaw(0, yaw, expect_angle);//yaw�Ƕ�ת��
								else
										real_yaw = transform_yaw(1, yaw, expect_angle);//yaw�Ƕ�ת��
								
								Direction_control(0, Expect_Y, expect_angle);//�˶�������� 
						}
						break;
				}
				
				speed_control(expect_speed_L1,expect_speed_R1,expect_speed_L2,expect_speed_R2);	//ת�ٿ���
						
				rt_exit_critical();    //API���˳��ٽ���
				
    }
}

void Control_init(void) //Control�߳�
{
    rt_thread_t tid;
	
	//��
	gpio_init(B21, GPO, 1, GPIO_PIN_CONFIG);			// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	
	//�����ʼ��
	pwm_init(S_MOTOR_PIN,50,4875);
	
	//�����
	gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);			// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	gpio_init(B10, GPO, 0, GPIO_PIN_CONFIG);			// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    
	//�������
	gpio_init(DIR_1, GPO, 1, GPIO_PIN_CONFIG); 		//B14 ��ʼ��DIR_1	GPIO
	gpio_init(DIR_2, GPO, 0, GPIO_PIN_CONFIG); 		//B15 
	gpio_init(DIR_3, GPO, 1, GPIO_PIN_CONFIG);    //B21
    gpio_init(DIR_4, GPO, 0, GPIO_PIN_CONFIG);    //B23
	//PWM���ų�ʼ��
	pwm_init(PWM_1, 17000, 0);     //D2 ��ʼ��PWM_1 ����17K ռ�ձ�0
	pwm_init(PWM_2, 17000, 0);     //D3 
    pwm_init(PWM_3, 17000, 0);     //D12			
    pwm_init(PWM_4, 17000, 0);     //D13				
	
	//��������ʼ��
	//һ��QTIMER���� ���������������룬�䶨����zf_qtimer.h�ļ���
    //������Ҫע��һ�£�����Ǵ���������ı���������������LSB����Ӧ����A������ DIR����Ӧ����B������ ���ɽ���
	qtimer_quad_init(ENCODER1_QTIMER,ENCODER1_A,ENCODER1_B);			//L1
	qtimer_quad_init(ENCODER2_QTIMER,ENCODER2_A,ENCODER2_B);			//R1
    qtimer_quad_init(ENCODER3_QTIMER,ENCODER3_A,ENCODER3_B);			//L2
    qtimer_quad_init(ENCODER4_QTIMER,ENCODER4_A,ENCODER4_B);			//R2
	
	// ������̬������
	Control_mutex = rt_mutex_create("Control_mutex", RT_IPC_FLAG_FIFO);
	
	//����Control���ź�����8ms�ж��ͷ�һ��
    Control_sem = rt_sem_create("Control", 0, RT_IPC_FLAG_FIFO);		
    
    //����Control���߳� ���ȼ�19
    tid = rt_thread_create("Control", Control_entry, RT_NULL, 1024, 19, 3);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}


