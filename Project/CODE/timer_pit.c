#include "headfile.h"


//-------------------------------------------------------------------------------------------------------------------
//  brief      vofa+��λ��
//  return     void
//  since      v1.0
//  Sample usage: ���ж�5ms
//-------------------------------------------------------------------------------------------------------------------

void View_vofa(void) //vofa��λ��������
{
		UploadDataJF.CH_Data[0] = (float)now_speed_L1;
        UploadDataJF.CH_Data[1] = (float)now_speed_R1;
		UploadDataJF.CH_Data[2] = (float)now_speed_L2; 
		UploadDataJF.CH_Data[3] = (float)now_speed_R2;
		//UploadDataJF.CH_Data[3] = (float)acc_Y;
		//UploadDataJF.CH_Data[4] = (float)ACC[2];
		//UploadDataJF.CH_Data[5] = (float)acc_Z;
	
		VOFAplusUpload_JustFloat(&UploadDataJF);
}	

/*******************************************************************************************************************
*  brief      timer1��ʱ��
*  return     void
*  since      v1.0
*  Sample usage:	  1��ϵͳ����(1ms)�ж�һ��  Ӳ����ʱ��	������  ���ڶ�ʱ�ͷ��ź��������Ƹ��̵߳�����
*******************************************************************************************************************/

void timer1_pit_entry(void *parameter) 
{
    static uint32 time;
    time++;
		//��SWITCH_1 D27 �ر�ʱ���ͷ��ź���
		if(GoGoGo)
		{
				if(0 == (time%3))
				{
						//3ms�ͷ�һ��Imu�ź�������ȡһ��Imu����
						//���ź���Ϊ��ʱ���ͷ�
						//if(Imu_sem->value == 0)
							///rt_sem_release(Imu_sem);
					
						while(mpu_dmp_get_data(&pitch,&roll,&yaw)==0);	//��̬�ǻ�ȡ	ֻ��ȡyaw�����ټ�����
				}	
				
				if(0 == (time%3))
				{
						encoder_deal();	//������ֵ�Ĵ���
					
						//5ms�ͷ�һ��Control�ź���������һ�ο��Ƹ���
						if(Control_sem->value == 0)
								rt_sem_release(Control_sem);	
						
				}
				
//				if(0 == (time%8))
//				{
//						//8ms�ͷ�һ��Image�ź���������һ��ͼ����
//						//if(Image_sem->value == 0)
//						//	rt_sem_release(Image_sem);
//						
//						//View_vofa(); //vofa��λ��������
//				}	
				
				
//				if(0 == (time%100))
//				{
//						//100ms�ͷ�һ��Message�ź���������һ�γ�ģ��Ϣ����
//						if(Message_sem->value == 0)
//							rt_sem_release(Message_sem);
//					
//				}	
		}
		
		if(!GoGoGo)	//�ж��Ƿ���Image�ź��� 1ʱ����
		{
				//8ms�ͷ�һ��Image�ź���������һ��ͼ����
				if(Image_sem->value == 0)
					rt_sem_release(Image_sem);
				
				//View_vofa(); //vofa��λ��������
		}	
		
//		if(0 == (time%3))
//		{
//				encoder_deal();	//������ֵ�Ĵ���
//		}
		
}


void timer_pit_init(void)
{
    rt_timer_t timer;
			
		//����ת����ģ��������Ŷ����� wireless.h�ļ���
	  seekfree_wireless_init();
	
    //����һ����ʱ�� �������� 1ms
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    
    //������ʱ��
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }
    
}
