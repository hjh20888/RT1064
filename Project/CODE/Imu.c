#include "headfile.h"

//Imu�ź���
rt_sem_t Imu_sem;

//������/�����/�����/ת����ĺ����
float pitch,roll,yaw,real_yaw;		


//-------------------------------------------------------------------------------------------------------------------
//  brief      MPU6050��̬������ٶȼ����ݻ�ȡ
//  return     void
//  since      v1.0
//  Sample usage:	  ����һ������Ϊ3ms		ֻ��ȡyaw�����ټ�����
//-------------------------------------------------------------------------------------------------------------------

void get_icm20602(void)//��̬������ٶȼ����ݻ�ȡ
{
		get_icm20602_accdata_spi();
		get_icm20602_gyro_spi();
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      yaw�Ƕ�ת��
//  return     float
//  since      v1.0
//  Sample usage:		mode=0:��ת		mode=1:��ת
//									real_yaw = transform_yaw(0, yaw, -90);
//-------------------------------------------------------------------------------------------------------------------

float transform_yaw(unsigned char mode, float yaw, float expect_yaw) //yaw�Ƕ�ת��
{
		if(mode == 0)	//��ת
		{
				if(yaw > 0)
						yaw = -360 + yaw;
				if(yaw < -180)
				{
						yaw = 360 + yaw;
				}
		}	
		else	//��ת
		{
				if(yaw < 0)
						yaw = 360 + yaw;
				if(yaw > 180)
				{
						yaw = -360 + yaw;
				}	
		}	
			
		return yaw;
}	


/*******************************************************************************************************************
*  brief      Imu�߳�
*  return     void
*  since      v1.0
*  Sample usage:	  ��ȡ���ź�����ʼ����	3ms����һ��
*******************************************************************************************************************/

void Imu_entry(void *parameter)
{
    while(1)
    {
        //��ȡImu�ź��������û��������ȴ����ͷ�CPU����Ȩ
        rt_sem_take(Imu_sem,RT_WAITING_FOREVER);

        //while(mpu_dmp_get_data(&pitch,&roll,&yaw)==0);	//��̬�ǻ�ȡ	ֻ��ȡyaw�����ټ�����
			
    }
}

void Imu_init(void) //Imu�߳�
{
    rt_thread_t tid;
	
		
		//icm20602_init_spi();
		//imu_filter_init();
	
	simiic_init();
    mpu6050_init();
	while(mpu_dmp_init())//dmp��ʼ��
	{
		ips114_showstr(0,1,"error");
		systick_delay_ms(200);
		ips114_showstr(0,1,"     ");
		systick_delay_ms(200);
	}
	
    //����Imu���ź�����3ms�ж��ͷ�һ��
    Imu_sem = rt_sem_create("Imu", 0, RT_IPC_FLAG_FIFO);		
    
    //����IMU���߳� ���ȼ�19
    tid = rt_thread_create("Imu", Imu_entry, RT_NULL, 1024, 19, 2);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}