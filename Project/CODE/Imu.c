#include "headfile.h"

//Imu信号量
rt_sem_t Imu_sem;

//俯仰角/横滚角/航向角/转换后的航向角
float pitch,roll,yaw,real_yaw;		


//-------------------------------------------------------------------------------------------------------------------
//  brief      MPU6050姿态角与加速度计数据获取
//  return     void
//  since      v1.0
//  Sample usage:	  周期一般设置为3ms		只获取yaw，减少计算量
//-------------------------------------------------------------------------------------------------------------------

void get_icm20602(void)//姿态角与加速度计数据获取
{
		get_icm20602_accdata_spi();
		get_icm20602_gyro_spi();
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      yaw角度转换
//  return     float
//  since      v1.0
//  Sample usage:		mode=0:右转		mode=1:左转
//									real_yaw = transform_yaw(0, yaw, -90);
//-------------------------------------------------------------------------------------------------------------------

float transform_yaw(unsigned char mode, float yaw, float expect_yaw) //yaw角度转换
{
		if(mode == 0)	//右转
		{
				if(yaw > 0)
						yaw = -360 + yaw;
				if(yaw < -180)
				{
						yaw = 360 + yaw;
				}
		}	
		else	//左转
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
*  brief      Imu线程
*  return     void
*  since      v1.0
*  Sample usage:	  获取到信号量后开始运行	3ms运行一次
*******************************************************************************************************************/

void Imu_entry(void *parameter)
{
    while(1)
    {
        //获取Imu信号量，如果没有则持续等待并释放CPU控制权
        rt_sem_take(Imu_sem,RT_WAITING_FOREVER);

        //while(mpu_dmp_get_data(&pitch,&roll,&yaw)==0);	//姿态角获取	只获取yaw，减少计算量
			
    }
}

void Imu_init(void) //Imu线程
{
    rt_thread_t tid;
	
		
		//icm20602_init_spi();
		//imu_filter_init();
	
	simiic_init();
    mpu6050_init();
	while(mpu_dmp_init())//dmp初始化
	{
		ips114_showstr(0,1,"error");
		systick_delay_ms(200);
		ips114_showstr(0,1,"     ");
		systick_delay_ms(200);
	}
	
    //创建Imu的信号量，3ms中断释放一次
    Imu_sem = rt_sem_create("Imu", 0, RT_IPC_FLAG_FIFO);		
    
    //创建IMU的线程 优先级19
    tid = rt_thread_create("Imu", Imu_entry, RT_NULL, 1024, 19, 2);
    
    //启动线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}