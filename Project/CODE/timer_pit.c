#include "headfile.h"


//-------------------------------------------------------------------------------------------------------------------
//  brief      vofa+上位机
//  return     void
//  since      v1.0
//  Sample usage: 放中断5ms
//-------------------------------------------------------------------------------------------------------------------

void View_vofa(void) //vofa上位机看数据
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
*  brief      timer1定时器
*  return     void
*  since      v1.0
*  Sample usage:	  1个系统节拍(1ms)中断一次  硬件定时器	快进快出  用于定时释放信号量来控制各线程的运行
*******************************************************************************************************************/

void timer1_pit_entry(void *parameter) 
{
    static uint32 time;
    time++;
		//当SWITCH_1 D27 关闭时，释放信号量
		if(GoGoGo)
		{
				if(0 == (time%3))
				{
						//3ms释放一次Imu信号量，获取一次Imu数据
						//当信号量为零时再释放
						//if(Imu_sem->value == 0)
							///rt_sem_release(Imu_sem);
					
						while(mpu_dmp_get_data(&pitch,&roll,&yaw)==0);	//姿态角获取	只获取yaw，减少计算量
				}	
				
				if(0 == (time%3))
				{
						encoder_deal();	//编码器值的处理
					
						//5ms释放一次Control信号量，进行一次控制更新
						if(Control_sem->value == 0)
								rt_sem_release(Control_sem);	
						
				}
				
//				if(0 == (time%8))
//				{
//						//8ms释放一次Image信号量，进行一次图像处理
//						//if(Image_sem->value == 0)
//						//	rt_sem_release(Image_sem);
//						
//						//View_vofa(); //vofa上位机看数据
//				}	
				
				
//				if(0 == (time%100))
//				{
//						//100ms释放一次Message信号量，进行一次车模信息发送
//						if(Message_sem->value == 0)
//							rt_sem_release(Message_sem);
//					
//				}	
		}
		
		if(!GoGoGo)	//判断是否发送Image信号量 1时发送
		{
				//8ms释放一次Image信号量，进行一次图像处理
				if(Image_sem->value == 0)
					rt_sem_release(Image_sem);
				
				//View_vofa(); //vofa上位机看数据
		}	
		
//		if(0 == (time%3))
//		{
//				encoder_deal();	//编码器值的处理
//		}
		
}


void timer_pit_init(void)
{
    rt_timer_t timer;
			
		//无线转串口模块相关引脚定义在 wireless.h文件中
	  seekfree_wireless_init();
	
    //创建一个定时器 周期运行 1ms
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    
    //启动定时器
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }
    
}
