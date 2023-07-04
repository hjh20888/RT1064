/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/


//整套推荐IO查看Projecct文件夹下的TXT文本


//打开新的工程或者工程移动了位置务必执行以下操作
//第一步 关闭上面所有打开的文件
//第二步 project  clean  等待下方进度条走完

//下载代码前请根据自己使用的下载器在工程里设置下载器为自己所使用的

#include "headfile.h"



rt_sem_t camera_sem;

int main(void)
{
    
    display_init();
	button_init();
    buzzer_init();
    
	All_parameter_Init();//各种参数的初始化
	
	Mode_select_init();  //Mode_select线程
	Control_init(); 		 //Control线程
		
	Imu_init();   			 //Imu线程
			
		//gpio_set(BUZZER_PIN, 1);    //打开蜂鸣器
	Image_init(); 			 //Image线程
	
    openart_Init(); 		 //open art初始化
    timer_pit_init();
	//gpio_set(BUZZER_PIN, 0);    //关闭蜂鸣器
	  
	//Message_init(); 		 //Message发送线程
	//test_init();  		 //测试线程
    
    EnableGlobalIRQ(0);
    while (1)
    {
        rt_thread_mdelay(500);
    }
}

  



