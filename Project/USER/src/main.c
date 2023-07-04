/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/


//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�


//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//��һ�� �ر��������д򿪵��ļ�
//�ڶ��� project  clean  �ȴ��·�����������

//���ش���ǰ������Լ�ʹ�õ��������ڹ���������������Ϊ�Լ���ʹ�õ�

#include "headfile.h"



rt_sem_t camera_sem;

int main(void)
{
    
    display_init();
	button_init();
    buzzer_init();
    
	All_parameter_Init();//���ֲ����ĳ�ʼ��
	
	Mode_select_init();  //Mode_select�߳�
	Control_init(); 		 //Control�߳�
		
	Imu_init();   			 //Imu�߳�
			
		//gpio_set(BUZZER_PIN, 1);    //�򿪷�����
	Image_init(); 			 //Image�߳�
	
    openart_Init(); 		 //open art��ʼ��
    timer_pit_init();
	//gpio_set(BUZZER_PIN, 0);    //�رշ�����
	  
	//Message_init(); 		 //Message�����߳�
	//test_init();  		 //�����߳�
    
    EnableGlobalIRQ(0);
    while (1)
    {
        rt_thread_mdelay(500);
    }
}

  



