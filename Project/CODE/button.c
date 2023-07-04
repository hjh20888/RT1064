#include "button.h"


//开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

//上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

//开关信号量
rt_sem_t key1_sem;
rt_sem_t key2_sem;
rt_sem_t key3_sem;
rt_sem_t key4_sem;
rt_sem_t display_sem; //SWITCH_2 D4 控制发车 关闭屏幕

//Message信号量
rt_sem_t Message_sem;

//display 显示页数
unsigned int pages = 1;
unsigned int cleardisplay = 0;

//发送信息
unsigned char message[18]; 
unsigned char times=0,now_x=0,now_y=0;

//-------------------------------------------------------------------------------------------------------------------
//  brief      信息格式化发送
//  return     void
//  since      v1.0
//  Sample usage:		100ms发送一次 搜到图片后立即发送一次
//-------------------------------------------------------------------------------------------------------------------

void message_format_send(void)	//信息格式化发送
{
		
		//时间
		message[0] = times/100000%10 + 48;	//时间十万位
		message[1] = times/10000%10 + 48;		//时间万位
		message[2] = times/1000%10 + 48;		//时间千位
		message[3] = '.';
		message[4] = times/100%10 + 48; //时间百位
		message[5] = times/10%10 + 48; 	//时间十位
		message[6] = times/1%10 + 48; 	//时间位
		message[7] = ' ';
		
		//横坐标x
		message[8] = now_x/10 + 48;
		message[9] = now_x%10 + 48; 
		message[10] = ' ';
		//纵坐标y
		message[11] = now_y/10 + 48;
		message[12] = now_y%10 + 48; 
		message[13] = ' ';
		
		//大类
		message[14] = uart4_data[1] + 48;
		message[15] = ' ';
		
		//小类
		message[16] = uart4_data[2] + 48;
		message[17] = '\n';
		
		//信息发送 识别后立即发送一条
		//uart_putstr(USART_8,message);
		uart_putbuff(USART_8, message, 18);
}	

/*******************************************************************************************************************
*  brief      Message发送线程
*  return     void
*  since      v1.0
*  Sample usage:	  获取到信号量后开始运行	100ms运行一次 搜到图片后立即发送一次
*******************************************************************************************************************/

void Message_entry(void *parameter)
{
    while(1)
    {
        //获取Message信号量，如果没有则持续等待并释放CPU控制权
        rt_sem_take(Message_sem,RT_WAITING_FOREVER);
				message_format_send();	//信息格式化发送
    }
}

void Message_init(void) //Message发送线程
{
    rt_thread_t tid;
	
    //创建Message的信号量，100ms中断释放一次
    Message_sem = rt_sem_create("Message", 0, RT_IPC_FLAG_FIFO);		
    
    //创建Message的线程 优先级19 时间片1ms
    tid = rt_thread_create("Message", Message_entry, RT_NULL, 256, 19, 1);
    
    //启动线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}

/*******************************************************************************************************************
*  brief      button定时器线程
*  return     void
*  since      v1.0
*  Sample usage: 
*******************************************************************************************************************/

void button_entry(void *parameter)
{
    //保存按键状态
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    
    //读取当前按键状态
    key1_status = gpio_get(KEY_1);
    key2_status = gpio_get(KEY_2);
    key3_status = gpio_get(KEY_3);
    key4_status = gpio_get(KEY_4);
    
    //检测到按键按下之后并放开 释放一次信号量
    if(key1_status && !key1_last_status)    
    {
				pages++;
				if(pages > 2)
						pages = 1;
				cleardisplay = 1;
				//key_P++;
        rt_sem_release(key1_sem);
        rt_mb_send(buzzer_mailbox, 100);
    }
    if(key2_status && !key2_last_status)    
    {
				pages--;
				if(pages == 0)
						pages = 2;
				cleardisplay = 1;
				//key_P--;
        rt_sem_release(key2_sem);
        rt_mb_send(buzzer_mailbox, 100);
    }
    if(key3_status && !key3_last_status)    
    {
			  //key_I++;
				threshold+=1;
        rt_sem_release(key3_sem);
        rt_mb_send(buzzer_mailbox, 100);
    }
    if(key4_status && !key4_last_status)    
    {
				//key_I--;
				threshold-=1;
        rt_sem_release(key4_sem);
        rt_mb_send(buzzer_mailbox, 100);	
    }
		
		//当SWITCH_1 D27 关闭时，释放信号量打开屏幕，否则关闭
		if(!GoGoGo)
		{
				//当信号量为零时再释放
				if(display_sem->value == 0)
					rt_sem_release(display_sem);
		}	
		
		//当SWITCH_2 D4 打开时，路径规划
		if(path_plan)		
		{
				image_mode = 2;//摄像头模式选择 0 关闭摄像头		1 识别坐标点		2 路径规划
		}	
		
}

void button_init(void)
{
    rt_timer_t timer1;
    
	  // 初始化为GPIO浮空输入 默认上拉高电平
    gpio_init(KEY_1, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);			
	  gpio_init(KEY_2, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
	  gpio_init(KEY_3, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
	  gpio_init(KEY_4, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
		gpio_init(SWITCH_1, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
		gpio_init(SWITCH_2, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
	
    //创建按键的信号量，当按键按下就释放信号量，在需要使用按键的地方获取信号量即可
    key1_sem = rt_sem_create("key1", 0, RT_IPC_FLAG_FIFO);		
    key2_sem = rt_sem_create("key2", 0, RT_IPC_FLAG_FIFO);  
    key3_sem = rt_sem_create("key3", 0, RT_IPC_FLAG_FIFO);  
    key4_sem = rt_sem_create("key4", 0, RT_IPC_FLAG_FIFO);  
		display_sem = rt_sem_create("display", 0, RT_IPC_FLAG_FIFO);
    
    timer1 = rt_timer_create("button", button_entry, RT_NULL, 20, RT_TIMER_FLAG_PERIODIC);

    if(RT_NULL != timer1) 
    {
        rt_timer_start(timer1);
    }
}

/*******************************************************************************************************************
*  brief      测试线程
*  return     void
*  since      v1.0
*  Sample usage: 
*******************************************************************************************************************/

void test_entry(void *parameter) //测试线程
{
	while(1)
	{
		;
	}
}	

void test_init(void) //测试线程
{
		rt_thread_t tid;
    
    //创建显示线程 优先级设置为20
    tid = rt_thread_create("test", test_entry, RT_NULL, 1024, 31, 3);
    
    //启动显示线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }

}	