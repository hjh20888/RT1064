#include "button.h"


//����״̬����
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

//��һ�ο���״̬����
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

//�����ź���
rt_sem_t key1_sem;
rt_sem_t key2_sem;
rt_sem_t key3_sem;
rt_sem_t key4_sem;
rt_sem_t display_sem; //SWITCH_2 D4 ���Ʒ��� �ر���Ļ

//Message�ź���
rt_sem_t Message_sem;

//display ��ʾҳ��
unsigned int pages = 1;
unsigned int cleardisplay = 0;

//������Ϣ
unsigned char message[18]; 
unsigned char times=0,now_x=0,now_y=0;

//-------------------------------------------------------------------------------------------------------------------
//  brief      ��Ϣ��ʽ������
//  return     void
//  since      v1.0
//  Sample usage:		100ms����һ�� �ѵ�ͼƬ����������һ��
//-------------------------------------------------------------------------------------------------------------------

void message_format_send(void)	//��Ϣ��ʽ������
{
		
		//ʱ��
		message[0] = times/100000%10 + 48;	//ʱ��ʮ��λ
		message[1] = times/10000%10 + 48;		//ʱ����λ
		message[2] = times/1000%10 + 48;		//ʱ��ǧλ
		message[3] = '.';
		message[4] = times/100%10 + 48; //ʱ���λ
		message[5] = times/10%10 + 48; 	//ʱ��ʮλ
		message[6] = times/1%10 + 48; 	//ʱ��λ
		message[7] = ' ';
		
		//������x
		message[8] = now_x/10 + 48;
		message[9] = now_x%10 + 48; 
		message[10] = ' ';
		//������y
		message[11] = now_y/10 + 48;
		message[12] = now_y%10 + 48; 
		message[13] = ' ';
		
		//����
		message[14] = uart4_data[1] + 48;
		message[15] = ' ';
		
		//С��
		message[16] = uart4_data[2] + 48;
		message[17] = '\n';
		
		//��Ϣ���� ʶ�����������һ��
		//uart_putstr(USART_8,message);
		uart_putbuff(USART_8, message, 18);
}	

/*******************************************************************************************************************
*  brief      Message�����߳�
*  return     void
*  since      v1.0
*  Sample usage:	  ��ȡ���ź�����ʼ����	100ms����һ�� �ѵ�ͼƬ����������һ��
*******************************************************************************************************************/

void Message_entry(void *parameter)
{
    while(1)
    {
        //��ȡMessage�ź��������û��������ȴ����ͷ�CPU����Ȩ
        rt_sem_take(Message_sem,RT_WAITING_FOREVER);
				message_format_send();	//��Ϣ��ʽ������
    }
}

void Message_init(void) //Message�����߳�
{
    rt_thread_t tid;
	
    //����Message���ź�����100ms�ж��ͷ�һ��
    Message_sem = rt_sem_create("Message", 0, RT_IPC_FLAG_FIFO);		
    
    //����Message���߳� ���ȼ�19 ʱ��Ƭ1ms
    tid = rt_thread_create("Message", Message_entry, RT_NULL, 256, 19, 1);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}

/*******************************************************************************************************************
*  brief      button��ʱ���߳�
*  return     void
*  since      v1.0
*  Sample usage: 
*******************************************************************************************************************/

void button_entry(void *parameter)
{
    //���水��״̬
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    
    //��ȡ��ǰ����״̬
    key1_status = gpio_get(KEY_1);
    key2_status = gpio_get(KEY_2);
    key3_status = gpio_get(KEY_3);
    key4_status = gpio_get(KEY_4);
    
    //��⵽��������֮�󲢷ſ� �ͷ�һ���ź���
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
		
		//��SWITCH_1 D27 �ر�ʱ���ͷ��ź�������Ļ������ر�
		if(!GoGoGo)
		{
				//���ź���Ϊ��ʱ���ͷ�
				if(display_sem->value == 0)
					rt_sem_release(display_sem);
		}	
		
		//��SWITCH_2 D4 ��ʱ��·���滮
		if(path_plan)		
		{
				image_mode = 2;//����ͷģʽѡ�� 0 �ر�����ͷ		1 ʶ�������		2 ·���滮
		}	
		
}

void button_init(void)
{
    rt_timer_t timer1;
    
	  // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    gpio_init(KEY_1, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);			
	  gpio_init(KEY_2, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
	  gpio_init(KEY_3, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
	  gpio_init(KEY_4, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
		gpio_init(SWITCH_1, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
		gpio_init(SWITCH_2, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
	
    //�����������ź��������������¾��ͷ��ź���������Ҫʹ�ð����ĵط���ȡ�ź�������
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
*  brief      �����߳�
*  return     void
*  since      v1.0
*  Sample usage: 
*******************************************************************************************************************/

void test_entry(void *parameter) //�����߳�
{
	while(1)
	{
		;
	}
}	

void test_init(void) //�����߳�
{
		rt_thread_t tid;
    
    //������ʾ�߳� ���ȼ�����Ϊ20
    tid = rt_thread_create("test", test_entry, RT_NULL, 1024, 31, 3);
    
    //������ʾ�߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }

}	