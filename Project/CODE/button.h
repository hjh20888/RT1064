#ifndef _button_h
#define _button_h

#include "headfile.h"

#define KEY_1   C31	// ���������ϰ�����Ӧ����
#define KEY_2   C27	// ���������ϰ�����Ӧ����
#define KEY_3   C26	// ���������ϰ�����Ӧ����
#define KEY_4   C4	// ���������ϰ�����Ӧ����
#define SWITCH_1   D27
#define SWITCH_2   D4
#define GoGoGo	gpio_get(SWITCH_1)	//���Ʒ�������Ļ����
#define path_plan	gpio_get(SWITCH_2)	//·���滮����

//�����ź���
extern rt_sem_t key1_sem;
extern rt_sem_t key2_sem;
extern rt_sem_t key3_sem;
extern rt_sem_t key4_sem;
extern rt_sem_t display_sem; //SWITCH_2 D4 ���Ʒ��� �ر���Ļ

//Message�ź���
extern rt_sem_t Message_sem;

//������Ϣ
extern unsigned char message[18];
extern unsigned char times,now_x,now_y;

//display
extern unsigned int pages;
extern unsigned int cleardisplay;

void message_format_send(void);	//��Ϣ��ʽ������

void Message_init(void); //Message�����߳�

void button_init(void); //button��ʱ���߳�

void test_init(void);   //�����߳�

#endif