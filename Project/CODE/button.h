#ifndef _button_h
#define _button_h

#include "headfile.h"

#define KEY_1   C31	// 定义主板上按键对应引脚
#define KEY_2   C27	// 定义主板上按键对应引脚
#define KEY_3   C26	// 定义主板上按键对应引脚
#define KEY_4   C4	// 定义主板上按键对应引脚
#define SWITCH_1   D27
#define SWITCH_2   D4
#define GoGoGo	gpio_get(SWITCH_1)	//控制发车与屏幕开关
#define path_plan	gpio_get(SWITCH_2)	//路径规划开关

//开关信号量
extern rt_sem_t key1_sem;
extern rt_sem_t key2_sem;
extern rt_sem_t key3_sem;
extern rt_sem_t key4_sem;
extern rt_sem_t display_sem; //SWITCH_2 D4 控制发车 关闭屏幕

//Message信号量
extern rt_sem_t Message_sem;

//发送信息
extern unsigned char message[18];
extern unsigned char times,now_x,now_y;

//display
extern unsigned int pages;
extern unsigned int cleardisplay;

void message_format_send(void);	//信息格式化发送

void Message_init(void); //Message发送线程

void button_init(void); //button定时器线程

void test_init(void);   //测试线程

#endif