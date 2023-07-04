#ifndef _buzzer_h
#define _buzzer_h

#include "headfile.h"

#define BUZZER_PIN			B11			// 定义主板上蜂鸣器对应引脚

extern rt_mailbox_t buzzer_mailbox;

void buzzer_init(void);

#endif