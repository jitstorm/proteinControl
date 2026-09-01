#ifndef __TIME_H
#define __TIME_H

#include "stm32f10x.h"
void TIM1_Init_100us(void);
void Timer2_Init(void);
void Motor_RunTime_Update(void);  // 如果该函数跟时间紧密相关，也可声明在这里
void TIM4_10us_Init(void);
uint32_t millis(void);
#endif
