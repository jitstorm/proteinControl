#ifndef __DELAY_H
#define __DELAY_H
#include <stdint.h>  // 包含uint32_t的定义
#include "stm32f10x.h"
void Delay(uint32_t nCount);
void delay_us(uint16_t time);
void delay_ms(uint16_t time);
void Delay_Init(void);
void Delay_us(uint32_t nus);
void Delay_ms(uint16_t nms);
#endif
