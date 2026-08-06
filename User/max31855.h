#ifndef __MAX31855_H
#define __MAX31855_H

#include "stm32f10x.h"
extern int temperature;  // 外部引用温度变量
void MAX31855_Init(void);
float MAX31855_GetTemperature(void);

#endif
