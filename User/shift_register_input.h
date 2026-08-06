#ifndef __SHIFT_REGISTER_INPUT_H__
#define __SHIFT_REGISTER_INPUT_H__

#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include <DELAY.h>
#define SHIFT_REGISTER_INPUT_COUNT 2  // 硬件仅串联 2 个 74HC165
#define SHIFT_REGISTER_INPUT_CHANNEL_COUNT (SHIFT_REGISTER_INPUT_COUNT * 8u) // 每片提供 8 路输入
extern uint8_t inputData[SHIFT_REGISTER_INPUT_COUNT];

void ShiftRegisterInput_Init(void);
void ShiftRegisterInput_ReadAll(uint8_t *data);
uint8_t ShiftRegisterInput_IsBusy(void);
uint8_t ShiftRegisterInput_HasNewData(void);
void ShiftRegisterInput_CopyLatest(uint8_t *data);
#endif // __SHIFT_REGISTER_INPUT_H__
