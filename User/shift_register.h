#ifndef __SHIFT_REGISTER_H__
#define __SHIFT_REGISTER_H__

#include "stm32f10x.h"

// struct shift_register
// {
//   /* data */
//   char index;//下标
//   char enable;//使能
//   int time;//时间
// };
#define MOTOR_NUM 19 // 排除未使用的 MT1~MT5 后，实际可用 19 路单向电机输出
typedef struct {
  uint8_t enabled;     // 当前使能状态
  uint16_t on_time_ms; // 需要保持开启的时间（ms）
  uint16_t timer_ms;   // 计时器，用于递减
} MotorCtrl_t;



#define SHIFT_REGISTER_COUNT  4  // 4 个 74HC595
extern uint8_t HC595Data[SHIFT_REGISTER_COUNT]; // 示例数据
void ShiftRegister_Init(void);
void ShiftRegister_Write(uint8_t data, uint8_t registerIndex);
void ShiftRegister_WriteAll(uint8_t *data);
#endif // __SHIFT_REGISTER_H__
