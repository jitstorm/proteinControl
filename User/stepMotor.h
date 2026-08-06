#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include "stm32f10x.h"
typedef struct
{
  GPIO_TypeDef *GPIOx;  // GPIO 端口
  uint16_t GPIO_Pin;    // GPIO 引脚
  uint16_t timer_count; // 定时器计算
  uint8_t step_state;    // 进入跑一个脉冲
  uint8_t run_speed;    // 运行速度
  uint8_t target_speed; // 目标速度


  uint16_t target_steps; // 目标脉冲数
  uint16_t run_steps;    // 需要跑脉冲数
  uint16_t run_state;    // 运行状态 1:进来，2右跑，3左跑，4停止
  uint8_t pulse_state;
  uint8_t update_speed_interval ;  //更新速度间隔
  unsigned long time_millis;
  uint8_t direction;
  uint8_t cmd;//区分是否要识别传感器
  uint8_t _currentType;
  unsigned long pause_delay_time; // 避免z轴旋转过快
  uint8_t dir_index; // 方向index
  uint8_t hc165_index; // 165index
  char hc165_value; // 165value
  int reback_steps;//返回步数
      volatile uint8_t  event_flags;   // ISR->慢任务 事件通知

  uint8_t afterBySensorWroking;// 用于传感器触发之后是否已经再走几步的状态判断
} stepMotor;
// 事件位
#define EVT_STEPS_ZERO   (1u<<0)   // run_steps 归零事件（由ISR置位）
#define EVT_STEP_TAKEN   (1u<<1)   // 走了一步（可选）
void Enable_GPIO_Clock(GPIO_TypeDef *GPIOx);
void StepMotor_Init(stepMotor *motor);
void StepMotor_run(stepMotor *motor);
int StepMotor_CalculatePulseThreshold(stepMotor *motor);
void StepMotor_run_check(stepMotor *motor, char isTriggerSensor, char direction);
void StepMotor_run_check1(stepMotor *motor, char isTriggerSensor, char direction);
void StepMotor_runWithSensor(stepMotor *motor, char sensorState);
void StepMotor_run_check_sm(stepMotor *motor, uint8_t isTriggerSensor, uint8_t direction);
void StepMotor_step_pulse_ISR(stepMotor *m);
#endif
