#include "stepMotor.h"
#include "time.h"
#include "shift_register_input.h"
#include "shift_register.h"
#include "protocol.h"
#define MAX_RUN_SPEED 40

GPIO_InitTypeDef GPIO_InitStructure;
// 启用 GPIO 时钟
void Enable_GPIO_Clock(GPIO_TypeDef *GPIOx)
{
  if (GPIOx == GPIOA)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  else if (GPIOx == GPIOB)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  else if (GPIOx == GPIOC)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

// 示例：给 A 电机发送一个脉冲
//  GPIO_SetBits(motorA.GPIOx, motorA.GPIO_Pin);
//  // 延时
//  GPIO_ResetBits(motorA.GPIOx, motorA.GPIO_Pin);
// 初始化某个步进电机的 GPIO 引脚为推挽输出
void StepMotor_Init(stepMotor *motor)
{
  Enable_GPIO_Clock(motor->GPIOx);

  GPIO_InitStructure.GPIO_Pin = motor->GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(motor->GPIOx, &GPIO_InitStructure);

  // 默认输出低
  GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
}

void StepMotor_UpdateSpeed(stepMotor *motor)
{
  // 计算更新速度所需的时间间隔（单位：ms）
  float time_interval = ((float)(motor->update_speed_interval * 10)) / (float)motor->target_speed;
  uint32_t current_millis = millis();
  int threshold;
  int remainingSteps;
  // 如果还没到更新时间，直接返回
  if ((current_millis - motor->time_millis) < time_interval)
  {
    return;
  }

  // 获取当前剩余步数
  remainingSteps = motor->run_steps;

  // 计算下一秒应该执行的脉冲数（你自己实现的逻辑）
  threshold = StepMotor_CalculatePulseThreshold(motor);

  if (threshold > remainingSteps)
  {
    // 需要减速
    motor->run_speed--;
    if (motor->run_speed < 5)
    {
      motor->run_speed = 5;
    }
  }
  else if (motor->run_speed < motor->target_speed)
  {
    // 加速
    motor->run_speed++;
    if (motor->run_speed > motor->target_speed)
    {
      motor->run_speed = motor->target_speed;
    }
  }

  // 更新时间戳
  motor->time_millis = current_millis;
}

void StepMotor_SetDir(stepMotor *motor, char direction)
{
  if (motor->dir_index == 4)
  {
    HC595Data[1] = (HC595Data[1] & ~(1 << 4)) | ((direction & 0x01) << 4);
    ShiftRegister_WriteAll(HC595Data);
  }
  else if (motor->dir_index == 3)
  {
    HC595Data[1] = (HC595Data[1] & ~(1 << 5)) | ((direction & 0x01) << 5);
    ShiftRegister_WriteAll(HC595Data);
  }
}
void StepMotor_run(stepMotor *motor)
{
  // if (motor->run_steps)
  // {
  //   // 翻转输出状态
  //   motor->pulse_state ^= 1;

  //   if (motor->pulse_state)
  //   {
  //     GPIO_SetBits(motor->GPIOx, motor->GPIO_Pin);
  //   }
  //   else
  //   {
  //     GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
  //   }
  //   // 每个完整脉冲周期后减少一步
  //   if (motor->pulse_state == 0) // 下降沿时减少步数（可改为上升沿）
  //   {
  //     motor->run_steps--;
  //     StepMotor_UpdateSpeed(motor);
  //   }
  //   if (motor->run_steps == 0)
  //   {
  //     motor->run_speed = 1;
  //   }
  // }
  if (motor->event_flags & EVT_STEPS_ZERO)
  {
    motor->event_flags &= ~EVT_STEPS_ZERO;

    // 停止后的默认处理
    StepMotor_SetDir(motor, 0); // 停车时方向归0（按需）
    motor->run_speed = 1;       // 速度复位（按你原逻辑）
                                // 这里可以按需要发完成事件/回报上层
  }
}

void StepMotor_runWithSensor(stepMotor *motor, char sensorState)
{
  // if (sensorState)
  // {
  //   motor->run_steps = 0;
  //   GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
  //   motor->run_speed = 1;
  //   motor->hc165_index = 0;
  //   motor->hc165_value = 0;
  // }

  // if (motor->run_steps)
  // {
  //   // 翻转输出状态
  //   motor->pulse_state ^= 1;

  //   if (motor->pulse_state)
  //   {
  //     GPIO_SetBits(motor->GPIOx, motor->GPIO_Pin);
  //   }
  //   else
  //   {
  //     GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
  //   }
  //   // 每个完整脉冲周期后减少一步
  //   if (motor->pulse_state == 0) // 下降沿时减少步数（可改为上升沿）
  //   {
  //     motor->run_steps--;
  //     StepMotor_UpdateSpeed(motor);
  //   }
  //   if (motor->run_steps == 0)
  //   {
  //     motor->run_speed = 1;
  //   }
  // }
  if (sensorState && !motor->afterBySensorWroking)
  {
    motor->run_steps = 50; // 让 TIM3 很快置 EVT_STEPS_ZERO
    motor->afterBySensorWroking = 1;
  }
  // 收尾：这一步走完了
  if (motor->event_flags & EVT_STEPS_ZERO)
  {
    motor->event_flags &= ~EVT_STEPS_ZERO;
    StepMotor_SetDir(motor, 0);
    motor->run_speed = 1;
    // 这里也可以通知上层“到位/停下”
  }
}

void StepMotor_step_pulse_ISR(stepMotor *m)
{
  if (m->run_steps == 0)
    return;

  m->pulse_state ^= 1;
  if (m->pulse_state)
    GPIO_SetBits(m->GPIOx, m->GPIO_Pin);
  else
    GPIO_ResetBits(m->GPIOx, m->GPIO_Pin);

  // 在下降沿计步：与你原函数一致（pulse_state==0时减计数）
  if (m->pulse_state == 0)
  {
    if (m->run_steps)
      m->run_steps--;
    StepMotor_UpdateSpeed(m); // ← 你的加减速算法
    m->event_flags |= EVT_STEP_TAKEN;
    if (m->run_steps == 0)
    {
      m->event_flags |= EVT_STEPS_ZERO; // 通知慢任务处理收尾/换向
      m->run_speed = 1;                 // 与你原逻辑一致
    }
  }
}

void StepMotor_run_check_sm(stepMotor *motor, uint8_t isTriggerSensor, uint8_t direction)
{
  // 0) 统一在最前面消费“步数归零事件”
  if (motor->event_flags & EVT_STEPS_ZERO)
  {
    motor->event_flags &= ~EVT_STEPS_ZERO;

    StepMotor_SetDir(motor, 0);
    motor->run_speed = 1;

    // 原来你在“统一收尾”里做的 run_state++ / 记时间，挪到这里
    if (direction == 2 && motor->_currentType == 2 && motor->run_state != 4)
    {
      motor->run_state++;
      if (motor->run_state == 3)
      {
        motor->pause_delay_time = millis();
      }
    }
    // 注意：这里不要 return；后面还要看是否需要立刻启动左跑
  }

  // 1) 非此类流程，直接退出（但不要再限定 run_state）
  if (!(direction == 2 && motor->_currentType == 2))
  {
    return;
  }

  // 2) 右跑触发：state==1 && 传感器=1 && 现在没在跑
  if (isTriggerSensor && (motor->run_steps == 0) && motor->run_state == 1)
  {
    StepMotor_SetDir(motor, 1);
    motor->run_steps = 2000;
    motor->run_state = 2;
    return;
  }

  // 3) 若当前不触发传感器，且已经停下（run_steps==0），则启动左跑
  if (!isTriggerSensor && (motor->run_state <= 3) && (motor->run_steps == 0))
  {
    StepMotor_SetDir(motor, 0);
    motor->run_steps = 0xFFFF;
    motor->run_state = 3;
    return;
  }

  // 4) 左跑过程中若再次触发传感器，则立即停到 state=4
  if (motor->run_steps > 0 && isTriggerSensor && motor->run_state == 3)
  {
    motor->run_steps = 0;
    motor->run_state = 4;
    motor->run_speed = 1;
  }
}

void StepMotor_run_check(stepMotor *motor, char isTriggerSensor, char direction)
{
  if (direction == 2 && (motor->run_state == 1 || motor->run_state == 3) &&
      motor->_currentType == 2)
  {

    if (isTriggerSensor && (motor->run_steps == 0) && motor->run_state == 1)
    { // 已经触发了传感器 需要反转方向跑一下
      StepMotor_SetDir(motor, 1);
      // setCurrentStep(2000);
      motor->run_steps = 2000;
      // runSteps = 0;
      motor->run_state = 2; // 2右跑，继续
    }
    else if (!isTriggerSensor && motor->run_state <= 3 && (motor->run_steps != 0) && millis() - motor->pause_delay_time > 100)
    {
      motor->run_steps = 0;
    }
    else if (!isTriggerSensor && motor->run_state <= 3 && (motor->run_steps == 0) && millis() - motor->pause_delay_time > 100)
    {
      StepMotor_SetDir(motor, 0);
      motor->run_steps = 0xffff;
      // runSteps = 0;
      motor->run_state = 3; // 3左跑，结束。
    }
  }
  if (motor->run_steps > 0 && direction == 2 && motor->_currentType == 2)
  {
    // 翻转输出状态
    motor->pulse_state ^= 1;

    if (motor->pulse_state)
    {
      GPIO_SetBits(motor->GPIOx, motor->GPIO_Pin);
    }
    else
    {
      GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
    }
    if (motor->pulse_state == 0)
    {

      motor->run_steps--;
      StepMotor_UpdateSpeed(motor);
      if (isTriggerSensor && direction == 2 && motor->run_state == 3)
      {
        GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
        motor->run_steps = 0;
        motor->run_state = 4; // 4停止
        motor->run_speed = 1;
      }
    }
    if (motor->run_steps == 0)
    {
      StepMotor_SetDir(motor, 0);
      if (direction == 2 && motor->run_state != 4)
      {
        motor->run_state++;
        if (motor->run_state == 3)
        {
          motor->pause_delay_time = millis();
        }
      }
      if (direction == 2 && motor->run_state == 4)
      {
        // sendCommandPacket(_cmd, 1);//不需要发送
      }
      motor->run_speed = 1;
    }
  }
}

void StepMotor_run_check1(stepMotor *motor, char isTriggerSensor, char direction)
{

  if (direction == 1 && (motor->run_state == 1 || motor->run_state == 3) &&
      motor->_currentType == 1)
  {
    if (isTriggerSensor && (motor->run_steps == 0) && motor->run_state == 1)
    { // 已经触发了传感器 需要反转方向跑一下
      StepMotor_SetDir(motor, 0);
      // setCurrentStep(2000);
      motor->run_steps = 2000;
      // runSteps = 0;
      motor->run_state = 2; // 2右跑，继续
    }
    else if (!isTriggerSensor && motor->run_state <= 3 && (motor->run_steps != 0) && millis() - motor->pause_delay_time > 1000)
    {
      motor->run_steps = 0;
    }
    else if (!isTriggerSensor && motor->run_state <= 3 && (motor->run_steps == 0) && millis() - motor->pause_delay_time > 1000)
    {
      StepMotor_SetDir(motor, 1);
      motor->run_steps = 0xffff;
      motor->run_state = 3; // 3左跑，结束。
    }
  }
  if (motor->run_steps > 0 && direction == 1 && motor->_currentType == 1)
  {
    // 翻转输出状态
    motor->pulse_state ^= 1;

    if (motor->pulse_state)
    {
      GPIO_SetBits(motor->GPIOx, motor->GPIO_Pin);
    }
    else
    {
      GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
    }
    if (motor->pulse_state == 0)
    {
      // updateMotorSpeed();
      // setCurrentStep(motor->run_steps - 1);
      // runSteps++;
      motor->run_steps--;
      StepMotor_UpdateSpeed(motor);
      if (isTriggerSensor && direction == 1 && motor->run_state == 3)
      {
        GPIO_ResetBits(motor->GPIOx, motor->GPIO_Pin);
        motor->run_steps = 0;
        // runSteps = 0;
        motor->run_state = 4; // 4停止
        // sendCommandPacket(_cmd, 1);
        motor->run_speed = 1;
      }
    }
    if (motor->run_steps == 0)
    {
      StepMotor_SetDir(motor, 0);
      if (direction == 1 && motor->run_state != 4)
      {
        motor->run_state++;
        if (motor->run_state == 3)
        {
          motor->pause_delay_time = millis();
        }
      }
      if (direction == 1 && motor->run_state == 4)
      {
        // sendCommandPacket(_cmd, 1);//不需要发送
      }
      motor->run_speed = 1;
    }
  }
}
int StepMotor_CalculatePulseThreshold(stepMotor *motor)
{
  return 50 * motor->run_speed;
}
