#ifndef __STEP_DMA_H__
#define __STEP_DMA_H__

#include "stm32f10x.h"
#include <stdint.h>

/*
 * 初始化：TIM5 + DMA2_Channel2，PB11 推挽输出
 * tim5_clk_hz：TIM5 的计数时钟（通常 72MHz）
 */
void stepdma_pb11_init(uint32_t tim5_clk_hz);

/*
 * 固定速度跑 steps 步（steps/s），跑完自动停
 * steps: 步数（步进驱动器看到的 STEP 上升沿次数）
 * fstep_hz: steps/s
 */
void stepdma_pb11_move_steps(uint32_t steps, uint32_t fstep_hz);

/*
 * 梯形/三角形速度曲线跑 steps 步（末端减到 0，更柔）
 * f_start: 起步速度 steps/s（建议 >= 1）
 * f_max: 速度上限 steps/s
 * accel: 加速度 steps/s^2
 *
 * 注意：末端会逐渐减到非常低，然后我们“跑完最后一步就关定时器”，实现 0 速度停止。
 */
void stepdma_pb11_move_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel);
void stepdma_pb11_request_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel);

/* 立即停止（关 TIM5 + DMA，PB11 拉低） */
void stepdma_pb11_stop(void);

/* 是否正在运行（1=跑步中，0=已停止） */
uint8_t stepdma_pb11_is_running(void);
/* PB10 : TIM6 + DMA2_Channel3 */
void stepdma_pb10_init(uint32_t tim6_clk_hz);
void stepdma_pb10_stop(void);
uint8_t stepdma_pb10_is_running(void);
void stepdma_pb10_move_steps(uint32_t steps, uint32_t fstep_hz);
void stepdma_pb10_move_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel);
void stepdma_pb10_request_trap(uint32_t steps, uint32_t f_start, uint32_t f_max, uint32_t accel);

/** 初始化第二轴：PB10 STEP、TIM6 和 DMA2 通道3。 */
void Stepper2_Init(void);
/** 设置第二轴方向；运行中请求会被忽略，避免脉冲期间换向。 */
void Stepper2_SetDirection(uint8_t direction);
/** 启动第二轴梯形加减速运动，运行中重复启动返回 0。 */
uint8_t Stepper2_Start(uint8_t direction, uint32_t steps, uint32_t target_frequency);
/** 立即停止第二轴，并将 PB10 保持为低电平。 */
void Stepper2_Stop(void);
/** 查询第二轴是否正在运行。 */
uint8_t Stepper2_IsBusy(void);
/** 获取第二轴已完成的上升沿步数。 */
uint32_t Stepper2_GetCompletedSteps(void);
/** 获取第二轴尚未完成的步数。 */
uint32_t Stepper2_GetRemainingSteps(void);

/* PU3 资源集中定义，TIM7 更新事件触发 DMA 写 GPIOB->BSRR。 */
#define PU3_STEP_GPIO GPIOB
#define PU3_STEP_PIN GPIO_Pin_13
#define PU3_TIMER TIM7
#define PU3_DMA_CHANNEL DMA2_Channel4
#define PU3_DMA_IRQn DMA2_Channel4_5_IRQn

/** 初始化 PU3 的 PB13、TIM7 和 DMA2 通道4。 */
void PU3_Stepper_Init(void);
/** 启动 PU3 梯形加减速运动；运行中或零步数时返回 0。 */
uint8_t PU3_Stepper_Start(uint32_t steps, uint8_t direction,
                          uint32_t start_frequency,
                          uint32_t maximum_frequency,
                          uint32_t acceleration);
/** 停止 PU3，并将 PB13 保持为低电平。 */
void PU3_Stepper_Stop(void);
/** 查询 PU3 是否正在输出 DMA 步进脉冲。 */
uint8_t PU3_Stepper_IsRunning(void);

#endif
