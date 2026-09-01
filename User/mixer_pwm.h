#ifndef __MIXER_PWM_H
#define __MIXER_PWM_H

#include <stdint.h>

/** 搅拌电机逻辑速度允许的最大千分比。 */
#define MIXER_PWM_SPEED_MAX 1000u

/**
 * 初始化搅拌电机的 TIM3_CH3 PWM 与 ENC 使能输出。
 *
 * PB0 配置为 TIM3_CH3 的 20 kHz PWM，PB1 配置为驱动器 ENC。初始化完成后
 * 必定输出 0% PWM 且 ENC 为低，防止 MCU 上电时搅拌电机自动运行。
 *
 * @param 无。
 * @return 无返回值；硬件输出保持停止状态。
 */
void MixerPwm_Init(void);

/**
 * 设置搅拌电机的逻辑速度并同步驱动 PWM 与 ENC。
 *
 * 速度以千分比表示；大于 1000 的值会限制为 1000。speed 为 0 时必须同时
 * 清零 TIM3_CH3 占空比并关闭 PB1 ENC，不能只保留 ENC 高电平。
 *
 * @param speed 目标速度，范围为 0 至 1000，其中 1000 表示 100% 有效占空比。
 * @return 无返回值；调用后已保存并实际应用限制后的逻辑速度。
 */
void MixerPwm_SetSpeed(uint16_t speed);

/**
 * 启动或覆盖一项由 MCU 自行计时的搅拌 PWM 任务。
 *
 * speed 为 0 时立即进入安全停止；speed 大于 0 时 duration_100ms 必须大于 0，
 * 否则拒绝本次请求且不改变正在执行的任务。合法的新请求会覆盖旧速度和截止时间。
 *
 * @param speed 目标速度，范围为 0 至 1000，超过上限会限制为 1000。
 * @param duration_100ms 运行时长，单位为 100 毫秒，必须大于 0。
 * @return 请求被接受时返回 1；speed 大于 0 且时长为 0 时返回 0。
 */
uint8_t MixerPwm_StartTimed(uint16_t speed, uint16_t duration_100ms);

/**
 * 在主循环中推进搅拌 PWM 的非阻塞定时任务。
 *
 * 使用 TIM2 提供的 millis() 毫秒时基判断截止时间，到期后统一调用
 * MixerPwm_Stop()，绝不使用阻塞延时或 TIM3 更新中断。
 *
 * @param 无。
 * @return 无返回值；到期时硬件输出与运行状态均被安全清除。
 */
void MixerPwm_Process(void);

/**
 * 以安全顺序停止搅拌电机。
 *
 * 先清零 TIM3_CH3 比较值，再将 PB1 ENC 拉低，确保 PWM 与驱动器使能均关闭。
 *
 * @param 无。
 * @return 无返回值；停止后逻辑速度为 0。
 */
void MixerPwm_Stop(void);

/**
 * 查询搅拌 PWM 是否仍有未到期的定时任务。
 *
 * @param 无。
 * @return 正在运行时返回 1；已停止、未初始化或已到期时返回 0。
 */
uint8_t MixerPwm_IsRunning(void);

/**
 * 获取当前已实际应用的搅拌电机逻辑速度。
 *
 * 返回保存的千分比速度而不是 TIM3 CCR3 寄存器值，避免极性配置或定时器周期
 * 变化时让协议层误解速度语义。
 *
 * @param 无。
 * @return 当前逻辑速度，范围为 0 至 1000。
 */
uint16_t MixerPwm_GetSpeed(void);

/**
 * 获取当前定时任务剩余运行时间。
 *
 * 返回值以 100 毫秒为单位；若当前毫秒值已越过截止时间或任务未运行，返回 0，
 * 避免无符号减法下溢导致 Android 误判为长时间运行。
 *
 * @param 无。
 * @return 剩余时间，单位为 100 毫秒，范围为 0 至 65535。
 */
uint16_t MixerPwm_GetRemaining100ms(void);

#endif
