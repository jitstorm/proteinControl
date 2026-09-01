#include "mixer_pwm.h"
#include "stm32f10x.h"
#include "time.h"

/* DIRC- 的有效电平集中在此处；当前驱动确认高有效，后续仅修改该宏即可切换。 */
#define MIXER_PWM_ACTIVE_LOW 0u
#define MIXER_PWM_PERIOD 3599u

static uint16_t s_mixer_pwm_speed = 0u;
static uint8_t s_mixer_pwm_running = 0u;
static uint32_t s_mixer_pwm_deadline_ms = 0u;

/**
 * 将千分比逻辑速度转换为 TIM3_CH3 的有效 PWM 计数。
 *
 * 使用 ARR+1 作为 100% 的有效周期，确保 speed=1000 时输出连续有效电平。
 *
 * @param speed 已完成上限限制的逻辑速度，单位为千分比。
 * @return 应写入 CCR3 的有效计数值。
 */
static uint16_t MixerPwm_SpeedToCompare(uint16_t speed)
{
    return (uint16_t)(((uint32_t)speed * (MIXER_PWM_PERIOD + 1u)) /
                      MIXER_PWM_SPEED_MAX);
}

/**
 * 根据 DIRC- 的已确认有效电平生成 TIM3_CH3 输出极性。
 *
 * 极性集中在此处，后续硬件确认低有效时无需修改协议或速度换算逻辑。
 *
 * @param 无。
 * @return TIM3_CH3 应使用的输出极性配置。
 */
static uint16_t MixerPwm_GetOutputPolarity(void)
{
    return (MIXER_PWM_ACTIVE_LOW != 0u) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
}

/**
 * 初始化搅拌电机的 TIM3_CH3 PWM 与 ENC 使能输出。
 *
 * PB0 配置为 TIM3_CH3 的 20 kHz PWM，PB1 配置为驱动器 ENC。初始化完成后
 * 必定输出 0% PWM 且 ENC 为低，防止 MCU 上电时搅拌电机自动运行。
 *
 * @param 无。
 * @return 无返回值；硬件输出保持停止状态。
 */
void MixerPwm_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef output_compare;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* PB1 是驱动器 ENC；先预置为低再设为推挽输出，禁止初始化瞬间误使能。 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);

    /* TIM3 默认映射下 CH3 位于 PB0，AF 推挽输出由硬件产生 PWM。 */
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    /* 72 MHz / ((0 + 1) * (3599 + 1)) = 20 kHz。 */
    time_base.TIM_Prescaler = 0u;
    time_base.TIM_Period = MIXER_PWM_PERIOD;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &time_base);

    TIM_OCStructInit(&output_compare);
    output_compare.TIM_OCMode = TIM_OCMode_PWM1;
    output_compare.TIM_OutputState = TIM_OutputState_Enable;
    output_compare.TIM_Pulse = 0u;
    output_compare.TIM_OCPolarity = MixerPwm_GetOutputPolarity();
    TIM_OC3Init(TIM3, &output_compare);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* TIM3 仅承担 CH3 PWM，不启用 Update IRQ 或 TIM3 NVIC。 */
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_Cmd(TIM3, ENABLE);

    MixerPwm_Stop();
}

/**
 * 设置搅拌电机的逻辑速度并同步驱动 PWM 与 ENC。
 *
 * 速度以千分比表示；大于 1000 的值会限制为 1000。speed 为 0 时必须同时
 * 清零 TIM3_CH3 占空比并关闭 PB1 ENC，不能只保留 ENC 高电平。
 *
 * @param speed 目标速度，范围为 0 至 1000，其中 1000 表示 100% 有效占空比。
 * @return 无返回值；调用后已保存并实际应用限制后的逻辑速度。
 */
void MixerPwm_SetSpeed(uint16_t speed)
{
    if (speed > MIXER_PWM_SPEED_MAX)
    {
        speed = MIXER_PWM_SPEED_MAX;
    }

    if (speed == 0u)
    {
        MixerPwm_Stop();
        return;
    }

    /* 先写入 PB0 的目标占空比，再使能 PB1 ENC，避免先使能驱动器后出现旧占空比。 */
    TIM_SetCompare3(TIM3, MixerPwm_SpeedToCompare(speed));
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
    s_mixer_pwm_speed = speed;
}

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
uint8_t MixerPwm_StartTimed(uint16_t speed, uint16_t duration_100ms)
{
    if (speed == 0u)
    {
        MixerPwm_Stop();
        return 1u;
    }

    if (duration_100ms == 0u)
    {
        return 0u;
    }

    if (speed > MIXER_PWM_SPEED_MAX)
    {
        speed = MIXER_PWM_SPEED_MAX;
    }

    /* 合法重发直接覆盖旧任务；时间从本次实际 PWM/ENC 生效时重新计算。 */
    MixerPwm_SetSpeed(speed);
    s_mixer_pwm_deadline_ms = millis() + ((uint32_t)duration_100ms * 100u);
    s_mixer_pwm_running = 1u;
    return 1u;
}

/**
 * 在主循环中推进搅拌 PWM 的非阻塞定时任务。
 *
 * 使用 TIM2 提供的 millis() 毫秒时基判断截止时间，到期后统一调用
 * MixerPwm_Stop()，绝不使用阻塞延时或 TIM3 更新中断。
 *
 * @param 无。
 * @return 无返回值；到期时硬件输出与运行状态均被安全清除。
 */
void MixerPwm_Process(void)
{
    if ((s_mixer_pwm_running != 0u) &&
        ((int32_t)(millis() - s_mixer_pwm_deadline_ms) >= 0))
    {
        /* 截止时间使用有符号差值比较，允许 uint32_t 毫秒计数自然回绕。 */
        MixerPwm_Stop();
    }
}

/**
 * 以安全顺序停止搅拌电机。
 *
 * 先清零 TIM3_CH3 比较值，再将 PB1 ENC 拉低，确保 PWM 与驱动器使能均关闭。
 *
 * @param 无。
 * @return 无返回值；停止后逻辑速度为 0。
 */
void MixerPwm_Stop(void)
{
    TIM_SetCompare3(TIM3, 0u);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    s_mixer_pwm_speed = 0u;
    s_mixer_pwm_running = 0u;
    s_mixer_pwm_deadline_ms = 0u;
}

/**
 * 查询搅拌 PWM 是否仍有未到期的定时任务。
 *
 * @param 无。
 * @return 正在运行时返回 1；已停止、未初始化或已到期时返回 0。
 */
uint8_t MixerPwm_IsRunning(void)
{
    return s_mixer_pwm_running;
}

/**
 * 获取当前已实际应用的搅拌电机逻辑速度。
 *
 * 返回保存的千分比速度而不是 TIM3 CCR3 寄存器值，避免极性配置或定时器周期
 * 变化时让协议层误解速度语义。
 *
 * @param 无。
 * @return 当前逻辑速度，范围为 0 至 1000。
 */
uint16_t MixerPwm_GetSpeed(void)
{
    return s_mixer_pwm_speed;
}

/**
 * 获取当前定时任务剩余运行时间。
 *
 * 返回值以 100 毫秒为单位；若当前毫秒值已越过截止时间或任务未运行，返回 0，
 * 避免无符号减法下溢导致 Android 误判为长时间运行。
 *
 * @param 无。
 * @return 剩余时间，单位为 100 毫秒，范围为 0 至 65535。
 */
uint16_t MixerPwm_GetRemaining100ms(void)
{
    uint32_t now;
    uint32_t remaining_ms;

    now = millis();
    if ((s_mixer_pwm_running == 0u) ||
        ((int32_t)(s_mixer_pwm_deadline_ms - now) <= 0))
    {
        return 0u;
    }

    remaining_ms = s_mixer_pwm_deadline_ms - now;
    /* 向上取整，避免尚有不足 100 ms 时提前向 Android 报告为零。 */
    return (uint16_t)((remaining_ms + 99u) / 100u);
}
