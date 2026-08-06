#include "pwm.h"

volatile uint8_t pwm_pc14_duty = 0;
volatile uint8_t pwm_pc14_enable = 0;
volatile uint8_t pwm_pc15_duty = 0;
volatile uint8_t pwm_pc15_enable = 0;
volatile uint8_t pwm_pc1_enable = 0;
volatile uint8_t pwm_pc2_enable = 0;
static volatile uint8_t pwm_pb0_enable = 0;
static volatile uint8_t pwm_pa0_enable = 0;
static volatile uint8_t pwm_pa1_enable = 0;
static volatile uint8_t pwm_pa11_enable = 0;

static uint8_t PWM_ClampPercent(uint8_t percent)
{
  if (percent > 100)
  {
    percent = 100;
  }
  return percent;
}

void PWM_Init(const PWM_Channel *pwm, uint32_t period, uint32_t prescaler)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 禁用 JTAG，释�?PB3~PB5
  // 开启重映射（只处理 TIM1�?
  if (pwm->remap == ENABLE)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 所�?remap 都需�?AFIO 时钟

    if (pwm->TIMx == TIM1)
    {
      // 保险起见�?USB 禁用操作（针�?TIM1_CH4 = PA11 的情况）
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
      NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

      // 重映�?TIM1（视你使用的是部分还是完全映射）
      // GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); // 你可以按实际�?
    }
    else if (pwm->TIMx == TIM2)
    {
      // TIM2 �?PB3 是完全重映射才支�?CH2
      GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); // TIM2_CH2 �?PB3
    }
    else if (pwm->TIMx == TIM3)
    { 
      // TIM3 �?PB4 是部分重映射支持 CH1
      GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3_CH1 �?PB4
    }
  }

  if ((uint32_t)pwm->TIMx >= APB2PERIPH_BASE)
  {
    RCC_APB2PeriphClockCmd(pwm->TIM_RCC, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(pwm->TIM_RCC, ENABLE);
  }

  // 配置 GPIO
  GPIO_InitStructure.GPIO_Pin = pwm->GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(pwm->GPIOx, &GPIO_InitStructure);

  // 定时器时基配�?
  TIM_TimeBaseStructure.TIM_Period = period - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(pwm->TIMx, &TIM_TimeBaseStructure);

  // PWM 输出模式配置
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  switch (pwm->channel)
  {
  case 1:
    TIM_OC1Init(pwm->TIMx, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(pwm->TIMx, TIM_OCPreload_Enable);
    break;
  case 2:
    TIM_OC2Init(pwm->TIMx, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(pwm->TIMx, TIM_OCPreload_Enable);
    break;
  case 3:
    TIM_OC3Init(pwm->TIMx, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(pwm->TIMx, TIM_OCPreload_Enable);
    break;
  case 4:
    TIM_OC4Init(pwm->TIMx, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(pwm->TIMx, TIM_OCPreload_Enable);
    break;
  }

  TIM_ARRPreloadConfig(pwm->TIMx, ENABLE);
  TIM_Cmd(pwm->TIMx, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE); // 🔥 高级定时器必须加�?
}

static uint16_t PWM_PercentToCCR(TIM_TypeDef *TIMx, uint8_t percent)
{
  uint32_t arr;
  uint32_t ccr;

  percent = PWM_ClampPercent(percent);

  arr = TIMx->ARR;
  if (percent >= 100)
  {
    return (arr < 0xFFFF) ? (uint16_t)(arr + 1) : 0xFFFF;
  }

  ccr = ((arr + 1) * percent) / 100;

  return (uint16_t)ccr;
}

static void PWM_ConfigPAExtraPin(uint16_t pin, GPIOMode_TypeDef mode)
{
  GPIO_InitTypeDef gpio;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  gpio.GPIO_Pin = pin;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = mode;
  GPIO_Init(GPIOA, &gpio);
}

static void PWM_SetPAExtraIdle(uint16_t pin)
{
  PWM_ConfigPAExtraPin(pin, GPIO_Mode_Out_PP);
  /* The terminal side is inverted by the driver stage, so PA high = terminal 0V. */
  GPIO_SetBits(GPIOA, pin);
}

void PWM_PA0_PA1_PA11_Init(void)
{
  TIM_OCInitTypeDef oc;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  /* Keep TIM2_CH1/TIM2_CH2 on PA0/PA1. */
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);
  GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, DISABLE);
  GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, DISABLE);

  PWM_SetPAExtraIdle(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11);

  TIM_OCStructInit(&oc);
  oc.TIM_OCMode = TIM_OCMode_PWM1;
  oc.TIM_OutputState = TIM_OutputState_Enable;
  oc.TIM_Pulse = 0;
  /* PA0/PA1/PA11 terminal outputs are inverted by the driver stage. */
  oc.TIM_OCPolarity = TIM_OCPolarity_Low;

  /* TIM2 is already the 1ms time base; only attach CH1/CH2 PWM outputs. */
  TIM_OC1Init(TIM2, &oc);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM2, &oc);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* TIM1 is already used by PB0 CH2N; CH4 reuses the same period. */
  TIM_OC4Init(TIM1, &oc);
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_SetCompare1(TIM2, 0);
  TIM_SetCompare2(TIM2, 0);
  TIM_SetCompare4(TIM1, 0);
  TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
  TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
  TIM_Cmd(TIM2, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_SetPA0_Duty(uint8_t percent)
{
  percent = PWM_ClampPercent(percent);
  pwm_pa0_enable = (percent > 0) ? 1 : 0;
  if (percent == 0)
  {
    TIM_SetCompare1(TIM2, 0);
    PWM_SetPAExtraIdle(GPIO_Pin_0);
    return;
  }

  PWM_ConfigPAExtraPin(GPIO_Pin_0, GPIO_Mode_AF_PP);
  TIM_SetCompare1(TIM2, PWM_PercentToCCR(TIM2, percent));
}

void PWM_SetPA1_Duty(uint8_t percent)
{
  percent = PWM_ClampPercent(percent);
  pwm_pa1_enable = (percent > 0) ? 1 : 0;
  if (percent == 0)
  {
    TIM_SetCompare2(TIM2, 0);
    PWM_SetPAExtraIdle(GPIO_Pin_1);
    return;
  }

  PWM_ConfigPAExtraPin(GPIO_Pin_1, GPIO_Mode_AF_PP);
  TIM_SetCompare2(TIM2, PWM_PercentToCCR(TIM2, percent));
}

void PWM_SetPA11_Duty(uint8_t percent)
{
  percent = PWM_ClampPercent(percent);
  pwm_pa11_enable = (percent > 0) ? 1 : 0;
  if (percent == 0)
  {
    TIM_SetCompare4(TIM1, 0);
    PWM_SetPAExtraIdle(GPIO_Pin_11);
    return;
  }

  PWM_ConfigPAExtraPin(GPIO_Pin_11, GPIO_Mode_AF_PP);
  TIM_SetCompare4(TIM1, PWM_PercentToCCR(TIM1, percent));
}

void PWM_SetPB0_Duty(uint8_t percent)
{
  /* PB0 已作为正反转电机 5 的 IN2，旧 PWM 入口禁止再驱动该引脚。 */
  (void)percent;
  pwm_pb0_enable = 0;
}

/**
 * 获取 PWM 电机运行状态位图。
 *
 * bit0=PB0，bit1=PC14，bit2=PC15，bit3=PC1，
 * bit4=PC2，bit5=PA0，bit6=PA1，bit7=PA11。
 */
uint8_t PWM_GetMotorRunStatus(void)
{
  uint8_t status = 0;

  /* 使用状态缓存表达控制意图，避免直接读取复用输出寄存器造成误判。 */
  if (pwm_pb0_enable)
  {
    status |= (1 << 0);
  }
  if (pwm_pc14_enable)
  {
    status |= (1 << 1);
  }
  if (pwm_pc15_enable)
  {
    status |= (1 << 2);
  }
  if (pwm_pc1_enable)
  {
    status |= (1 << 3);
  }
  if (pwm_pc2_enable)
  {
    status |= (1 << 4);
  }
  if (pwm_pa0_enable)
  {
    status |= (1 << 5);
  }
  if (pwm_pa1_enable)
  {
    status |= (1 << 6);
  }
  if (pwm_pa11_enable)
  {
    status |= (1 << 7);
  }

  return status;
}

static void PWM_ConfigPC14PC15Pin(uint16_t pin)
{
  /* 相关引脚已由正反转电机模块独占，禁止旧配置函数修改模式。 */
  (void)pin;
}

static void PWM_UpdatePC14PC15Enable(void)
{
  /* PC0 已作为正反转电机 3 的 EN，禁止旧逻辑修改输出。 */
}

void PWM_SetPC14_Duty(uint8_t percent)
{
  /* PC14 已作为正反转电机 3 的 IN1，旧 PWM 入口禁止再驱动该引脚。 */
  (void)percent;
  pwm_pc14_duty = 0;
  pwm_pc14_enable = 0;
}

void PWM_SetPC15_Duty(uint8_t percent)
{
  percent = PWM_ClampPercent(percent);
  PWM_SetPC15_Motor((percent > 0) ? 1 : 0);
}

void PWM_SetPC15_Motor(uint8_t enable)
{
  /* PC15 已作为正反转电机 3 的 IN2，旧普通 GPIO 入口禁止再驱动该引脚。 */
  (void)enable;
  pwm_pc15_duty = 0;
  pwm_pc15_enable = 0;
}

static void PWM_UpdatePC1PC2Enable(void)
{
  /* PC3 已作为正反转电机 4 的 EN，禁止旧逻辑修改输出。 */
}

void PWM_SetPC1_Motor(uint8_t enable)
{
  /* PC1 已作为正反转电机 4 的 IN1，旧普通 GPIO 入口禁止再驱动该引脚。 */
  (void)enable;
  pwm_pc1_enable = 0;
}

void PWM_SetPC2_Motor(uint8_t enable)
{
  /* PC2 已作为正反转电机 4 的 IN2，旧普通 GPIO 入口禁止再驱动该引脚。 */
  (void)enable;
  pwm_pc2_enable = 0;
}

void PWM_SetExtraDuty(uint8_t channel, uint8_t percent)
{
  switch (channel)
  {
  case 0:
    PWM_SetPA0_Duty(percent);
    break;
  case 1:
    PWM_SetPA1_Duty(percent);
    break;
  case 2:
    PWM_SetPA11_Duty(percent);
    break;
  default:
    break;
  }
}

void pwm2_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 禁用 JTAG，释�?PB3~PB5

  // 1. 使能 GPIOB 时钟（必须先使能�?
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // 2. 配置 PB3 �?PB4 为推挽输�?
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4; // 同时配置这两个引�?
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       // 推挽输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // 输出速度
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void PWM_SetDuty(const PWM_Channel *pwm, float duty)
{
  uint16_t arr;
  uint16_t ccr_val;
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 100.0f)
    duty = 100.0f;

  arr = pwm->TIMx->ARR;
  ccr_val = (uint16_t)((duty / 100.0f) * arr);

  switch (pwm->channel)
  {
  case 1:
    pwm->TIMx->CCR1 = ccr_val;
    break;
  case 2:
    pwm->TIMx->CCR2 = ccr_val;
    break;
  case 3:
    pwm->TIMx->CCR3 = ccr_val;
    break;
  case 4:
    pwm->TIMx->CCR4 = ccr_val;
    break;
  }
}

// 设置 PWM 频率
void PWM_SetFreq(PWM_Channel *pwm, uint32_t frequency)
{
  uint32_t timclk = 72000000; // 72MHz
  uint32_t period = timclk / frequency - 1;

  // 更新定时器周�?
  pwm->TIMx->ARR = period;
  TIM_GenerateEvent(pwm->TIMx, TIM_EventSource_Update); // 触发更新事件，应用新频率
}

void GPIO_Config(void)
{
  GPIO_InitTypeDef gpio;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  // 在使能了 AFIO 时钟之后调用
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); // 开�?TIM1 部分重映�?�?CH2N �?PB0

  gpio.GPIO_Pin = GPIO_Pin_0; // PB0 = TIM1_CH2N
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽
  GPIO_Init(GPIOB, &gpio);
}

void TIM1_PWM_CH2N_Config(uint16_t arr, uint16_t psc, uint16_t ccr)
{
  TIM_OCInitTypeDef oc;
  TIM_BDTRInitTypeDef bdtr;

  // 1) 基本定时�?
  TIM_TimeBaseInitTypeDef tb;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  TIM_TimeBaseStructInit(&tb);
  tb.TIM_Period = arr;    // 自动重装�?
  tb.TIM_Prescaler = psc; // 预分�?
  tb.TIM_ClockDivision = TIM_CKD_DIV1;
  tb.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &tb);

  // 2) 通道2�?PWM1，仅使能互补输出(N)
  TIM_OCStructInit(&oc);
  oc.TIM_OCMode = TIM_OCMode_PWM1;
  oc.TIM_OutputState = TIM_OutputState_Disable;  // 主端不需�?
  oc.TIM_OutputNState = TIM_OutputNState_Enable; // 仅使�?N �?
  oc.TIM_Pulse = ccr;                            // 占空�?
  oc.TIM_OCPolarity = TIM_OCPolarity_High;       // 主端极性（无所谓）
  oc.TIM_OCNPolarity = TIM_OCNPolarity_High;     // N 端极性：需要反向就改为 Low
  oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
  oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  TIM_OC2Init(TIM1, &oc);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  // 3) 使能 ARR 预装�?
  TIM_ARRPreloadConfig(TIM1, ENABLE);

  // 4) BDTR（死�?刹车）。即使不用刹车，也要确保 MOE 可被置位
  TIM_BDTRStructInit(&bdtr);
  bdtr.TIM_OSSRState = TIM_OSSRState_Enable;
  bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
  bdtr.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  bdtr.TIM_Break = TIM_Break_Disable;
  bdtr.TIM_BreakPolarity = TIM_BreakPolarity_High;
  bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; // 允许自动置位 MOE
  TIM_BDTRConfig(TIM1, &bdtr);
  // �?5) 立即生效一次（�?PSC/ARR/CCR 装载到影子寄存器，并�?CNT�?
  TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
  // 6) 计数器启�?+ 主输出使能（关键�?
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE); // 设置 BDTR.MOE=1
}
// 0~100(�?；注意：PWM1模式�?00%无法做到绝对满占空，会差1/ARR个计�?
void TIM1_SetDuty_CH2N_Percent(float percent)
{
  uint32_t arr;
  uint32_t ccr;
  if (percent < 0.0f)
    percent = 0.0f;
  if (percent > 100.0f)
    percent = 100.0f;

  arr = TIM1->ARR; // 当前自动重装载�?
  ccr = (uint32_t)((percent * (arr + 1) / 100.0f) + 0.5f);

  if (ccr > arr)
    ccr = arr;                          // 饱和�?arr
  TIM_SetCompare2(TIM1, (uint16_t)ccr); // 预装载已开，下一次更新自动生�?
}
