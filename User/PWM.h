#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

// PWM 閫氶亾缁撴瀯浣?
typedef struct {
    TIM_TypeDef* TIMx;         // 瀹氭椂鍣?
    uint8_t channel;           // 閫氶亾锛?~4
    GPIO_TypeDef* GPIOx;       // GPIO 绔彛
    uint16_t GPIO_Pin;         // GPIO 寮曡剼
    uint32_t GPIO_RCC;         // GPIO RCC 鏃堕挓
    uint32_t TIM_RCC;          // TIM RCC 鏃堕挓
    FunctionalState remap; // ENABLE or DISABLE
} PWM_Channel;

// 澶栭儴鎺ュ彛
void pwm2_init(void);
void PWM_Init(const PWM_Channel* pwm, uint32_t period, uint32_t prescaler);
void PWM_SetDuty(const PWM_Channel* pwm, float duty);  // 0.0 ~ 100.0%
void PWM_SetFreq(PWM_Channel *pwm, uint32_t frequency);

void GPIO_Config(void);
void TIM1_PWM_CH2N_Config(uint16_t arr, uint16_t psc, uint16_t ccr);
void TIM1_SetDuty_CH2N_Percent(float percent);
void PWM_SetPB0_Duty(uint8_t percent);
void PWM_SetPC14_Duty(uint8_t percent);
void PWM_SetPC15_Duty(uint8_t percent);
void PWM_SetPC15_Motor(uint8_t enable);
void PWM_SetPC1_Motor(uint8_t enable);
void PWM_SetPC2_Motor(uint8_t enable);
void PWM_PA0_PA1_PA11_Init(void);
void PWM_SetPA0_Duty(uint8_t percent);
void PWM_SetPA1_Duty(uint8_t percent);
void PWM_SetPA11_Duty(uint8_t percent);
void PWM_SetExtraDuty(uint8_t channel, uint8_t percent);
/**
 * 获取 PWM 电机运行状态位图。
 *
 * bit0=PB0，bit1=PC14，bit2=PC15，bit3=PC1，
 * bit4=PC2，bit5=PA0，bit6=PA1，bit7=PA11。
 */
uint8_t PWM_GetMotorRunStatus(void);
#endif
