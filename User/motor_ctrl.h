#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "stm32f10x.h"
typedef struct
{
    GPIO_TypeDef *AH_Port;
    uint16_t AH_Pin;
    GPIO_TypeDef *AL_Port;
    uint16_t AL_Pin;
    GPIO_TypeDef *E_Port;
    uint16_t E_Pin;
    uint8_t direction;
    uint8_t enabled;
    uint16_t timer_ms;
    uint8_t HC165Index;
    uint8_t HC165Value;
    uint8_t enable_pwm;
    uint8_t speed;
} MotorGPIO;
typedef struct
{ 
    uint8_t direction;
    uint8_t enabled;
    uint16_t timer_ms;
    uint8_t HC165Index;
    uint8_t HC165Value;
} MotorFull;

// 声明外部变量
extern MotorGPIO motors[3];
extern MotorFull motors1[2];
extern MotorFull motors2[2];

// 电机A引脚定义
#define MOTOR1_AH_PORT GPIOC
#define MOTOR1_AH_PIN  GPIO_Pin_14
#define MOTOR1_AL_PORT GPIOC
#define MOTOR1_AL_PIN  GPIO_Pin_15
#define MOTOR1_E_PORT  GPIOC
#define MOTOR1_E_PIN   GPIO_Pin_0

// 电机B引脚定义
#define MOTOR2_AH_PORT GPIOC
#define MOTOR2_AH_PIN  GPIO_Pin_1
#define MOTOR2_AL_PORT GPIOC
#define MOTOR2_AL_PIN  GPIO_Pin_2
#define MOTOR2_E_PORT  GPIOC
#define MOTOR2_E_PIN   GPIO_Pin_3

// 电机C引脚定义
#define MOTOR3_AH_PORT GPIOB
#define MOTOR3_AH_PIN  GPIO_Pin_0
#define MOTOR3_AL_PORT GPIOC
#define MOTOR3_AL_PIN  GPIO_Pin_5
#define MOTOR3_E_PORT  GPIOB
#define MOTOR3_E_PIN   GPIO_Pin_1

// 控制函数
void Motor_InitAll(void);

void Motor_Control(uint8_t motor_id, uint8_t direction, uint8_t enable);
void Motor595_SingleDir_Control(uint8_t motor_id, uint8_t enable);
void Motor4And5_Control(uint8_t motor_id, uint8_t direction, uint8_t enable);
void Motor6And7_Control(uint8_t motor_id, uint8_t direction, uint8_t enable);
#endif
