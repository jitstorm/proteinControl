#include "motor_ctrl.h"
#include "shift_register.h"
#include "DELAY.h"

MotorGPIO motors[3] = {
    {MOTOR1_AH_PORT, MOTOR1_AH_PIN, MOTOR1_AL_PORT, MOTOR1_AL_PIN, MOTOR1_E_PORT, MOTOR1_E_PIN},
    {MOTOR2_AH_PORT, MOTOR2_AH_PIN, MOTOR2_AL_PORT, MOTOR2_AL_PIN, MOTOR2_E_PORT, MOTOR2_E_PIN},
    {MOTOR3_AH_PORT, MOTOR3_AH_PIN, MOTOR3_AL_PORT, MOTOR3_AL_PIN, MOTOR3_E_PORT, MOTOR3_E_PIN},
};
MotorFull motors1[2];
MotorFull motors2[2];
void Motor_InitAll(void)
{
    int i;

    /* PC14/PC15��PC0~PC3��PC5��PB0/PB1 ���� ReversibleMotor ͳһ��ռ�� */
    for (i = 0; i < 3; i++)
    {
        motors[i].direction = 1;
        motors[i].timer_ms = 0;
        motors[i].enabled = 0;
        motors[i].HC165Index = 0;
        motors[i].HC165Value = 0;
    }
    for (i = 0; i < 2; i++)
    {
        motors1[i].direction = 1;
        motors1[i].timer_ms = 0;
        motors1[i].enabled = 0;
        motors1[i].HC165Index = 0;
        motors1[i].HC165Value = 0;
        motors2[i].direction = 1;
        motors2[i].timer_ms = 0;
        motors2[i].enabled = 0;
        motors2[i].HC165Index = 0;
        motors2[i].HC165Value = 0;
    }
}

void Motor_Control(uint8_t motor_id, uint8_t direction, uint8_t enable)
{
  
    (void)motor_id;
    (void)direction;
    (void)enable;
}

void Motor4And5_Control(uint8_t motor_id, uint8_t direction, uint8_t enable)
{
    static uint8_t shift_data = 0x00;

    uint8_t bit_pos;

    if (motor_id == 4)
    {
        bit_pos = 1; // 正转bit=1 (0x02)，反转bit=2 (0x04)
    }
    else if (motor_id == 5)
    {
        bit_pos = 3; // 正转bit=3 (0x08)，反转bit=4 (0x10)
    }
    else
    {
        return; // 非法编号
    }

    // 清除两个控制位（�?�?
    shift_data &= ~(0x03 << bit_pos); // 比如 bit_pos=1, 清除 bit1 �?bit2

    // 设置新的方向
    if (enable)
    {
        if (direction == 1) // 正转
        {
            shift_data |= (0x01 << bit_pos); // 设置正转�?
        }
        else if (direction == 2) // 反转
        {
            shift_data |= (0x02 << bit_pos); // 设置反转�?
        }
    }

    HC595Data[0] = shift_data;
    ShiftRegister_WriteAll(HC595Data);
}

/**
 * @brief 控制单向电机输出�? * @param motor_id 逻辑通道号，范围�?0 �?18�? * @param time 非零时开启，零时关闭�? */
#if 0
/* ???????????????????? single_motor.c ??? */
void Motor595_SingleDir_Control(uint8_t motor_id, uint8_t time)
{
    static const uint8_t output_register_index[MOTOR_NUM] = {
        0, 0, 0, 0, 0, 0, 0,
        2, 2, 2, 2,
        3, 3, 3, 3, 3, 3, 3, 3
    };
    static const uint8_t output_bit_index[MOTOR_NUM] = {
        1, 2, 3, 4, 5, 6, 7,
        0, 1, 2, 3,
        0, 1, 2, 3, 4, 5, 6, 7
    };
    uint8_t index;
    uint8_t bit_pos;

    if (motor_id >= MOTOR_NUM)
        return;

    /* MT5 位和 MT1~MT4 位未接入，逻辑通道仅映射到其余 19 路有效输出�?*/
    index = output_register_index[motor_id];
    bit_pos = output_bit_index[motor_id];

    if (time)
        HC595Data[index] |= (1 << bit_pos);
    else
        HC595Data[index] &= ~(1 << bit_pos);

    ShiftRegister_WriteAll(HC595Data);
}
#endif
void Motor6And7_Control(uint8_t motor_id, uint8_t direction, uint8_t enable)
{
    uint8_t bit_pos;

    if (motor_id == 6)
    {
        bit_pos = 1;
    }
    else if (motor_id == 7)
    {
        bit_pos = 3;
    }
    else
    {
        return;
    }

    HC595Data[0] &= ~(0x03 << bit_pos);

    if (enable)
    {
        if (direction == 1)
        {
            HC595Data[0] |= (0x01 << bit_pos);
        }
        else if (direction == 2)
        {
            HC595Data[0] |= (0x02 << bit_pos);
        }
    }

    ShiftRegister_WriteAll(HC595Data);
}
