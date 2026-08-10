#include "stm32f10x.h"
#include "main.h"
#include <stdio.h>
#include <DELAY.h>
#include <USART.h>
#include <CLOCK.h>
#include <inttypes.h> // 引入标准整数格式化宏
#include <PWM.h>
#include "shift_register.h"
#include "shift_register_input.h"
#include "protocol.h"
#include "motor_ctrl.h"
#include "spi_helper.h"
#include "time.h"
#include "max31855.h"
#include "stepMotor.h"
#include "IWDG.h"
#include "step_dma.h"
#include "single_motor.h"
#include "reversible_motor.h"
#include "robot_arm.h"

#define MAIN_LOOP_PROCESS_LIMIT 8u

stepMotor stepMotorA = {GPIOB, GPIO_Pin_12, 0, 0, 1, 9, 1000, 1000, 0, 0, 100, 0, 0, 0, 0, 0, 4, 2000, 0};

extern MotorCtrl_t MotorCtrl[MOTOR_NUM]; // 595 输出电机控制状态
extern volatile char motor_tick_pending;
extern char handle_step_motor1;
extern char handle_step_motor2;

static uint8_t last_inputData[SHIFT_REGISTER_INPUT_COUNT];
static uint8_t inputData_inited;
static uint32_t last_temperature_report_ms;
static uint32_t last_single_motor_task_ms;

static void task_temperature_report(void)
{
    float temp;
    uint32_t now = millis();

    if ((now - last_temperature_report_ms) < 1000u)
        return;

    /* 热电偶读取较慢，只允许在 1 秒温度任务中执行。 */
    last_temperature_report_ms = now;
    temp = MAX31855_GetTemperature();
    temperature = (uint16_t)temp;
    send_temperature_frame(0x00);
}

static void task_uart_frames(void)
{
    uint8_t frame[10];
    uint8_t count = 0;

    /* 限制单轮处理量，避免串口持续有数据时阻塞其他任务。 */
    while (count < MAIN_LOOP_PROCESS_LIMIT && USART1_TakeFrame(frame))
    {
        parse_frame(frame);
        count++;
    }
}

static void task_protocol_commands(void)
{
    uint8_t cmd;
    uint8_t data[6];
    uint8_t count = 0;

    /* 命令队列与取帧任务使用相同的单轮处理上限。 */
    while (count < MAIN_LOOP_PROCESS_LIMIT && Protocol_TakeCommand(&cmd, data))
    {
        handle_command(cmd, data);
        count++;
    }
}

// static void task_step_done_report(void)
// {
//     uint8_t data[6] = {0};

//     if (stepdma_pb10_take_done_report())
//     {
//         data[0] = 1;
//         send_frame_event(0x1F, data);
//     }
// }

static void task_165_input_scan(void)
{
    ShiftRegisterInput_ReadAll(inputData);
}

static void task_input_change_report(void)
{
    uint8_t index;

    if (!inputData_inited)
    {
        for (index = 0u; index < SHIFT_REGISTER_INPUT_COUNT; index++)
        {
            last_inputData[index] = inputData[index];
        }
        inputData_inited = 1;
        return;
    }

    for (index = 0u; index < SHIFT_REGISTER_INPUT_COUNT; index++)
    {
        if (last_inputData[index] != inputData[index])
        {
            /* 两片 74HC165 的任一输入变化均上报当前完整快照。 */
            send_165Data();
            break;
        }
    }

    for (index = 0u; index < SHIFT_REGISTER_INPUT_COUNT; index++)
    {
        last_inputData[index] = inputData[index];
    }
}

static void task_single_motor(void)
{
    uint32_t now;
    uint32_t elapsed_ms;

    now = millis();
    elapsed_ms = now - last_single_motor_task_ms;
    if (elapsed_ms == 0u)
    {
        return;
    }

    /* 使用统一节拍推进各路独立任务，串口接收不会被定时或传感器检测阻塞。 */
    last_single_motor_task_ms = now;
    SingleMotor_Task(elapsed_ms);
    ReversibleMotor_Task(elapsed_ms);
}

#if 0
/* 旧非步进电机限位和定时逻辑仅作历史保留，已由统一电机状态机替代。 */
static void task_motor_limit_stop(void)
{
    uint8_t index;

    /* 限位检查在超时扣减前执行，保证限位停机优先。 */
    for (index = 0; index < 3; index++)
    {
        if (motors[index].timer_ms > 0 && motors[index].HC165Value > 0 &&
            (inputData[motors[index].HC165Index] & motors[index].HC165Value))
        {
            motors[index].timer_ms = 0;
            motors[index].enabled = 0;
            Motor_Control(index + 1, motors[index].direction, 0);
            motors[index].HC165Index = 0;
            motors[index].HC165Value = 0;
        }
    }
    for (index = 0; index < 2; index++)
    {
        if (motors2[index].timer_ms > 0 && motors2[index].HC165Value > 0 &&
            (inputData[motors2[index].HC165Index] & motors2[index].HC165Value))
        {
            motors2[index].timer_ms = 0;
            motors2[index].enabled = 0;
            Motor6And7_Control(index + 6, motors2[index].direction, 0);
            motors2[index].HC165Index = 0;
            motors2[index].HC165Value = 0;
            motor16_timeout_report[index] = 0;
        }
    }
}

static void task_motor_timeout_tick(void)
{
    uint8_t index;

    if (!motor_tick_pending)
        return;
    motor_tick_pending = 0;

    for (index = 0; index < MOTOR_NUM; index++)
    {
        if (MotorCtrl[index].enabled && MotorCtrl[index].timer_ms > 0 && --MotorCtrl[index].timer_ms == 0)
        {
            /* 595 单向电机计时到期后立即关闭输出。 */
            MotorCtrl[index].enabled = 0;
            Motor595_SingleDir_Control(index, 0);
        }
    }
    for (index = 0; index < 3; index++)
    {
        if (motors[index].timer_ms > 0 && --motors[index].timer_ms == 0)
        {
            motors[index].enabled = 0;
            Motor_Control(index + 1, motors[index].direction, 0);
            motors[index].HC165Index = 0;
            motors[index].HC165Value = 0;
        }
    }
    for (index = 0; index < 2; index++)
    {
        if (motors1[index].timer_ms > 0 && --motors1[index].timer_ms == 0)
        {
            motors1[index].enabled = 0;
            Motor4And5_Control(index + 4, motors1[index].direction, 0);
        }
        if (motors2[index].timer_ms > 0 && --motors2[index].timer_ms == 0)
        {
            motors2[index].enabled = 0;
            Motor6And7_Control(index + 6, motors2[index].direction, 0);
            motors2[index].HC165Index = 0;
            motors2[index].HC165Value = 0;
            if (motor16_timeout_report[index])
                send_motor16_timeout_event(index + 6, motors2[index].direction);
            motor16_timeout_report[index] = 0;
        }
    }
}
#endif

int main(void)
{
    SystemClock_Config();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    ShiftRegister_Init();
    ShiftRegisterInput_Init();
    Motor_InitAll();
    /* 新协议电机状态机统一管理 19 路单向和正反转电机的非阻塞任务。 */
    SingleMotor_Init();
    ReversibleMotor_Init();
    last_single_motor_task_ms = millis();

    SPI_GPIO_Init();
    SPI1_InitOnce();
    Timer2_Init();
    MAX31855_Init();
    TIM3_10us_Init();
    TIM4_10us_Init();
    stepdma_pb11_init(72000000);
    Stepper2_Init();
    PU3_Stepper_Init();
    /* 管理层仅初始化逻辑状态，绝不会在上电时输出 STEP。 */
    RobotArm_Init();
    GPIO_Config();
    TIM1_PWM_CH2N_Config(1000 - 1, 71, 0);
    PWM_PA0_PA1_PA11_Init();
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
    USART1_Init();
    Protocol_InitTxDiagnostics();
    /* 输出启动时的初始 74HC595 状态。 */
    ShiftRegister_WriteAll(HC595Data);
    while (1)
    {
        /* 串口1的485协议帧仅由中断入队，在主循环中统一解析。 */
        USART3_Process();
        task_temperature_report();
        task_uart_frames();
        task_protocol_commands();
        // task_step_done_report();
        task_165_input_scan();
        task_input_change_report();
        task_single_motor();
        /* 在主循环推进动作完成，不在 DMA 中断内执行业务状态机。 */
        RobotArm_Task();
    }
}
