#include "stm32f10x.h"
#include "main.h"
#include <DELAY.h>
#include <USART.h>
#include <CLOCK.h>
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
#include "mixer_pwm.h"
#include "step_dma.h"
#include "single_motor.h"
#include "reversible_motor.h"
#include "robot_arm.h"
#include "protocol_v2.h"
#include "robot_arm_protocol.h"

#define MAIN_LOOP_PROCESS_LIMIT 8u
#define MAIN_LOOP_RX_BYTE_LIMIT 64u
#define MAIN_LOOP_SLOW_THRESHOLD_MS 100u

typedef enum
{
    /** 主循环当前未运行具体业务任务，或上一轮已经完成。 */
    MAIN_STAGE_IDLE = 0u,
    /** 正在消费 USART3 的接收缓冲区。 */
    MAIN_STAGE_USART3,
    /** 正在把 USART1 接收字节组为 V1/V2 协议帧。 */
    MAIN_STAGE_UART_FRAMES,
    /** 正在执行已入队的 V1 协议命令及其同步回复。 */
    MAIN_STAGE_PROTOCOL,
    /** 正在读取两片 74HC165 的完整输入快照。 */
    MAIN_STAGE_INPUT_SCAN,
    /** 正在比较输入快照并发送变化事件。 */
    MAIN_STAGE_INPUT_REPORT,
    /** 正在推进单向和正反转电机的非阻塞定时状态。 */
    MAIN_STAGE_MOTOR,
    /** 正在推进 RobotArm 的 Home、移动或安全状态机。 */
    MAIN_STAGE_ROBOT_ARM,
    /** 正在处理 RobotArm V2 的异步终态与发送队列。 */
    MAIN_STAGE_ROBOT_PROTOCOL
} MainLoopStage;

/* 主循环卡顿诊断仅保存在 RAM；单位为 TIM2 的毫秒单调 tick。 */
volatile uint8_t dbg_main_stage = MAIN_STAGE_IDLE;
volatile uint32_t dbg_main_loop_last_duration_ms = 0u;
volatile uint32_t dbg_main_loop_max_duration_ms = 0u;
volatile uint32_t dbg_main_loop_slow_count = 0u;
volatile uint8_t dbg_main_loop_last_slow_stage = MAIN_STAGE_IDLE;
volatile uint32_t dbg_main_loop_last_slow_duration_ms = 0u;
static uint32_t dbg_main_stage_start_ms;

stepMotor stepMotorA = {GPIOB, GPIO_Pin_12, 0, 0, 1, 9, 1000, 1000, 0, 0, 100, 0, 0, 0, 0, 0, 4, 2000, 0};

extern MotorCtrl_t MotorCtrl[MOTOR_NUM]; // 595 输出电机控制状态
extern volatile char motor_tick_pending;
extern char handle_step_motor1;
extern char handle_step_motor2;
extern volatile uint32_t dbg_usart2_tx_timeout;

static uint8_t last_inputData[SHIFT_REGISTER_INPUT_COUNT];
static uint8_t inputData_inited;
static uint32_t last_temperature_report_ms;
static uint32_t last_single_motor_task_ms;

/**
 * 标记即将执行的主循环任务。
 *
 * 若任务在硬件等待中停住，Keil Watch 中的 dbg_main_stage 会保持在该任务，
 * 用于把秒级延迟定位到协议、输入扫描、电机或机械臂模块。
 *
 * @param stage 当前即将运行的主循环关键任务标识。
 */
static void Main_DebugBeginStage(MainLoopStage stage)
{
    dbg_main_stage = (uint8_t)stage;
    dbg_main_stage_start_ms = millis();
}

/**
 * 汇总刚完成任务的耗时并保存慢任务现场。
 *
 * @return 刚完成任务所用的毫秒数。
 */
static uint32_t Main_DebugEndStage(void)
{
    uint32_t duration_ms = millis() - dbg_main_stage_start_ms;

    if (duration_ms > MAIN_LOOP_SLOW_THRESHOLD_MS)
    {
        dbg_main_loop_slow_count++;
        dbg_main_loop_last_slow_stage = dbg_main_stage;
        dbg_main_loop_last_slow_duration_ms = duration_ms;
    }
    return duration_ms;
}

/**
 * 执行低优先级温度周期上报。
 *
 * 每秒读取热电偶并尝试发送 CMD=0x00；命令回复已在本轮更早阶段处理，
 * 若发送权在首字节前不可用则由发送层记录失败，本轮温度可跳过而不阻塞命令。
 */
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

/**
 * 将 RobotArm V2 队列中的完整帧同步提交给 USART1。
 *
 * 返回成功代表全部字节已发送且最终 TC 已确认；若首字节前未取得发送权则返回失败，
 * RobotArm 发送队列保留该帧等待后续轮询，绝不重发已经开始的帧。
 *
 * @param frame 固定 24B V2 原始帧。
 * @param length 本次原始帧长度。
 * @return 1 表示完整物理发送完成；0 表示整帧尚未开始发送。
 */
static uint8_t task_protocol_v2_send(const uint8_t *frame, uint8_t length)
{
    /* V2 命令字位于固定帧第 2 字节，用于记录首字节前失败的持久现场。 */
    USART1_SetTxDiagnosticCommand((length >= 3u) ? frame[2] : 0xFFu);
    /* 发送层返回成功即代表最终 TC 已确认；软超时诊断不能把已完整发送的帧当作失败重发。 */
    if (!USART_SendBuffer(USART1, (uint8_t *)frame, length))
    {
        return 0u;
    }
    return 1u;
}

static void task_uart_frames(void)
{
    uint8_t frame[PROTOCOL_V1_FRAME_SIZE];
    uint8_t byte;
    uint8_t count = 0u;
    uint8_t byte_count = 0u;
    ProtocolV2Frame_t v2_frame;

    /* 每轮只消费有限字节；半帧状态保留到下一次主循环继续。 */
    while ((byte_count < MAIN_LOOP_RX_BYTE_LIMIT) && UART1GetByte(&byte))
    {
        ProtocolV2_InputByte(byte);
        byte_count++;
    }

    /* V1 仍进入原 parse_frame 和命令队列，旧业务行为保持不变。 */
    while ((count < MAIN_LOOP_PROCESS_LIMIT) &&
           ProtocolV2_TakeV1Frame(frame))
    {
        parse_frame(frame);
        count++;
    }

    count = 0u;
    while ((count < MAIN_LOOP_PROCESS_LIMIT) &&
           RobotArmProtocol_CanAcceptRequest() &&
           ProtocolV2_TakeFrame(&v2_frame))
    {
        RobotArmProtocol_HandleFrame(&v2_frame);
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
    /* 新协议电机状态机统一管理 19 路单向和四路保留正反转电机的非阻塞任务。 */
    SingleMotor_Init();
    ReversibleMotor_Init();
    last_single_motor_task_ms = millis();

    SPI_GPIO_Init();
    SPI1_InitOnce();
    Timer2_Init();
    MAX31855_Init();
    MixerPwm_Init();
    TIM4_10us_Init();
    stepdma_pb11_init(72000000);
    Stepper2_Init();
    PU3_Stepper_Init();
    /* 管理层仅初始化逻辑状态，绝不会在上电时输出 STEP。 */
    RobotArm_Init();
    // PWM_PA0_PA1_PA11_Init();
    USART1_Init();
    ProtocolV2_Init();
    RobotArmProtocol_Init(task_protocol_v2_send);
    Protocol_InitTxDiagnostics();
    /* 输出启动时的初始 74HC595 状态。 */
    ShiftRegister_WriteAll(HC595Data);
    while (1)
    {
        uint32_t loop_start_ms = millis();

        /* 串口1的485协议帧仅由中断入队，在主循环中统一解析。 */
        Main_DebugBeginStage(MAIN_STAGE_USART3);
        USART3_Process();
        (void)Main_DebugEndStage();
        /*
         * 先清空已到达的命令并发送 ACK/response，使周期 0x00 主动上报
         * 在同一轮主循环中只能排在命令回复之后，避免临界时刻抢占回复。
         */
        Main_DebugBeginStage(MAIN_STAGE_UART_FRAMES);
        task_uart_frames();
        (void)Main_DebugEndStage();
        /* 使用 TIM2 的毫秒时基推进搅拌到期停机，TIM3 仅保持 CH3 PWM 输出。 */
        MixerPwm_Process();
        Main_DebugBeginStage(MAIN_STAGE_PROTOCOL);
        task_protocol_commands();
        (void)Main_DebugEndStage();
        /* 0x00 为低优先级主动上报；USART 发送入口忙时会返回失败，本次上报可跳过。 */
        // task_temperature_report();
        // task_step_done_report();
        Main_DebugBeginStage(MAIN_STAGE_INPUT_SCAN);
        task_165_input_scan();
        (void)Main_DebugEndStage();
        Main_DebugBeginStage(MAIN_STAGE_INPUT_REPORT);
        task_input_change_report();
        (void)Main_DebugEndStage();
        Main_DebugBeginStage(MAIN_STAGE_MOTOR);
        task_single_motor();
        (void)Main_DebugEndStage();
        /* 在主循环推进动作完成，不在 DMA 中断内执行业务状态机。 */
        Main_DebugBeginStage(MAIN_STAGE_ROBOT_ARM);
        RobotArm_Task();
        (void)Main_DebugEndStage();
        Main_DebugBeginStage(MAIN_STAGE_ROBOT_PROTOCOL);
        RobotArmProtocol_Task();
        (void)Main_DebugEndStage();

        dbg_main_loop_last_duration_ms = millis() - loop_start_ms;
        if (dbg_main_loop_last_duration_ms > dbg_main_loop_max_duration_ms)
        {
            dbg_main_loop_max_duration_ms = dbg_main_loop_last_duration_ms;
        }
        dbg_main_stage = MAIN_STAGE_IDLE;
    }
}
