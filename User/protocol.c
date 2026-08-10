#include "protocol.h"
#include "shift_register.h"
#include "shift_register_input.h"
#include "DELAY.h"
#include "motor_ctrl.h"
#include "spi_helper.h"
#include "main.h"
#include "max31855.h"
#include "stepMotor.h"
#include "shift_register.h"
#include "shift_register_input.h"
#include "PWM.h"
#include "USART.h"
#include <stdio.h>
#include "step_dma.h"
#include "single_motor.h"
#include "reversible_motor.h"
#include "time.h"
#include "robot_arm.h"

#define PROTOCOL_MAX_SIZE (SHIFT_REGISTER_COUNT + 3)
#define PROTOCOL_HEADER 0xAA
#define PROTOCOL_TAIL 0x55
#define FRAME_SIZE 10
#define PROTOCOL_COMMAND_QUEUE_SIZE 4
#define USART2_TX_WAIT_TIMEOUT_MS 5u
/* 步进方向使用 U86 对应的 595[1] 后四位，避免占用单向电机所在的 595[2]。 */
#define STEP_DIR_X_595_BIT 4u
#define STEP_DIR_RESERVED3_595_BIT 6u
#define STEP_DIR_RESERVED4_595_BIT 7u
extern MotorCtrl_t MotorCtrl[MOTOR_NUM]; // �?这是声明
extern stepMotor stepMotorA;
uint8_t pwm_duty;
uint16_t pwm_duty0;
int temperature;
char test_time;
uint8_t motor16_timeout_report[2] = {0, 0};
static uint8_t protocol_command_cmd[PROTOCOL_COMMAND_QUEUE_SIZE];
static uint8_t protocol_command_data[PROTOCOL_COMMAND_QUEUE_SIZE][6];
static uint8_t protocol_command_head = 0;
static uint8_t protocol_command_tail = 0;
static uint8_t protocol_command_count = 0;

volatile uint32_t dbg_usart_rx_bytes = 0;
volatile uint32_t dbg_usart_rx_frames = 0;
volatile uint32_t dbg_usart_ore = 0;
volatile uint32_t dbg_usart_fe = 0;
volatile uint32_t dbg_usart_ne = 0;
volatile uint32_t dbg_usart_pe = 0;
volatile uint32_t dbg_parser_bad_head = 0;
volatile uint32_t dbg_parser_bad_tail = 0;
volatile uint32_t dbg_parser_bad_checksum = 0;
volatile uint32_t dbg_frame_queue_overflow = 0;
volatile uint32_t dbg_cmd_queue_overflow = 0;
volatile uint32_t dbg_rx_cmd_count[256] = {0};
volatile uint32_t dbg_handle_cmd_count[256] = {0};
volatile uint32_t dbg_tx_cmd_count[256] = {0};
volatile uint32_t dbg_tx_ack_enter_count[256] = {0};
volatile uint32_t dbg_tx_ack_complete_count[256] = {0};
volatile uint32_t dbg_tx_event_enter_count[256] = {0};
volatile uint32_t dbg_tx_event_complete_count[256] = {0};
volatile uint32_t dbg_usart2_tx_frames_begin = 0;
volatile uint32_t dbg_usart2_tx_frames_complete = 0;
volatile uint32_t dbg_usart2_tx_bytes = 0;
volatile uint32_t dbg_usart2_tx_timeout = 0;
volatile uint32_t dbg_usart2_tx_last_begin_ms = 0;
volatile uint32_t dbg_usart2_tx_last_complete_ms = 0;
volatile uint32_t dbg_usart2_tx_last_timeout_ms = 0;
volatile uint8_t dbg_usart2_tx_last_cmd = 0;
volatile uint8_t dbg_usart2_tx_last_timeout_stage = 0;
volatile uint8_t dbg_usart2_tx_last_byte_index = 0;
volatile uint8_t dbg_usart2_tx_last_timeout_cmd = 0;
volatile uint8_t dbg_usart2_tx_last_timeout_source = 0;
volatile uint8_t dbg_usart2_tx_current_byte_index = 0;
volatile uint8_t dbg_usart2_tx_current_cmd = 0;
volatile uint8_t dbg_usart2_tx_current_source = 0;

typedef enum
{
    TX_SOURCE_ACK = 0,
    TX_SOURCE_EVENT,
    TX_SOURCE_DEBUG
} TxSource;

static void send_frame_internal(uint8_t cmd, uint8_t *data, TxSource source);
#if 0
static void send_frame_debug(uint8_t cmd, uint8_t *data);
#endif

/**
 * 显式清除最近一次 USART2 TX timeout 证据。
 */
void Protocol_ResetUsart2TxTimeoutEvidence(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    dbg_usart2_tx_last_timeout_stage = 0u;
    dbg_usart2_tx_last_byte_index = 0xFFu;
    dbg_usart2_tx_last_timeout_cmd = 0u;
    dbg_usart2_tx_last_timeout_source = 0u;
    dbg_usart2_tx_last_timeout_ms = 0u;
    if (!primask)
    {
        __enable_irq();
    }
}

/**
 * 初始化 USART2 TX 诊断状态。
 */
void Protocol_InitTxDiagnostics(void)
{
    Protocol_ResetUsart2TxTimeoutEvidence();
    dbg_usart2_tx_current_byte_index = 0xFFu;
    dbg_usart2_tx_current_cmd = 0u;
    dbg_usart2_tx_current_source = TX_SOURCE_ACK;
}

/**
 * 原子记录 USART2 TX timeout 现场。
 *
 * stage=1 表示 TXE timeout，stage=2 表示 TC timeout。
 */
void Protocol_RecordUsart2TxTimeout(uint8_t stage)
{
    uint32_t now;
    uint32_t primask;

    now = millis();
    primask = __get_PRIMASK();
    __disable_irq();
    dbg_usart2_tx_timeout++;
    dbg_usart2_tx_last_timeout_stage = stage;
    dbg_usart2_tx_last_byte_index =
        (stage == 2u) ? 9u : dbg_usart2_tx_current_byte_index;
    dbg_usart2_tx_last_timeout_cmd = dbg_usart2_tx_current_cmd;
    dbg_usart2_tx_last_timeout_source = dbg_usart2_tx_current_source;
    dbg_usart2_tx_last_timeout_ms = now;
    if (!primask)
    {
        __enable_irq();
    }
}

/* PB10 传感器停止配置，只在 0x20 命令运行期间有效。 */
static uint8_t pb10_stop_sensor_enabled = 0;
static uint8_t pb10_stop_sensor_index = 0;
static uint8_t pb10_stop_sensor_mask = 0;
static uint8_t pb10_stop_sensor_level = 0;
static uint8_t pb10_stop_sensor_number = 0;

static void Protocol_DisablePb10StopSensor(void)
{
    pb10_stop_sensor_enabled = 0;
    pb10_stop_sensor_index = 0;
    pb10_stop_sensor_mask = 0;
    pb10_stop_sensor_level = 0;
    pb10_stop_sensor_number = 0;
}

static uint8_t Protocol_ConfigurePb10StopSensor(uint8_t sensor_number, uint8_t trigger_level)
{
    uint8_t sensor_index;

    /* 硬件仅有 16 路 HC165 输入，拒绝越界编号以避免访问 inputData 越界。 */
    if (sensor_number < 1u || sensor_number > SHIFT_REGISTER_INPUT_CHANNEL_COUNT)
    {
        Protocol_DisablePb10StopSensor();
        return 0;
    }

    sensor_index = (uint8_t)((sensor_number - 1u) / 8u);
    /* 串行读取顺序与传感器编号相反，因此按当前芯片数量反序换算。 */
    pb10_stop_sensor_index = (uint8_t)((SHIFT_REGISTER_INPUT_COUNT - 1u) - sensor_index);
    pb10_stop_sensor_mask = (uint8_t)(1u << ((sensor_number - 1u) % 8u));
    pb10_stop_sensor_level = trigger_level ? 1u : 0u;
    pb10_stop_sensor_number = sensor_number;
    pb10_stop_sensor_enabled = 1u;
    return 1;
}

void Protocol_CheckPb10StopSensor(const uint8_t *sensor_data)
{
    uint8_t current_level;
    uint8_t report_data[6] = {0};

    if (!pb10_stop_sensor_enabled)
    {
        return;
    }

    /* 步进已自然结束时清除绑定，防止后续误触发。 */
    if (!stepdma_pb10_is_running())
    {
        Protocol_DisablePb10StopSensor();
        return;
    }

    current_level = (sensor_data[pb10_stop_sensor_index] & pb10_stop_sensor_mask) ? 1u : 0u;
    if (current_level != pb10_stop_sensor_level)
    {
        return;
    }

    /* 先撤销传感器绑定再停机，保证同一次触发只上报一次。 */
    report_data[0] = 1u;
    report_data[1] = 1u;
    report_data[2] = pb10_stop_sensor_number;
    (void)report_data;
    Protocol_DisablePb10StopSensor();
    stepdma_pb10_stop();
    // send_frame_event(0x1F, report_data);
}
void initPacket(Packet *pkt)
{
    pkt->STX = 0xAA;
    pkt->ETX = 0x55;
    pkt->CMD = 0;
    pkt->DATA1 = 0;
    pkt->DATA2 = 0;
    pkt->DATA3 = 0;
    pkt->DATA4 = 0;
    pkt->DATA5 = 0;
    pkt->DATA6 = 0;
    pkt->SUM = 0;
}

void computeChecksum(Packet *pkt)
{
    /* 校验和覆盖帧头、命令和 6 字节数据，帧尾固定放在最后一个字节。 */
    pkt->SUM = pkt->STX + pkt->CMD + pkt->DATA1 + pkt->DATA2 +
               pkt->DATA3 + pkt->DATA4 + pkt->DATA5 + pkt->DATA6;
}

void createHandshakeReply(Packet *pkt)
{
    initPacket(pkt);
    pkt->CMD = 0x00;
    pkt->DATA1 = 0x00;
    pkt->DATA2 = 0x00;
    pkt->DATA3 = 0x00;
    pkt->DATA4 = 0x00;
    pkt->DATA5 = 0x00;
    pkt->DATA6 = 0x00;
    // USART_SendData(USART1, pkt->SUM);
    computeChecksum(pkt);

    // USART_SendData(USART1, pkt->SUM);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        ;
}

void sendPacket(USART_TypeDef *USARTx, const Packet *pkt)
{
    int i;
    uint8_t frame[10];

    frame[0] = pkt->STX;
    frame[1] = pkt->CMD;
    frame[2] = pkt->DATA1;
    frame[3] = pkt->DATA2;
    frame[4] = pkt->DATA3;
    frame[5] = pkt->DATA4;
    frame[6] = pkt->DATA5;
    frame[7] = pkt->DATA6;
    /* 线上的固定顺序为帧尾 0x55 在前，校验和在最后。 */
    frame[8] = pkt->ETX;
    frame[9] = pkt->SUM;

    /* Packet 结构体字段顺序保持兼容，发送时按线上的 SUM、55 顺序重组。 */
    for (i = 0; i < 10; i++)
    {
        while (!(USARTx->SR & USART_SR_TXE))
            ; // 等待发送缓冲区为空
        USART_SendByte(USARTx, frame[i]);
    }
}

void USART_ProtocolHandler(uint8_t *buffer)
{
    int i;
    for (i = 0; i < PROTOCOL_MAX_SIZE; i++)
    {
        USART_SendByte(USART1, buffer[i]);
        buffer[i] = 0x00;
    }
}
#if 0
/* 旧 0x7E 调试命令已停用，诊断实现保留供后续恢复。 */
static uint32_t Protocol_GetDebugCounter(uint8_t type, uint8_t index)
{
    /* ??????????????????? 10 ??????? */
    if (type == 0u)
    {
        switch (index)
        {
        case 0:
            return dbg_usart_rx_bytes;
        case 1:
            return dbg_usart_rx_frames;
        case 2:
            return dbg_usart_ore;
        case 3:
            return dbg_usart_fe;
        case 4:
            return dbg_usart_ne;
        case 5:
            return dbg_usart_pe;
        case 6:
            return dbg_parser_bad_head;
        case 7:
            return dbg_parser_bad_tail;
        case 8:
            return dbg_parser_bad_checksum;
        case 9:
            return dbg_frame_queue_overflow;
        case 10:
            return dbg_cmd_queue_overflow;
        case 11:
            return dbg_usart2_tx_frames_begin;
        case 12:
            return dbg_usart2_tx_frames_complete;
        case 13:
            return dbg_usart2_tx_bytes;
        case 14:
            return dbg_usart2_tx_timeout;
        case 15:
            return dbg_usart2_tx_last_cmd;
        case 16:
            return dbg_usart2_tx_last_begin_ms;
        case 17:
            return dbg_usart2_tx_last_complete_ms;
        case 18:
            return dbg_usart2_tx_last_timeout_stage;
        case 19:
            return dbg_usart2_tx_last_byte_index;
        case 20:
            return dbg_usart2_tx_last_timeout_cmd;
        case 21:
            return dbg_usart2_tx_last_timeout_source;
        case 22:
            return dbg_usart2_tx_last_timeout_ms;
        case 23:
            return dbg_usart2_tx_current_cmd;
        case 24:
            return dbg_usart2_tx_current_source;
        case 25:
            return dbg_usart2_tx_current_byte_index;
        default:
            return 0;
        }
    }
    if (type == 1u)
    {
        return dbg_rx_cmd_count[index];
    }
    if (type == 2u)
    {
        return dbg_handle_cmd_count[index];
    }
    if (type == 3u)
    {
        return dbg_tx_cmd_count[index];
    }
    if (type == 4u)
    {
        return dbg_tx_ack_enter_count[index];
    }
    if (type == 5u)
    {
        return dbg_tx_event_enter_count[index];
    }
    if (type == 6u)
    {
        return dbg_tx_ack_complete_count[index];
    }
    if (type == 7u)
    {
        return dbg_tx_event_complete_count[index];
    }
    return 0;
}

static void Protocol_SendDebugCounter(uint8_t *data)
{
    uint32_t value;

    value = Protocol_GetDebugCounter(data[0], data[1]);
    /* D2..D5 ?????????? uint32 ???? */
    data[2] = (uint8_t)(value >> 24);
    data[3] = (uint8_t)(value >> 16);
    data[4] = (uint8_t)(value >> 8);
    data[5] = (uint8_t)value;
    send_frame_debug(0x7E, data);
}
#endif

static void Protocol_QueueCommand(uint8_t cmd, uint8_t *data)
{
    uint8_t i;

    if (protocol_command_count >= PROTOCOL_COMMAND_QUEUE_SIZE)
    {
        /* ?????????????????????????? */
        dbg_cmd_queue_overflow++;
        return;
    }

    protocol_command_cmd[protocol_command_tail] = cmd;
    for (i = 0; i < 6; i++)
    {
        protocol_command_data[protocol_command_tail][i] = data[i];
    }

    protocol_command_tail++;
    if (protocol_command_tail >= PROTOCOL_COMMAND_QUEUE_SIZE)
    {
        protocol_command_tail = 0;
    }
    protocol_command_count++;
}

uint8_t Protocol_TakeCommand(uint8_t *cmd, uint8_t *data)
{
    uint8_t i;

    if (protocol_command_count == 0)
    {
        return 0;
    }

    *cmd = protocol_command_cmd[protocol_command_head];
    for (i = 0; i < 6; i++)
    {
        data[i] = protocol_command_data[protocol_command_head][i];
    }

    protocol_command_head++;
    if (protocol_command_head >= PROTOCOL_COMMAND_QUEUE_SIZE)
    {
        protocol_command_head = 0;
    }
    protocol_command_count--;

    return 1;
}

void parse_frame(uint8_t *frame)
{
    uint8_t cmd;
    uint8_t data[6];
    uint8_t checksum = 0;
    int j;

    if (frame[0] != PROTOCOL_HEADER)
    {
        dbg_parser_bad_head++;
        return;
    }

    if (frame[8] != PROTOCOL_TAIL)
    {
        dbg_parser_bad_tail++;
        return;
    }

    for (j = 0; j < 8; j++)
    {
        checksum += frame[j];
    }
    if (checksum != frame[9])
    {
        dbg_parser_bad_checksum++;
        return;
    }

    cmd = frame[1];
    /* ???????????????? CMD ??? */
    dbg_usart_rx_frames++;
    dbg_rx_cmd_count[cmd]++;
    for (j = 0; j < 6; j++)
    {
        data[j] = frame[2 + j];
    }

    Protocol_QueueCommand(cmd, data);
}
static void send_165DataSource(TxSource source)
{
    uint8_t data[6] = {0};

    data[0] = inputData[0];
    data[1] = inputData[1];
    send_frame_internal(0x01, data, source);
}
void send_165Data()
{
    send_165DataSource(TX_SOURCE_EVENT);
}

void StepMotor_SetDirction(char direction)
{
    /* 兼容旧调用入口：统一使用 X 轴对应的 DIR1 输出。 */
    HC595Data[1] = (HC595Data[1] & ~(1 << STEP_DIR_X_595_BIT)) |
                   ((direction & 0x01) << STEP_DIR_X_595_BIT);
    ShiftRegister_WriteAll(HC595Data);
}

void getHc165IndexByData(char motorNumber, char HC165number)
{
    motors[motorNumber - 1].HC165Index = (SHIFT_REGISTER_INPUT_COUNT - 1u) -
                                         ((HC165number - 1) / 8);
    /* 按串行读取顺序换算芯片索引，避免依赖固定芯片数量。 */
    motors[motorNumber - 1].HC165Value = 1 << ((HC165number - 1) % 8);
}
void getHc165IndexForMotorFull(MotorFull *motor, char HC165number)
{
    uint8_t index = (HC165number - 1) / 8;
    uint8_t bitmask = 1 << ((HC165number - 1) % 8);

    /* 串行读取顺序与传感器编号相反，使用芯片总数计算反序索引。 */
    motor->HC165Index = (SHIFT_REGISTER_INPUT_COUNT - 1u) - index;
    motor->HC165Value = bitmask;
}

void getHc165IndexForStepMotor(stepMotor *motor, char HC165number)
{
    uint8_t index = (HC165number - 1) / 8;        // 第几�?65芯片（从0开始）
    uint8_t bitmask = 1 << (HC165number - 1) % 8; // 掩码：哪个bit�?

    // HC165 芯片编号按串行读取顺序反向映射，避免依赖固定芯片数量。
    motor->hc165_index = (SHIFT_REGISTER_INPUT_COUNT - 1u) - index;

    /* 串行读取顺序与传感器编号相反，使用芯片总数计算反序索引。 */
    motor->hc165_index = (SHIFT_REGISTER_INPUT_COUNT - 1u) - index;
    motor->hc165_value = bitmask;
}
void send_temperature_frame(uint8_t cmd)
{
    uint8_t data[6];
    uint16_t remote_temperature;

    remote_temperature = USART1_GetRemoteTemperature();


    data[0] = (uint8_t)(remote_temperature >> 8);
    data[1] = (uint8_t)remote_temperature;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    send_frame_event(cmd, data);
}
// 模拟器测试用
// void send_temperature_frame(uint8_t cmd)
// {
//     uint8_t data[6];
//     uint16_t remote_temperature;
//     uint16_t local_temperature;

//     remote_temperature = USART1_GetRemoteTemperature();
//     // remote_temperature = 215;
//     local_temperature = (uint16_t)temperature;

//     data[0] = (uint8_t)(remote_temperature >> 8);
//     data[1] = (uint8_t)0x32;
//     data[2] = (uint8_t)(local_temperature >> 8);
//     data[3] = (uint8_t)0x16;
//     data[4] = 0;
//     data[5] = 0;
//     send_frame_event(cmd, data);
// }

void send_motor16_timeout_event(uint8_t motor_id, uint8_t direction)
{
    uint8_t data[6] = {0};

    data[0] = motor_id;
    data[1] = direction;
    data[4] = 1; /* 1: timed operation completed */
    send_frame_event(0x16, data);
}


#define SINGLE_MOTOR_RESULT_OK 0x00u
#define SINGLE_MOTOR_RESULT_INVALID_MOTOR 0x01u
#define SINGLE_MOTOR_RESULT_INVALID_ACTION 0x02u
#define SINGLE_MOTOR_RESULT_INVALID_SENSOR 0x03u
#define SINGLE_MOTOR_RESULT_INVALID_TIME 0x04u
#define SINGLE_MOTOR_RESULT_UNSUPPORTED 0x05u

static void Protocol_SendSingleMotorResult(uint8_t cmd, uint8_t result)
{
    uint8_t response[6] = {0u};

    response[0] = result;
    if (result != SINGLE_MOTOR_RESULT_OK)
    {
        SingleMotor_RecordError(result);
    }
    send_frame(cmd, response);
}

/**
 * @brief 发送正反转电机命令的统一结果应答。
 * @param cmd 当前命令字。
 * @param result 执行结果码。
 */
static void Protocol_SendReversibleMotorResult(uint8_t cmd, uint8_t result)
{
    uint8_t response[6] = {0u};

    /* 沿用既有应答帧，仅在 D0 返回结果，其他保留字节固定清零。 */
    response[0] = result;
    if (result != SINGLE_MOTOR_RESULT_OK)
    {
        ReversibleMotor_RecordError(result);
    }
    send_frame(cmd, response);
}

static uint8_t Protocol_IsAllZero(const uint8_t *data, uint8_t start_index)
{
    uint8_t index;

    for (index = start_index; index < 6u; index++)
    {
        if (data[index] != 0u)
        {
            return 0u;
        }
    }
    return 1u;
}

/* 协议仅定义停止、正转和反转；换向死区时实际输出已停止，因此按停止上报。 */
static uint8_t Protocol_GetReversibleMotorStateForResponse(ReversibleMotorState state)
{
    if (state == REV_MOTOR_DEADTIME)
    {
        return (uint8_t)REV_MOTOR_STOPPED;
    }

    return (uint8_t)state;
}

/**
 * @brief 分发当前协议命令。
 * @param cmd 已完成校验的命令字。
 * @param data 六字节命令数据。
 */
void handle_command(uint8_t cmd, uint8_t *data)
{
    uint32_t duration_ms;
    uint16_t timeout_100ms;
    uint8_t bitmap[3];

    dbg_handle_cmd_count[cmd]++;

    switch (cmd)
    {
    case 0x00:
        /* 保留旧功能：当前命令为空操作。 */
        break;
    case 0x01:
        /* 保留旧功能：读取两字节 74HC165 输入快照。 */
        send_165DataSource(TX_SOURCE_ACK);
        break;
    case 0x05:
        if (data[0] < 1u || data[0] > SINGLE_MOTOR_COUNT)
        {
            /* 05 的回应格式固定回显请求字段，错误仅保留在内部诊断。 */
            SingleMotor_RecordError(SINGLE_MOTOR_RESULT_INVALID_MOTOR);
            send_frame(cmd, data);
            break;
        }
        if (data[1] > 1u || !Protocol_IsAllZero(data, 2u))
        {
            SingleMotor_RecordError(SINGLE_MOTOR_RESULT_INVALID_ACTION);
            send_frame(cmd, data);
            break;
        }
        SingleMotor_Immediate(data[0], data[1]);
        /* 成功回应按协议保留电机编号与动作字段。 */
        send_frame(cmd, data);
        break;
    case 0x06:
        if (data[0] < 1u || data[0] > SINGLE_MOTOR_COUNT)
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_MOTOR);
            break;
        }
        if (!Protocol_IsAllZero(data, 2u))
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        /* 文档约定 DATA2 为 100ms 单位的单字节定时时间。 */
        duration_ms = (uint32_t)data[1] * 100u;
        if (duration_ms == 0u)
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_TIME);
            break;
        }
        SingleMotor_StartTimed(data[0], duration_ms);
        Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_OK);
        break;
    case 0x07:
        if (data[0] < 1u || data[0] > SINGLE_MOTOR_COUNT)
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_MOTOR);
            break;
        }
        if (data[1] < 1u || data[1] > SHIFT_REGISTER_INPUT_CHANNEL_COUNT)
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_SENSOR);
            break;
        }
        if (data[2] > 1u || data[5] != 0u)
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        timeout_100ms = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
        if (timeout_100ms == 0u)
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_TIME);
            break;
        }
        SingleMotor_StartSensorTimed(data[0], data[1], data[2],
                                     (uint32_t)timeout_100ms * 100u);
        Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_OK);
        break;
    case 0x08:
        /* 查询帧不允许携带参数，返回三字节单向电机运行位图。 */
        if (!Protocol_IsAllZero(data, 0u))
        {
            Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        SingleMotor_GetRunningBitmap(bitmap);
        data[0] = bitmap[0];
        data[1] = bitmap[1];
        data[2] = bitmap[2];
        data[3] = 0u;
        data[4] = 0u;
        data[5] = 0u;
        send_frame(cmd, data);
        break;
    case 0x09:
        /* 立即控制仅接受已配置的正反转电机编号和停止/正转/反转动作。 */
        if (data[0] < 1u || data[0] > REVERSIBLE_MOTOR_COUNT)
        {
            /* 09 的回应格式固定回显请求字段，错误仅保留在内部诊断。 */
            ReversibleMotor_RecordError(SINGLE_MOTOR_RESULT_INVALID_MOTOR);
            send_frame(cmd, data);
            break;
        }
        if (data[1] > 2u || !Protocol_IsAllZero(data, 2u))
        {
            ReversibleMotor_RecordError(SINGLE_MOTOR_RESULT_INVALID_ACTION);
            send_frame(cmd, data);
            break;
        }
        ReversibleMotor_Immediate(data[0], data[1]);
        /* 成功回应按协议保留电机编号与动作字段。 */
        send_frame(cmd, data);
        break;
    case 0x0A:
        /* 文档约定 DATA3 为 100ms 单位的单字节定时时间。 */
        if (data[0] < 1u || data[0] > REVERSIBLE_MOTOR_COUNT)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_MOTOR);
            break;
        }
        if (data[1] < 1u || data[1] > 2u)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        if (!Protocol_IsAllZero(data, 3u))
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        duration_ms = (uint32_t)data[2] * 100u;
        if (duration_ms == 0u)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_TIME);
            break;
        }
        ReversibleMotor_StartTimed(data[0],
                                   (data[1] == 1u) ? REV_MOTOR_FORWARD : REV_MOTOR_REVERSE,
                                   duration_ms);
        Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_OK);
        break;
    case 0x0B:
        /* 传感器任务在完成全部校验后才允许改变电机输出。 */
        if (data[0] < 1u || data[0] > REVERSIBLE_MOTOR_COUNT)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_MOTOR);
            break;
        }
        if (data[1] < 1u || data[1] > 2u || data[3] > 1u)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        if (data[2] < 1u || data[2] > SHIFT_REGISTER_INPUT_CHANNEL_COUNT)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_SENSOR);
            break;
        }
        timeout_100ms = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
        if (timeout_100ms == 0u)
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_TIME);
            break;
        }
        ReversibleMotor_StartSensorTimed(data[0],
                                         (data[1] == 1u) ? REV_MOTOR_FORWARD : REV_MOTOR_REVERSE,
                                         data[2], data[3], (uint32_t)timeout_100ms * 100u);
        Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_OK);
        break;
    case 0x0C:
        if (!Protocol_IsAllZero(data, 0u))
        {
            Protocol_SendReversibleMotorResult(cmd, SINGLE_MOTOR_RESULT_INVALID_ACTION);
            break;
        }
        /* 文档约定 D0..D4 分别返回五路电机状态，D5 保留。 */
        data[0] = Protocol_GetReversibleMotorStateForResponse(g_reversible_motors[0].state);
        data[1] = Protocol_GetReversibleMotorStateForResponse(g_reversible_motors[1].state);
        data[2] = Protocol_GetReversibleMotorStateForResponse(g_reversible_motors[2].state);
        data[3] = Protocol_GetReversibleMotorStateForResponse(g_reversible_motors[3].state);
        data[4] = Protocol_GetReversibleMotorStateForResponse(g_reversible_motors[4].state);
        data[5] = 0u;
        send_frame(cmd, data);
        break;
    case 0x1B:
        send_frame(cmd, data);
        /* PU1 仅修改 DIR1，确保同片 DIR2、DIR3 等输出保持不变。 */
        HC595Data[1] = (HC595Data[1] & ~(1 << STEP_DIR_X_595_BIT)) |
                       ((data[2] & 0x01) << STEP_DIR_X_595_BIT);
        ShiftRegister_WriteAll(HC595Data);
        stepdma_pb11_request_trap(data[0] << 8 | data[1], 500u, data[3] * 200u, 100000u);
        /* 旧调试命令绕过管理层，完成后坐标不可再作为绝对位置依据。 */
        RobotArm_InvalidatePosition(ROBOT_AXIS_X);
        break;
    case 0x1C:
        Protocol_DisablePb10StopSensor();
        send_frame(cmd, data);
        /* 运行中的第二轴先停止，避免在脉冲输出期间直接切换第二片 595 的 Q5。 */
        if (stepdma_pb10_is_running())
        {
            Stepper2_Stop();
        }
        Stepper2_SetDirection(data[2]);
        Delay_us(2u);
        stepdma_pb10_request_trap(data[0] << 8 | data[1], 500, data[3] * 200, 100000);
        /* 保持旧帧行为，仅使管理层放弃该轴的旧坐标。 */
        RobotArm_InvalidatePosition(ROBOT_AXIS_Y);
        break;
    case 0x1E:
        /* PU3 沿用两轴步进参数格式，方向由 U86 的 DIR3(Q6) 独立控制。 */
        send_frame(cmd, data);
        (void)PU3_Stepper_Start(data[0] << 8 | data[1], data[2], 500u,
                                 data[3] * 200u, 100000u);
        /* 旧调试命令直接控制 PU3，必须失效化管理层 Z 坐标。 */
        RobotArm_InvalidatePosition(ROBOT_AXIS_Z);
        break;
    case 0x1F:
        data[0] = stepdma_pb10_is_running() ? 0 : 1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
        send_frame(cmd, data);
        break;
    case 0x20:
        if (Protocol_ConfigurePb10StopSensor(data[4], data[5]))
        {
            send_frame(cmd, data);
            /* 传感器任务同样禁止直接在运行中切换方向。 */
            if (stepdma_pb10_is_running())
            {
                Stepper2_Stop();
            }
            Stepper2_SetDirection(data[2]);
            Delay_us(2u);
            stepdma_pb10_request_trap(data[0] << 8 | data[1], 500, data[3] * 200, 100000);
            /* 带传感器停止的旧命令同样不向管理层报告精确执行步数。 */
            RobotArm_InvalidatePosition(ROBOT_AXIS_Y);
        }
        else
        {
            data[4] = 0;
            send_frame(cmd, data);
        }
        break;
    default:
        Protocol_SendSingleMotorResult(cmd, SINGLE_MOTOR_RESULT_UNSUPPORTED);
        break;
    }
}

void send_frame_usart1(uint8_t cmd, uint8_t *data)
{
    uint8_t frame[10];
    uint8_t checksum;
    int i;
    int j;
    frame[0] = 0xAA;
    frame[1] = cmd;
    for (i = 0; i < 6; i++)
    {
        frame[2 + i] = data[i];
    }

    checksum = 0;
    for (i = 0; i < 8; i++)
    {
        checksum += frame[i];
    }
    frame[8] = 0x55;
    frame[9] = checksum;

    for (j = 0; j < 10; j++)
    {
        USART_SendByte(USART1, frame[j]);
    }
}

static void send_frame_internal(uint8_t cmd, uint8_t *data, TxSource source)
{
    uint8_t frame[10];
    uint8_t checksum;
    uint8_t count_business;
    uint32_t tx_wait_start;
    uint32_t timeout_before;
    int i;
    int j;

    /* 当前上下文供 TXE/TC timeout 快照使用。 */
    dbg_usart2_tx_current_cmd = cmd;
    dbg_usart2_tx_current_source = (uint8_t)source;
    dbg_usart2_tx_current_byte_index = 0xFFu;

    /* DEBUG 回复不进入业务 TX、ACK 或 EVENT 统计。 */
    count_business = (source != TX_SOURCE_DEBUG) ? 1u : 0u;
    if (count_business)
    {
        dbg_tx_cmd_count[cmd]++;
        if (source == TX_SOURCE_ACK)
        {
            dbg_tx_ack_enter_count[cmd]++;
        }
        else
        {
            dbg_tx_event_enter_count[cmd]++;
        }
    }

    frame[0] = 0xAA;
    frame[1] = cmd;
    for (i = 0; i < 6; i++)
    {
        frame[2 + i] = data[i];
    }

    checksum = 0;
    for (i = 0; i < 8; i++)
    {
        checksum += frame[i];
    }
    frame[8] = 0x55;
    frame[9] = checksum;

    /* 正常帧开始不得清除之前持久化的 timeout 证据。 */
    timeout_before = dbg_usart2_tx_timeout;
    if (count_business)
    {
        dbg_usart2_tx_last_cmd = cmd;
        dbg_usart2_tx_last_begin_ms = millis();
        dbg_usart2_tx_frames_begin++;
    }

    for (j = 0; j < 10; j++)
    {
        /* 写每个字节前先发布当前帧内位置。 */
        dbg_usart2_tx_current_byte_index = (uint8_t)j;
        USART_SendByte(USART1, frame[j]);
        if (dbg_usart2_tx_timeout != timeout_before)
        {
            dbg_usart2_tx_current_byte_index = 0xFFu;
            return;
        }
        if (count_business)
        {
            dbg_usart2_tx_bytes++;
        }
    }

    /* TXE 只表示 DR 已空，最后继续等待物理发送完成。 */
    dbg_usart2_tx_current_byte_index = 9u;
    tx_wait_start = millis();
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
        if ((millis() - tx_wait_start) >= USART2_TX_WAIT_TIMEOUT_MS)
        {
            Protocol_RecordUsart2TxTimeout(2u);
            dbg_usart2_tx_current_byte_index = 0xFFu;
            return;
        }
    }
    dbg_usart2_tx_current_byte_index = 0xFFu;

    if (count_business)
    {
        dbg_usart2_tx_frames_complete++;
        dbg_usart2_tx_last_complete_ms = millis();
        /* 正常完成不得清除之前持久化的 timeout 证据。 */
        if (source == TX_SOURCE_ACK)
        {
            dbg_tx_ack_complete_count[cmd]++;
        }
        else
        {
            dbg_tx_event_complete_count[cmd]++;
        }
    }
}

/**
 * 发送命令 ACK 或查询回复。
 */
void send_frame(uint8_t cmd, uint8_t *data)
{
    send_frame_internal(cmd, data, TX_SOURCE_ACK);
}

/**
 * 发送周期或状态主动上报。
 */
void send_frame_event(uint8_t cmd, uint8_t *data)
{
    send_frame_internal(cmd, data, TX_SOURCE_EVENT);
}

#if 0
static void send_frame_debug(uint8_t cmd, uint8_t *data)
{
    send_frame_internal(cmd, data, TX_SOURCE_DEBUG);
}
#endif
void send_frame2(uint8_t cmd, uint8_t *data)
{
    uint8_t frame[10];
    uint8_t checksum;
    int i;
    int j;
    frame[0] = 0xAA;
    frame[1] = cmd;
    for (i = 0; i < 6; i++)
    {
        frame[2 + i] = data[i];
    }

    checksum = 0;
    for (i = 0; i < 8; i++)
    {
        checksum += frame[i];
    }
    frame[8] = 0x55;
    frame[9] = checksum;
    for (j = 0; j < 10; j++)
    {
        USART_SendByte(USART1, frame[j]);
    }
}
