#include "robot_arm_protocol.h"
#include "robot_arm.h"

typedef struct
{
    uint8_t valid;
    uint8_t request_cmd;
    uint8_t axis;
    uint16_t seq;
} RobotArmProtocolActive_t;

#define ROBOT_ARM_PROTOCOL_TX_QUEUE_SIZE 4u

typedef enum
{
    ROBOT_PROTOCOL_TX_ACK = 0,
    ROBOT_PROTOCOL_TX_STATUS,
    ROBOT_PROTOCOL_TX_EVENT
} RobotArmProtocolTxKind_t;

typedef struct
{
    uint8_t raw[PROTOCOL_V2_FRAME_SIZE];
    uint8_t kind;
    uint8_t request_cmd;
    uint16_t seq;
} RobotArmProtocolTxEntry_t;

typedef enum
{
    ROBOT_TERMINAL_NONE = 0,
    ROBOT_TERMINAL_PRODUCED,
    ROBOT_TERMINAL_QUEUED
} RobotArmProtocolTerminalState_t;

typedef struct
{
    RobotArmProtocolTerminalState_t state;
    uint8_t event_type;
    uint8_t result;
} RobotArmProtocolTerminal_t;

typedef struct
{
    uint8_t valid;
    uint16_t seq;
} RobotArmProtocolPendingStopAck_t;

static RobotArmProtocolTx_t s_tx_callback;
static RobotArmProtocolActive_t s_active;
static RobotArmProtocolTerminal_t s_terminal;
static RobotArmProtocolPendingStopAck_t s_pending_stop_ack;
static RobotArmProtocolPendingStopAck_t s_pending_active_ack;
static RobotArmProtocolTxEntry_t s_tx_queue[ROBOT_ARM_PROTOCOL_TX_QUEUE_SIZE];
static uint8_t s_tx_head;
static uint8_t s_tx_tail;
static uint8_t s_tx_count;
static RobotArmProtocolStats_t s_stats;

static void RobotArmProtocol_ClearData(uint8_t *data)
{
    uint8_t index;
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++)
    {
        data[index] = 0u;
    }
}

static void RobotArmProtocol_FlushTx(void)
{
    RobotArmProtocolTxEntry_t *entry;
    if (s_tx_callback == 0)
    {
        return;
    }
    while (s_tx_count > 0u)
    {
        entry = &s_tx_queue[s_tx_head];
        if (!s_tx_callback(entry->raw, PROTOCOL_V2_FRAME_SIZE))
        {
            return;
        }
        /* 当前回调是阻塞发送；返回成功时 24B 已全部完成 UART 发送。 */
        if (entry->kind == ROBOT_PROTOCOL_TX_STATUS)
        {
            s_stats.status_tx_submit_count++;
            s_stats.status_tx_done_count++;
        }
        s_stats.tx_consumed_count++;
        if ((entry->kind == ROBOT_PROTOCOL_TX_EVENT) &&
            s_active.valid &&
            (s_terminal.state == ROBOT_TERMINAL_QUEUED) &&
            (entry->seq == s_active.seq) &&
            (entry->request_cmd == s_active.request_cmd))
        {
            /* EVENT 被发送端消费后，原任务 SEQ/CMD 才结束生命周期。 */
            s_stats.event_consumed_count++;
            s_terminal.state = ROBOT_TERMINAL_NONE;
            s_active.valid = 0u;
        }
        s_tx_head = (uint8_t)((s_tx_head + 1u) %
                              ROBOT_ARM_PROTOCOL_TX_QUEUE_SIZE);
        s_tx_count--;
    }
}

static uint8_t RobotArmProtocol_Queue(uint8_t cmd,
                                      uint16_t seq,
                                      const uint8_t *data,
                                      RobotArmProtocolTxKind_t kind,
                                      uint8_t request_cmd)
{
    ProtocolV2Frame_t frame;
    uint8_t index;
    RobotArmProtocol_FlushTx();
    if (s_tx_count >= ROBOT_ARM_PROTOCOL_TX_QUEUE_SIZE)
    {
        s_stats.tx_overflow_count++;
        if (kind == ROBOT_PROTOCOL_TX_ACK) s_stats.ack_overflow_count++;
        if (kind == ROBOT_PROTOCOL_TX_STATUS) s_stats.status_overflow_count++;
        return 0u;
    }
    frame.cmd = cmd;
    frame.seq = seq;
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++)
    {
        frame.data[index] = data[index];
    }
    ProtocolV2_Encode(&frame, s_tx_queue[s_tx_tail].raw);
    s_tx_queue[s_tx_tail].kind = (uint8_t)kind;
    s_tx_queue[s_tx_tail].request_cmd = request_cmd;
    s_tx_queue[s_tx_tail].seq = seq;
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) %
                          ROBOT_ARM_PROTOCOL_TX_QUEUE_SIZE);
    s_tx_count++;
    s_stats.tx_queued_count++;
    return 1u;
}

static uint8_t RobotArmProtocol_SendAck(uint8_t request_cmd,
                                        uint16_t seq,
                                        RobotArmProtocolAck_t ack,
                                        uint8_t result)
{
    uint8_t data[PROTOCOL_V2_DATA_SIZE];
    RobotArmProtocol_ClearData(data);
    data[0] = request_cmd;
    data[1] = (uint8_t)ack;
    data[2] = result;
    if (!RobotArmProtocol_Queue(ROBOT_ARM_CMD_ACK, seq, data,
                                ROBOT_PROTOCOL_TX_ACK, request_cmd))
    {
        return 0u;
    }
    RobotArmProtocol_FlushTx();
    return 1u;
}

static uint8_t RobotArmProtocol_EventFromReason(RobotMoveEndReason_t reason)
{
    switch (reason)
    {
    case ROBOT_MOVE_END_COMPLETED:
        return ROBOT_ARM_EVENT_COMPLETED;
    case ROBOT_MOVE_END_STOPPED:
        return ROBOT_ARM_EVENT_STOPPED;
    case ROBOT_MOVE_END_LIMIT:
        return ROBOT_ARM_EVENT_LIMIT;
    case ROBOT_MOVE_END_INTERLOCK:
        return ROBOT_ARM_EVENT_INTERLOCK;
    case ROBOT_MOVE_END_TIMEOUT:
        return ROBOT_ARM_EVENT_TIMEOUT;
    case ROBOT_MOVE_END_SENSOR:
        return ROBOT_ARM_EVENT_SENSOR_ERROR;
    default:
        return ROBOT_ARM_EVENT_DRIVER_ERROR;
    }
}

static uint8_t RobotArmProtocol_IsHomeCommand(uint8_t cmd)
{
    return ((cmd == ROBOT_ARM_CMD_HOME) ||
            (cmd == ROBOT_ARM_CMD_HOME_AXIS)) ? 1u : 0u;
}

static uint8_t RobotArmProtocol_QueueEvent(uint8_t request_cmd,
                                           uint16_t seq,
                                           uint8_t event_type,
                                           uint8_t result,
                                           uint8_t axis)
{
    uint8_t data[PROTOCOL_V2_DATA_SIZE];
    RobotArmProtocol_ClearData(data);
    data[0] = request_cmd;
    data[1] = event_type;
    data[2] = result;
    data[3] = axis;
    return RobotArmProtocol_Queue(ROBOT_ARM_CMD_EVENT, seq, data,
                                  ROBOT_PROTOCOL_TX_EVENT, request_cmd);
}

static void RobotArmProtocol_BindActive(const ProtocolV2Frame_t *request,
                                        uint8_t axis)
{
    s_active.valid = 1u;
    s_active.request_cmd = request->cmd;
    s_active.seq = request->seq;
    s_active.axis = axis;
    s_terminal.state = ROBOT_TERMINAL_NONE;
}

static void RobotArmProtocol_ProduceTerminal(uint8_t event_type,
                                             uint8_t result)
{
    if (!s_active.valid || (s_terminal.state != ROBOT_TERMINAL_NONE))
    {
        return;
    }
    s_terminal.event_type = event_type;
    s_terminal.result = result;
    s_terminal.state = ROBOT_TERMINAL_PRODUCED;
    s_stats.event_produced_count++;
}

static void RobotArmProtocol_TryQueueTerminal(void)
{
    if (!s_active.valid ||
        (s_terminal.state != ROBOT_TERMINAL_PRODUCED) ||
        s_pending_stop_ack.valid ||
        s_pending_active_ack.valid)
    {
        return;
    }
    if (!RobotArmProtocol_QueueEvent(s_active.request_cmd, s_active.seq,
                                     s_terminal.event_type,
                                     s_terminal.result,
                                     s_active.axis))
    {
        s_stats.event_retry_count++;
        return;
    }
    /* 先标记已可靠入队，再允许发送回调消费，防止同步回调丢失生命周期。 */
    s_terminal.state = ROBOT_TERMINAL_QUEUED;
    s_stats.event_queued_count++;
    RobotArmProtocol_FlushTx();
}

static void RobotArmProtocol_RetryStopAck(void)
{
    if (!s_pending_stop_ack.valid)
    {
        return;
    }
    if (RobotArmProtocol_SendAck(ROBOT_ARM_CMD_STOP,
                                 s_pending_stop_ack.seq,
                                 ROBOT_ARM_ACK_ACCEPTED,
                                 ROBOT_ARM_OK))
    {
        s_pending_stop_ack.valid = 0u;
    }
}

static void RobotArmProtocol_RetryActiveAck(void)
{
    if (!s_pending_active_ack.valid || !s_active.valid)
    {
        return;
    }
    if (RobotArmProtocol_SendAck(s_active.request_cmd,
                                 s_pending_active_ack.seq,
                                 ROBOT_ARM_ACK_ACCEPTED,
                                 ROBOT_ARM_OK))
    {
        s_pending_active_ack.valid = 0u;
    }
}

static RobotArmResult_t RobotArmProtocol_MoveAxisAbsolute(uint8_t axis,
                                                          int32_t target,
                                                          uint32_t speed)
{
    switch (axis)
    {
    case 0u: return RobotArm_MoveX(target, speed);
    case 1u: return RobotArm_MoveY(target, speed);
    case 2u: return RobotArm_MoveZ(target, speed);
    default: return ROBOT_ARM_ERR_NOT_SUPPORTED;
    }
}

static RobotArmResult_t RobotArmProtocol_MoveAxisRelative(uint8_t axis,
                                                          int32_t delta,
                                                          uint32_t speed)
{
    switch (axis)
    {
    case 0u: return RobotArm_MoveXRelative(delta, speed);
    case 1u: return RobotArm_MoveYRelative(delta, speed);
    case 2u: return RobotArm_MoveZRelative(delta, speed);
    default: return ROBOT_ARM_ERR_NOT_SUPPORTED;
    }
}

static void RobotArmProtocol_SendStatus(const ProtocolV2Frame_t *request)
{
    RobotArmStatus_t status;
    uint8_t data[PROTOCOL_V2_DATA_SIZE];
    uint8_t page = request->data[0];
    /* 每个进入 STATUS 分支的请求都留存计数，便于与构帧和发送完成数对比。 */
    s_stats.status_request_count++;
    RobotArmProtocol_ClearData(data);
    RobotArm_GetStatus(&status);

    if (page == 0u)
    {
        data[0] = 0u;
        data[1] = (uint8_t)status.arm_state;
        data[2] = (uint8_t)status.operation;
        data[3] = (uint8_t)status.error_code;
        data[4] = (uint8_t)status.x_state;
        data[5] = (uint8_t)status.y_state;
        data[6] = (uint8_t)status.z_state;
        data[7] = (uint8_t)((status.x_homed ? 1u : 0u) |
                            (status.y_homed ? 2u : 0u) |
                            (status.z_homed ? 4u : 0u));
        data[8] = (uint8_t)((status.x_valid ? 1u : 0u) |
                            (status.y_valid ? 2u : 0u) |
                            (status.z_valid ? 4u : 0u));
        data[9] = (uint8_t)((status.s1_x_home ? 1u : 0u) |
                            (status.s2_y_home ? 2u : 0u) |
                            (status.s3_z_home ? 4u : 0u) |
                            (status.s4_z_lower_limit ? 8u : 0u));
        data[10] = (uint8_t)status.last_move_end_reason;
        data[11] = (uint8_t)status.home_state;
        data[12] = (uint8_t)status.move_to_state;
        data[13] = (uint8_t)status.safe_move_state;
    }
    else if (page == 1u)
    {
        ProtocolV2_WriteI32LE(&data[0], status.x);
        ProtocolV2_WriteI32LE(&data[4], status.y);
        ProtocolV2_WriteI32LE(&data[8], status.z);
        data[12] = 1u;
    }
    else if (page == 2u)
    {
        ProtocolV2_WriteI32LE(&data[0], status.target_x);
        ProtocolV2_WriteI32LE(&data[4], status.target_y);
        ProtocolV2_WriteI32LE(&data[8], status.target_z);
        data[12] = 2u;
    }
    else
    {
        RobotArmProtocol_SendAck(request->cmd, request->seq,
                                 ROBOT_ARM_ACK_REJECTED,
                                 ROBOT_PROTOCOL_ERR_BAD_PAGE);
        return;
    }
    /* page0/page1/page2 均已填充完 DATA，随后统一编码为固定 24B 帧。 */
    s_stats.status_build_count++;
    if (RobotArmProtocol_Queue(ROBOT_ARM_CMD_STATUS_RSP, request->seq, data,
                               ROBOT_PROTOCOL_TX_STATUS, request->cmd))
    {
        RobotArmProtocol_FlushTx();
    }
    else
    {
        /* 队列满时整帧拒绝，绝不向发送层提交部分 STATUS_RSP。 */
        s_stats.status_tx_reject_count++;
    }
}

/** 初始化 RobotArm V2 命令层及发送回调。 */
void RobotArmProtocol_Init(RobotArmProtocolTx_t tx_callback)
{
    s_tx_callback = tx_callback;
    s_active.valid = 0u;
    s_active.request_cmd = 0u;
    s_active.axis = 0xFFu;
    s_active.seq = 0u;
    s_terminal.state = ROBOT_TERMINAL_NONE;
    s_terminal.event_type = 0u;
    s_terminal.result = 0u;
    s_pending_stop_ack.valid = 0u;
    s_pending_stop_ack.seq = 0u;
    s_pending_active_ack.valid = 0u;
    s_pending_active_ack.seq = 0u;
    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_tx_count = 0u;
    /* 裸机工程不依赖 C 运行库，逐项清零可避免编译器为结构体赋值生成 memset。 */
    s_stats.tx_queued_count = 0u;
    s_stats.tx_consumed_count = 0u;
    s_stats.tx_overflow_count = 0u;
    s_stats.ack_overflow_count = 0u;
    s_stats.status_overflow_count = 0u;
    s_stats.status_request_count = 0u;
    s_stats.status_build_count = 0u;
    s_stats.status_tx_submit_count = 0u;
    s_stats.status_tx_done_count = 0u;
    s_stats.status_tx_reject_count = 0u;
    s_stats.event_produced_count = 0u;
    s_stats.event_queued_count = 0u;
    s_stats.event_consumed_count = 0u;
    s_stats.event_retry_count = 0u;
}

/** 分发一帧已经通过 CRC 校验的 RobotArm V2 请求。 */
void RobotArmProtocol_HandleFrame(const ProtocolV2Frame_t *request)
{
    RobotArmResult_t result;
    uint8_t axis = 0xFFu;
    uint8_t ack_queued;
    int32_t value;
    uint32_t speed;

    if (request == 0)
    {
        return;
    }
    if (request->cmd == ROBOT_ARM_CMD_STATUS)
    {
        RobotArmProtocol_SendStatus(request);
        return;
    }
    if (request->cmd == ROBOT_ARM_CMD_STOP)
    {
        RobotArm_Stop();
        ack_queued = RobotArmProtocol_SendAck(
            request->cmd, request->seq,
            ROBOT_ARM_ACK_ACCEPTED, ROBOT_ARM_OK);
        if (!ack_queued)
        {
            s_pending_stop_ack.valid = 1u;
            s_pending_stop_ack.seq = request->seq;
        }
        /* 已经产生的终态拥有优先权，STOP 不得再制造第二个 STOPPED。 */
        if (s_active.valid &&
            (s_terminal.state == ROBOT_TERMINAL_NONE))
        {
            RobotArmProtocol_ProduceTerminal(ROBOT_ARM_EVENT_STOPPED,
                                             ROBOT_ARM_ERR_STOPPED);
        }
        if (ack_queued)
        {
            RobotArmProtocol_TryQueueTerminal();
        }
        return;
    }
    if (request->cmd == ROBOT_ARM_CMD_CLEAR_ERROR)
    {
        result = RobotArm_ClearError();
        RobotArmProtocol_SendAck(request->cmd, request->seq,
                                 (result == ROBOT_ARM_OK) ?
                                     ROBOT_ARM_ACK_ACCEPTED : ROBOT_ARM_ACK_REJECTED,
                                 (uint8_t)result);
        return;
    }

    if (s_active.valid)
    {
        /* 旧任务的最终 EVENT 被可靠消费前，不允许新动作覆盖其原始 SEQ/CMD。 */
        RobotArmProtocol_SendAck(request->cmd, request->seq,
                                 ROBOT_ARM_ACK_REJECTED,
                                 ROBOT_ARM_ERR_BUSY);
        return;
    }

    switch (request->cmd)
    {
    case ROBOT_ARM_CMD_HOME:
        result = RobotArm_Home();
        break;
    case ROBOT_ARM_CMD_HOME_AXIS:
        axis = request->data[0];
        if (axis > 2u)
        {
            RobotArmProtocol_SendAck(request->cmd, request->seq,
                                     ROBOT_ARM_ACK_REJECTED,
                                     ROBOT_PROTOCOL_ERR_BAD_AXIS);
            return;
        }
        result = RobotArm_HomeAxis((RobotAxisId_t)axis);
        break;
    case ROBOT_ARM_CMD_MOVE_AXIS_ABS:
        axis = request->data[0];
        if (axis > 2u)
        {
            RobotArmProtocol_SendAck(request->cmd, request->seq,
                                     ROBOT_ARM_ACK_REJECTED,
                                     ROBOT_PROTOCOL_ERR_BAD_AXIS);
            return;
        }
        value = ProtocolV2_ReadI32LE(&request->data[1]);
        speed = ProtocolV2_ReadU32LE(&request->data[5]);
        result = RobotArmProtocol_MoveAxisAbsolute(axis, value, speed);
        break;
    case ROBOT_ARM_CMD_MOVE_AXIS_REL:
        axis = request->data[0];
        if (axis > 2u)
        {
            RobotArmProtocol_SendAck(request->cmd, request->seq,
                                     ROBOT_ARM_ACK_REJECTED,
                                     ROBOT_PROTOCOL_ERR_BAD_AXIS);
            return;
        }
        value = ProtocolV2_ReadI32LE(&request->data[1]);
        speed = ProtocolV2_ReadU32LE(&request->data[5]);
        result = RobotArmProtocol_MoveAxisRelative(axis, value, speed);
        break;
    case ROBOT_ARM_CMD_MOVE_TO:
        result = RobotArm_MoveTo(ProtocolV2_ReadI32LE(&request->data[0]),
                                 ProtocolV2_ReadI32LE(&request->data[4]),
                                 ProtocolV2_ReadI32LE(&request->data[8]));
        break;
    case ROBOT_ARM_CMD_MOVE_TO_SAFE:
        result = RobotArm_MoveToSafe(ProtocolV2_ReadI32LE(&request->data[0]),
                                     ProtocolV2_ReadI32LE(&request->data[4]),
                                     ProtocolV2_ReadI32LE(&request->data[8]));
        break;
    default:
        RobotArmProtocol_SendAck(request->cmd, request->seq,
                                 ROBOT_ARM_ACK_REJECTED,
                                 ROBOT_PROTOCOL_ERR_BAD_CMD);
        return;
    }

    ack_queued = RobotArmProtocol_SendAck(
        request->cmd, request->seq,
        (result == ROBOT_ARM_OK) ?
            ROBOT_ARM_ACK_ACCEPTED : ROBOT_ARM_ACK_REJECTED,
        (uint8_t)result);
    if (result == ROBOT_ARM_OK)
    {
        RobotArmProtocol_BindActive(request, axis);
        if (!ack_queued)
        {
            s_pending_active_ack.valid = 1u;
            s_pending_active_ack.seq = request->seq;
        }
        if (RobotArm_IsBusy())
        {
            return;
        }
        /* 零位移任务同样先可靠保存 ACK，再产生原 SEQ 的完成 EVENT。 */
        RobotArmProtocol_ProduceTerminal(
            RobotArmProtocol_IsHomeCommand(request->cmd) ?
                ROBOT_ARM_EVENT_HOME_COMPLETED : ROBOT_ARM_EVENT_COMPLETED,
            ROBOT_ARM_OK);
        if (ack_queued)
        {
            RobotArmProtocol_TryQueueTerminal();
        }
    }
}

/** 轮询异步 RobotArm 操作并发送真正的完成或失败事件。 */
void RobotArmProtocol_Task(void)
{
    RobotArmStatus_t status;
    uint8_t event_type;
    uint8_t result;

    RobotArmProtocol_FlushTx();
    RobotArmProtocol_RetryStopAck();
    RobotArmProtocol_RetryActiveAck();
    RobotArmProtocol_TryQueueTerminal();
    if (!s_active.valid ||
        (s_terminal.state != ROBOT_TERMINAL_NONE) ||
        RobotArm_IsBusy())
    {
        return;
    }
    RobotArm_GetStatus(&status);
    if (status.arm_state == ROBOT_ARM_ERROR)
    {
        result = (uint8_t)status.error_code;
        event_type = RobotArmProtocol_IsHomeCommand(s_active.request_cmd) ?
                         ROBOT_ARM_EVENT_HOME_FAILED :
                         RobotArmProtocol_EventFromReason(status.last_move_end_reason);
    }
    else if ((status.arm_state == ROBOT_ARM_IDLE) &&
             (status.last_move_end_reason == ROBOT_MOVE_END_COMPLETED))
    {
        result = ROBOT_ARM_OK;
        event_type = RobotArmProtocol_IsHomeCommand(s_active.request_cmd) ?
                         ROBOT_ARM_EVENT_HOME_COMPLETED :
                         ROBOT_ARM_EVENT_COMPLETED;
    }
    else
    {
        return;
    }
    RobotArmProtocol_ProduceTerminal(event_type, result);
    RobotArmProtocol_TryQueueTerminal();
}

/** 查询发送队列是否至少能可靠保存一个命令可能产生的 ACK 和 EVENT。 */
uint8_t RobotArmProtocol_CanAcceptRequest(void)
{
    RobotArmProtocol_FlushTx();
    if (s_pending_stop_ack.valid || s_pending_active_ack.valid)
    {
        return 0u;
    }
    return (s_tx_count <= (ROBOT_ARM_PROTOCOL_TX_QUEUE_SIZE - 2u)) ? 1u : 0u;
}

/** 获取发送队列和异步事件生命周期统计。 */
void RobotArmProtocol_GetStats(RobotArmProtocolStats_t *stats)
{
    if (stats != 0)
    {
        *stats = s_stats;
    }
}
