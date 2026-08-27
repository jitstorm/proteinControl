#include <stdint.h>
#include "robot_arm_protocol.h"
#include "robot_arm.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_failure == 0)) s_failure = __LINE__; } while (0)

static int s_failure;
static ProtocolV2Frame_t s_tx[48];
static uint8_t s_tx_count;
static uint8_t s_tx_available = 1u;
static RobotArmResult_t s_next_result;
static uint8_t s_busy;
static uint8_t s_accept_busy = 1u;
static RobotArmStatus_t s_status;
static uint8_t s_last_call;
static uint8_t s_last_axis;
static int32_t s_last_x;
static int32_t s_last_y;
static int32_t s_last_z;
static uint32_t s_last_speed;

/** 为无 C 运行库的测试可执行文件提供最小内存复制实现。 */
void *memcpy(void *destination, const void *source, __SIZE_TYPE__ count)
{
    unsigned char *to = (unsigned char *)destination;
    const unsigned char *from = (const unsigned char *)source;
    __SIZE_TYPE__ index;
    for (index = 0u; index < count; index++) to[index] = from[index];
    return destination;
}

static void TestClearFrame(ProtocolV2Frame_t *frame, uint8_t cmd, uint16_t seq)
{
    uint8_t index;
    frame->cmd = cmd;
    frame->seq = seq;
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++) frame->data[index] = 0u;
}

static uint8_t TestTx(const uint8_t *raw, uint8_t length)
{
    if (!s_tx_available)
    {
        return 0u;
    }
    TEST_CHECK(length == PROTOCOL_V2_FRAME_SIZE);
    TEST_CHECK(s_tx_count < 48u);
    if (s_tx_count < 48u)
    {
        TEST_CHECK(ProtocolV2_Decode(raw, &s_tx[s_tx_count]) == 1u);
        s_tx_count++;
    }
    return 1u;
}

static void TestSetAsyncResult(RobotMoveEndReason_t reason,
                               RobotArmResult_t error)
{
    s_busy = 0u;
    s_status.arm_state = (error == ROBOT_ARM_OK) ? ROBOT_ARM_IDLE : ROBOT_ARM_ERROR;
    s_status.last_move_end_reason = reason;
    s_status.error_code = error;
}

static RobotArmResult_t TestAccept(uint8_t call)
{
    s_last_call = call;
    if ((s_next_result == ROBOT_ARM_OK) && s_accept_busy) s_busy = 1u;
    return s_next_result;
}

/** 模拟 RobotArm HomeAll 接口。 */
RobotArmResult_t RobotArm_Home(void) { return TestAccept(1u); }
/** 模拟 RobotArm 单轴 Home 接口。 */
RobotArmResult_t RobotArm_HomeAxis(RobotAxisId_t axis)
{
    s_last_axis = (uint8_t)axis;
    return TestAccept(2u);
}
/** 模拟 X 轴绝对运动接口。 */
RobotArmResult_t RobotArm_MoveX(int32_t target, uint32_t speed)
{
    s_last_axis = 0u; s_last_x = target; s_last_speed = speed; return TestAccept(3u);
}
/** 模拟 Y 轴绝对运动接口。 */
RobotArmResult_t RobotArm_MoveY(int32_t target, uint32_t speed)
{
    s_last_axis = 1u; s_last_x = target; s_last_speed = speed; return TestAccept(3u);
}
/** 模拟 Z 轴绝对运动接口。 */
RobotArmResult_t RobotArm_MoveZ(int32_t target, uint32_t speed)
{
    s_last_axis = 2u; s_last_x = target; s_last_speed = speed; return TestAccept(3u);
}
/** 模拟 X 轴相对运动接口。 */
RobotArmResult_t RobotArm_MoveXRelative(int32_t delta, uint32_t speed)
{
    s_last_axis = 0u; s_last_x = delta; s_last_speed = speed; return TestAccept(4u);
}
/** 模拟 Y 轴相对运动接口。 */
RobotArmResult_t RobotArm_MoveYRelative(int32_t delta, uint32_t speed)
{
    s_last_axis = 1u; s_last_x = delta; s_last_speed = speed; return TestAccept(4u);
}
/** 模拟 Z 轴相对运动接口。 */
RobotArmResult_t RobotArm_MoveZRelative(int32_t delta, uint32_t speed)
{
    s_last_axis = 2u; s_last_x = delta; s_last_speed = speed; return TestAccept(4u);
}
/** 模拟带临时速度的普通 MoveTo 接口。 */
RobotArmResult_t RobotArm_MoveToWithSpeed(int32_t x, int32_t y, int32_t z,
                                          uint32_t speed)
{
    s_last_x = x; s_last_y = y; s_last_z = z; s_last_speed = speed;
    return TestAccept(5u);
}
/** 保留旧 MoveTo 测试替身，确保历史调用仍走默认速度。 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z)
{
    return RobotArm_MoveToWithSpeed(x, y, z, 0u);
}
/** 模拟安全 MoveTo 接口。 */
RobotArmResult_t RobotArm_MoveToSafe(int32_t x, int32_t y, int32_t z)
{
    s_last_x = x; s_last_y = y; s_last_z = z; return TestAccept(6u);
}
/** 模拟 Stop，并结束当前异步任务。 */
void RobotArm_Stop(void)
{
    s_last_call = 7u;
    TestSetAsyncResult(ROBOT_MOVE_END_STOPPED, ROBOT_ARM_ERR_STOPPED);
}
/** 模拟 ClearError 接口。 */
RobotArmResult_t RobotArm_ClearError(void)
{
    s_last_call = 8u;
    return s_next_result;
}
/** 模拟 RobotArm Busy 查询。 */
uint8_t RobotArm_IsBusy(void) { return s_busy; }
/** 返回测试构造的 RobotArm 状态。 */
void RobotArm_GetStatus(RobotArmStatus_t *status) { *status = s_status; }

/** 运行 RobotArm V2 命令、ACK、EVENT 和 STATUS 分页测试。 */
int main(void)
{
    ProtocolV2Frame_t request;
    RobotArmProtocolStats_t protocol_stats;
    uint8_t before;
    uint8_t axis_index;
    uint8_t loop_index;

    RobotArmProtocol_Init(TestTx);
    s_next_result = ROBOT_ARM_OK;
    s_status.arm_state = ROBOT_ARM_IDLE;

    TestClearFrame(&request, ROBOT_ARM_CMD_HOME, 1u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 1u && s_tx[0].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[0].seq == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_ACCEPTED);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[1].cmd == ROBOT_ARM_CMD_EVENT);
    TEST_CHECK(s_tx[1].data[1] == ROBOT_ARM_EVENT_HOME_COMPLETED);

    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 2u);
    request.data[0] = 3u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[2].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[2].data[2] == ROBOT_PROTOCOL_ERR_BAD_AXIS);
    request.data[0] = 2u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 2u && s_last_axis == 2u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();
    for (axis_index = 0u; axis_index < 2u; axis_index++)
    {
        request.seq++;
        request.data[0] = axis_index;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_last_call == 2u && s_last_axis == axis_index);
        TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
        RobotArmProtocol_Task();
    }

    /* Home 异步失败统一使用 HOME_FAILED，并保留具体 RobotArm 错误。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME, 12u);
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_SENSOR, ROBOT_ARM_ERR_SENSOR);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_EVENT_HOME_FAILED);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_SENSOR);

    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_AXIS_ABS, 3u);
    request.data[0] = 1u;
    ProtocolV2_WriteI32LE(&request.data[1], -1234);
    ProtocolV2_WriteU32LE(&request.data[5], 0u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 3u && s_last_axis == 1u);
    TEST_CHECK(s_last_x == -1234 && s_last_speed == 0u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_AXIS_REL, 4u);
    request.data[0] = 0u;
    ProtocolV2_WriteI32LE(&request.data[1], 25);
    ProtocolV2_WriteU32LE(&request.data[5], 500u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 4u && s_last_x == 25 && s_last_speed == 500u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 5u);
    ProtocolV2_WriteI32LE(&request.data[0], 10);
    ProtocolV2_WriteI32LE(&request.data[4], 20);
    ProtocolV2_WriteI32LE(&request.data[8], 30);
    /* 未设置速度的旧帧仍以 D12-D13=00 00 进入既有默认速度路径。 */
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 5u && s_last_x == 10 && s_last_y == 20 &&
               s_last_z == 30 && s_last_speed == 0u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 51u);
    ProtocolV2_WriteI32LE(&request.data[0], 11);
    ProtocolV2_WriteI32LE(&request.data[4], 22);
    ProtocolV2_WriteI32LE(&request.data[8], 33);
    ProtocolV2_WriteU16LE(&request.data[12], 5000u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 5u && s_last_x == 11 && s_last_y == 22 &&
               s_last_z == 33 && s_last_speed == 5000u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    s_next_result = ROBOT_ARM_ERR_CONFIG;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO_SAFE, 6u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_CONFIG);

    s_next_result = ROBOT_ARM_OK;
    request.seq = 105u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 6u);
    TestClearFrame(&request, ROBOT_ARM_CMD_STOP, 106u);
    before = s_tx_count;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[before].cmd == ROBOT_ARM_CMD_ACK && s_tx[before].seq == 106u);
    TEST_CHECK(s_tx[before + 1u].cmd == ROBOT_ARM_CMD_EVENT);
    TEST_CHECK(s_tx[before + 1u].seq == 105u);
    TEST_CHECK(s_tx[before + 1u].data[1] == ROBOT_ARM_EVENT_STOPPED);

    /* STOPPED EVENT 发送完成后协议 active 必须释放，下一条 MOVE_TO 不得被 BUSY 拒绝。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 107u);
    ProtocolV2_WriteI32LE(&request.data[0], 1);
    ProtocolV2_WriteI32LE(&request.data[4], 2);
    ProtocolV2_WriteI32LE(&request.data[8], 3);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 5u);
    TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_ACK_ACCEPTED);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    s_next_result = ROBOT_ARM_OK;
    TestClearFrame(&request, ROBOT_ARM_CMD_CLEAR_ERROR, 7u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 8u && s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_ACK_ACCEPTED);

    s_status.x = 0; s_status.y = 0; s_status.z = 0;
    s_status.target_x = 11; s_status.target_y = -22; s_status.target_z = 33;
    s_status.x_homed = 1u; s_status.z_homed = 1u;
    s_status.x_valid = 1u; s_status.y_valid = 1u;
    s_status.s1_x_home = 1u; s_status.s4_z_lower_limit = 1u;
    TestClearFrame(&request, ROBOT_ARM_CMD_STATUS, 8u);
    request.data[0] = 0u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_STATUS_RSP);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[7] == 5u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[8] == 3u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[9] == 9u);

    /* 零坐标也必须占满三个 int32 字段，页标识只能写入 D12。 */
    request.data[0] = 1u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[0]) == 0);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[4]) == 0);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[8]) == 0);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[12] == 1u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[13] == 0u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[14] == 0u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[15] == 0u);

    /* 正负坐标必须按 int32 小端序完整往返，不能被页标识覆盖。 */
    s_status.x = 1234; s_status.y = -5678; s_status.z = 9012;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[0]) == 1234);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[4]) == -5678);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[8]) == 9012);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[12] == 1u);

    /* page2 复用 RobotArmStatus 的正式目标快照，并验证全部 XYZ 与保留位。 */
    request.data[0] = 2u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[0]) == 11);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[4]) == -22);
    TEST_CHECK(ProtocolV2_ReadI32LE(&s_tx[s_tx_count - 1u].data[8]) == 33);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[12] == 2u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[13] == 0u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[14] == 0u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[15] == 0u);

    /* RobotArm 拒绝原因保持原值进入 ACK，不与协议错误混淆。 */
    s_next_result = ROBOT_ARM_ERR_POSITION_UNKNOWN;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 20u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_POSITION_UNKNOWN);
    s_next_result = ROBOT_ARM_ERR_BUSY;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_BUSY);
    s_next_result = ROBOT_ARM_ERR_LIMIT;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_LIMIT);

    /* 异步错误通过原请求 SEQ 的 EVENT 表达。 */
    s_next_result = ROBOT_ARM_OK;
    request.seq = 21u;
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_TIMEOUT, ROBOT_ARM_ERR_MOVE_TIMEOUT);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[s_tx_count - 1u].seq == 21u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_EVENT_TIMEOUT);
    request.seq = 22u;
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_INTERLOCK, ROBOT_ARM_ERR_INTERLOCK);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_EVENT_INTERLOCK);
    request.seq = 24u;
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_LIMIT, ROBOT_ARM_ERR_LIMIT);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_EVENT_LIMIT);
    request.seq = 25u;
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_DRIVER_ERROR, ROBOT_ARM_ERR_DRIVER);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_EVENT_DRIVER_ERROR);

    /* 零位移 API 返回 OK 但不进入 Busy 时，仍需 ACK 后立即发完成 EVENT。 */
    s_accept_busy = 0u;
    s_next_result = ROBOT_ARM_OK;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 23u);
    before = s_tx_count;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[before].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[before + 1u].cmd == ROBOT_ARM_CMD_EVENT);
    TEST_CHECK(s_tx[before + 1u].seq == 23u);
    s_accept_busy = 1u;

    TestClearFrame(&request, 0x69u, 30u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_PROTOCOL_ERR_BAD_CMD);

    /* 同一完成终态即使连续轮询 100 次，也只能生产和发送一个 EVENT。 */
    RobotArmProtocol_Init(TestTx);
    s_tx_count = 0u;
    s_tx_available = 1u;
    s_accept_busy = 1u;
    s_next_result = ROBOT_ARM_OK;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 65535u);
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();
    before = s_tx_count;
    for (loop_index = 0u; loop_index < 100u; loop_index++)
    {
        RobotArmProtocol_Task();
    }
    TEST_CHECK(s_tx_count == before);
    TEST_CHECK(s_tx_count == 2u);
    TEST_CHECK(s_tx[1].cmd == ROBOT_ARM_CMD_EVENT && s_tx[1].seq == 65535u);
    RobotArmProtocol_GetStats(&protocol_stats);
    TEST_CHECK(protocol_stats.event_produced_count == 1u);
    TEST_CHECK(protocol_stats.event_queued_count == 1u);
    TEST_CHECK(protocol_stats.event_consumed_count == 1u);

    /* 发送队列满时 EVENT 保持 produced，恢复空间后仍使用原任务 SEQ。 */
    RobotArmProtocol_Init(TestTx);
    s_tx_count = 0u;
    s_tx_available = 0u;
    s_next_result = ROBOT_ARM_OK;
    s_accept_busy = 1u;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 65535u);
    RobotArmProtocol_HandleFrame(&request); /* 队列1：ACK */
    TestClearFrame(&request, ROBOT_ARM_CMD_STATUS, 0u);
    request.data[0] = 0u;
    RobotArmProtocol_HandleFrame(&request); /* 队列2：STATUS */
    request.seq = 1u;
    RobotArmProtocol_HandleFrame(&request); /* 队列3：STATUS */
    request.seq = 2u;
    RobotArmProtocol_HandleFrame(&request); /* 队列4：STATUS */
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();
    RobotArmProtocol_GetStats(&protocol_stats);
    TEST_CHECK(protocol_stats.event_produced_count == 1u);
    TEST_CHECK(protocol_stats.event_queued_count == 0u);
    TEST_CHECK(protocol_stats.event_consumed_count == 0u);
    TEST_CHECK(protocol_stats.event_retry_count > 0u);
    TEST_CHECK(protocol_stats.tx_overflow_count > 0u);
    for (loop_index = 0u; loop_index < 100u; loop_index++) RobotArmProtocol_Task();
    RobotArmProtocol_GetStats(&protocol_stats);
    TEST_CHECK(protocol_stats.event_produced_count == 1u);
    s_tx_available = 1u;
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx_count == 5u);
    TEST_CHECK(s_tx[0].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[1].cmd == ROBOT_ARM_CMD_STATUS_RSP);
    TEST_CHECK(s_tx[2].cmd == ROBOT_ARM_CMD_STATUS_RSP);
    TEST_CHECK(s_tx[3].cmd == ROBOT_ARM_CMD_STATUS_RSP);
    TEST_CHECK(s_tx[4].cmd == ROBOT_ARM_CMD_EVENT);
    TEST_CHECK(s_tx[4].seq == 65535u);
    RobotArmProtocol_GetStats(&protocol_stats);
    TEST_CHECK(protocol_stats.event_queued_count == 1u);
    TEST_CHECK(protocol_stats.event_consumed_count == 1u);

    /* 旧终态尚未轮询时，新动作不得覆盖旧任务的 active_seq。 */
    RobotArmProtocol_Init(TestTx);
    s_tx_count = 0u;
    s_tx_available = 1u;
    s_accept_busy = 1u;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 41u);
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO_SAFE, 42u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[1].cmd == ROBOT_ARM_CMD_ACK && s_tx[1].seq == 42u);
    TEST_CHECK(s_tx[1].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[1].data[2] == ROBOT_ARM_ERR_BUSY);
    RobotArmProtocol_Task();
    TEST_CHECK(s_tx[2].cmd == ROBOT_ARM_CMD_EVENT && s_tx[2].seq == 41u);

    /* SEQ 从 65535 回绕到 0 后，新的 ACK/EVENT 必须匹配 0。 */
    s_accept_busy = 0u;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 0u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[s_tx_count - 2u].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[s_tx_count - 2u].seq == 0u);
    TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_EVENT);
    TEST_CHECK(s_tx[s_tx_count - 1u].seq == 0u);

    /* 完成状态刚出现但尚未生产 EVENT 时收到 STOP，只允许 STOPPED 一个终态。 */
    RobotArmProtocol_Init(TestTx);
    s_tx_count = 0u;
    s_tx_available = 1u;
    s_accept_busy = 1u;
    s_next_result = ROBOT_ARM_OK;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO_SAFE, 105u);
    RobotArmProtocol_HandleFrame(&request);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    TestClearFrame(&request, ROBOT_ARM_CMD_STOP, 106u);
    RobotArmProtocol_HandleFrame(&request);
    for (loop_index = 0u; loop_index < 100u; loop_index++) RobotArmProtocol_Task();
    TEST_CHECK(s_tx_count == 3u);
    TEST_CHECK(s_tx[0].cmd == ROBOT_ARM_CMD_ACK && s_tx[0].seq == 105u);
    TEST_CHECK(s_tx[1].cmd == ROBOT_ARM_CMD_ACK && s_tx[1].seq == 106u);
    TEST_CHECK(s_tx[2].cmd == ROBOT_ARM_CMD_EVENT && s_tx[2].seq == 105u);
    TEST_CHECK(s_tx[2].data[1] == ROBOT_ARM_EVENT_STOPPED);
    RobotArmProtocol_GetStats(&protocol_stats);
    TEST_CHECK(protocol_stats.event_produced_count == 1u);
    TEST_CHECK(protocol_stats.event_consumed_count == 1u);
    return s_failure;
}
