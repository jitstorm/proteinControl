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
static uint16_t s_last_speed;
static uint16_t s_last_acceleration;
static uint16_t s_last_x_speed;
static uint16_t s_last_y_speed;
static uint16_t s_last_z_speed;
static RobotMoveMotionMode_t s_last_motion_mode;
static RobotArmPhase_t s_last_phase;
static uint8_t s_phase_start_count;

/** 为无 C 运行库的测试可执行文件提供最小内存复制实现。 */
void *memcpy(void *destination, const void *source, __SIZE_TYPE__ count)
{
    unsigned char *to = (unsigned char *)destination;
    const unsigned char *from = (const unsigned char *)source;
    __SIZE_TYPE__ index;
    for (index = 0u; index < count; index++) to[index] = from[index];
    return destination;
}

/** 为协议状态结构清零提供最小实现，避免独立宿主测试依赖 C 运行库。 */
void *memset(void *destination, int value, __SIZE_TYPE__ count)
{
    unsigned char *bytes = (unsigned char *)destination;
    __SIZE_TYPE__ index;
    for (index = 0u; index < count; index++) bytes[index] = (unsigned char)value;
    return destination;
}

/** 协议测试只验证帧受理和终态转发，机械状态机由夹具直接驱动。 */
void RobotArm_Task(void) {}

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
RobotArmResult_t RobotArm_HomeAxisWithSpeed(RobotAxisId_t axis, uint16_t speed)
{
    s_last_axis = (uint8_t)axis; s_last_speed = speed; return TestAccept(2u);
}
/** 模拟携带加速度的单轴 Home，并记录协议层解码结果。 */
RobotArmResult_t RobotArm_HomeAxisWithSpeedAndAcceleration(
    RobotAxisId_t axis, uint16_t speed, uint16_t acceleration)
{
    s_last_axis = (uint8_t)axis;
    s_last_speed = speed;
    s_last_acceleration = acceleration;
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
                                          uint16_t x_speed, uint16_t y_speed,
                                          uint16_t z_speed)
{
    s_last_x = x; s_last_y = y; s_last_z = z;
    s_last_x_speed = x_speed; s_last_y_speed = y_speed; s_last_z_speed = z_speed;
    return TestAccept(5u);
}
/** 模拟可选同步模式的普通 MoveTo 接口，并记录协议传入的 DATA[15]。 */
RobotArmResult_t RobotArm_MoveToWithSpeedAndMode(
    int32_t x, int32_t y, int32_t z, uint16_t x_speed, uint16_t y_speed,
    uint16_t z_speed, RobotMoveMotionMode_t motion_mode)
{
    s_last_x = x; s_last_y = y; s_last_z = z;
    s_last_x_speed = x_speed; s_last_y_speed = y_speed; s_last_z_speed = z_speed;
    s_last_motion_mode = motion_mode;
    return TestAccept(5u);
}
/** 模拟批量 Phase 的执行入口，并记录主导轴边界频率和坐标增量。 */
RobotArmResult_t RobotArm_StartPhase(const RobotArmPhase_t *phase)
{
    if (phase == 0)
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    s_last_phase = *phase;
    s_phase_start_count++;
    return TestAccept(9u);
}
/** 保留旧 MoveTo 测试替身，确保历史调用仍走默认速度。 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z)
{
    return RobotArm_MoveToWithSpeed(x, y, z, 0u, 0u, 0u);
}
/** 模拟安全 MoveTo 接口。 */
RobotArmResult_t RobotArm_MoveToSafe(int32_t x, int32_t y, int32_t z)
{
    s_last_x = x; s_last_y = y; s_last_z = z; return TestAccept(6u);
}
RobotArmResult_t RobotArm_MoveToSafeWithSpeed(int32_t x, int32_t y, int32_t z,
                                              uint16_t x_speed,
                                              uint16_t y_speed,
                                              uint16_t z_speed)
{
    s_last_x = x; s_last_y = y; s_last_z = z;
    s_last_x_speed = x_speed; s_last_y_speed = y_speed; s_last_z_speed = z_speed;
    return TestAccept(6u);
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

/**
 * 验证单轴 Home 的 V2 接收、ACK 与 page3 终态查询模型。
 *
 * MCU 不主动发送 0x71；测试必须确认请求 ACK 与最终结果读取分离，且 page3
 * 保留原请求 CMD/SEQ，供 Android 将终态关联回单轴 Home 请求。
 */
int main(void)
{
    ProtocolV2Frame_t request;
    uint8_t axis;
    uint8_t before;

    RobotArmProtocol_Init(TestTx);
    s_next_result = ROBOT_ARM_OK;
    s_status.arm_state = ROBOT_ARM_IDLE;

    /* 非法轴必须在调用执行层前被拒绝。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 2u);
    request.data[0] = 3u;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx_count == 1u);
    TEST_CHECK(s_tx[0].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[0].data[2] == ROBOT_PROTOCOL_ERR_BAD_AXIS);

    /* homeSpeed=0 不得回退 MCU 默认值，必须在启动 Home 前以 CONFIG 拒绝。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 3u);
    request.data[0] = ROBOT_AXIS_X;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx[1].cmd == ROBOT_ARM_CMD_ACK);
    TEST_CHECK(s_tx[1].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[1].data[2] == ROBOT_ARM_ERR_CONFIG);

    for (axis = ROBOT_AXIS_X; axis < ROBOT_AXIS_COUNT; axis++)
    {
        TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS,
                       (uint16_t)(0x120u + axis));
        request.data[0] = axis;
        ProtocolV2_WriteU16LE(&request.data[1], (uint16_t)(100u + axis));
        ProtocolV2_WriteU16LE(&request.data[3], (uint16_t)(300u + axis));
        before = s_tx_count;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_last_call == 2u && s_last_axis == axis);
        TEST_CHECK(s_last_speed == (uint16_t)(100u + axis));
        TEST_CHECK(s_last_acceleration == (uint16_t)(300u + axis));
        TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));
        TEST_CHECK(s_tx[before].cmd == ROBOT_ARM_CMD_ACK);
        TEST_CHECK(s_tx[before].seq == request.seq);
        TEST_CHECK(s_tx[before].data[0] == ROBOT_ARM_CMD_HOME_AXIS);
        TEST_CHECK(s_tx[before].data[1] == ROBOT_ARM_ACK_ACCEPTED);

        /* ACK 后仅保存终态，不能自行发送 0x71 EVENT。 */
        TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
        RobotArmProtocol_Task();
        TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));

        TestClearFrame(&request, ROBOT_ARM_CMD_STATUS,
                       (uint16_t)(0x220u + axis));
        request.data[0] = 3u;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_STATUS_RSP);
        TEST_CHECK(s_tx[s_tx_count - 1u].seq == request.seq);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[0] == 3u);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[1] == 1u);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_CMD_HOME_AXIS);
        TEST_CHECK(ProtocolV2_ReadU16LE(&s_tx[s_tx_count - 1u].data[3]) ==
                   (uint16_t)(0x120u + axis));
        TEST_CHECK(s_tx[s_tx_count - 1u].data[5] == ROBOT_ARM_CMD_EVENT);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[6] == ROBOT_ARM_EVENT_HOME_COMPLETED);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[7] == ROBOT_ARM_OK);
    }

    /* 单轴和三轴运动均使用 int24 + uint16，协议层不得保留旧 int32/uint32 偏移。 */
    s_next_result = ROBOT_ARM_OK;
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_AXIS_REL, 0x310u);
    request.data[0] = ROBOT_AXIS_Y;
    request.data[1] = 0xFFu; request.data[2] = 0xFFu; request.data[3] = 0xFFu;
    ProtocolV2_WriteU16LE(&request.data[4], 65535u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 4u && s_last_axis == ROBOT_AXIS_Y);
    TEST_CHECK(s_last_x == -1 && s_last_speed == 65535u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 0x311u);
    request.data[0] = 1u; request.data[3] = 2u; request.data[6] = 3u;
    ProtocolV2_WriteU16LE(&request.data[9], 101u);
    ProtocolV2_WriteU16LE(&request.data[11], 202u);
    ProtocolV2_WriteU16LE(&request.data[13], 303u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 5u && s_last_x == 1 && s_last_y == 2 && s_last_z == 3);
    TEST_CHECK(s_last_x_speed == 101u && s_last_y_speed == 202u && s_last_z_speed == 303u);
    TEST_CHECK(s_last_motion_mode == ROBOT_MOVE_MOTION_SEQUENTIAL);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    /* DATA[15]=1 只改变普通 MOVE_TO 的内部关节启动方式，CMD 和 ACK 生命周期不变。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 0x312u);
    request.data[0] = 4u; request.data[3] = 5u; request.data[6] = 6u;
    ProtocolV2_WriteU16LE(&request.data[9], 401u);
    ProtocolV2_WriteU16LE(&request.data[11], 402u);
    ProtocolV2_WriteU16LE(&request.data[13], 403u);
    request.data[15] = ROBOT_MOVE_MOTION_XYZ_SYNC;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 5u);
    TEST_CHECK(s_last_motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();

    /* 未定义的 DATA[15] 必须在执行层前拒绝，避免未知路径变成真实机械动作。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO, 0x313u);
    ProtocolV2_WriteU16LE(&request.data[9], 100u);
    ProtocolV2_WriteU16LE(&request.data[11], 100u);
    ProtocolV2_WriteU16LE(&request.data[13], 100u);
    request.data[15] = 2u;
    before = s_tx_count;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));
    TEST_CHECK(s_tx[before].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[before].data[2] == ROBOT_ARM_ERR_CONFIG);

    /* SafeMove 的同步值必须被拒绝，不能绕过既有抬 Z 防撞路径。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_MOVE_TO_SAFE, 0x314u);
    ProtocolV2_WriteU16LE(&request.data[9], 100u);
    ProtocolV2_WriteU16LE(&request.data[11], 100u);
    ProtocolV2_WriteU16LE(&request.data[13], 100u);
    request.data[15] = ROBOT_MOVE_MOTION_XYZ_SYNC;
    before = s_tx_count;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));
    TEST_CHECK(s_tx[before].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[before].data[2] == ROBOT_ARM_ERR_CONFIG);

    /* 活动单轴 Home 未完成时，下一条动作必须得到 BUSY ACK，不能覆盖原 SEQ。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 0x330u);
    request.data[0] = ROBOT_AXIS_X;
    ProtocolV2_WriteU16LE(&request.data[1], 500u);
    RobotArmProtocol_HandleFrame(&request);
    before = s_tx_count;
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 0x331u);
    request.data[0] = ROBOT_AXIS_Y;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_axis == ROBOT_AXIS_X);
    TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));
    TEST_CHECK(s_tx[before].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[before].data[2] == ROBOT_ARM_ERR_BUSY);

    /* 执行层报告 Home 参数非法时，协议只返回拒绝 ACK，不生成 terminal。 */
    TestSetAsyncResult(ROBOT_MOVE_END_STOPPED, ROBOT_ARM_ERR_STOPPED);
    RobotArmProtocol_Task();
    s_next_result = ROBOT_ARM_ERR_CONFIG;
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 0x340u);
    request.data[0] = ROBOT_AXIS_Z;
    before = s_tx_count;
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));
    TEST_CHECK(s_tx[before].data[1] == ROBOT_ARM_ACK_REJECTED);
    TEST_CHECK(s_tx[before].data[2] == ROBOT_ARM_ERR_CONFIG);

    /* 0x39 必须一次接收全部 Phase；每轴完成后只启动下一条，不重新发旧 MOVE_TO。 */
    {
        ProtocolV2PhaseBatchFrame_t batch;
        uint8_t phase_offset;
        uint16_t clear_index;
        uint8_t phase_before;
        for (clear_index = 0u; clear_index < PROTOCOL_V2_PHASE_BATCH_MAX_PAYLOAD;
             clear_index++) batch.payload[clear_index] = 0u;
        batch.seq = 0x390u;
        batch.length = 36u;
        ProtocolV2_WriteU16LE(&batch.payload[0], 0x1234u);
        batch.payload[2] = 2u;
        phase_offset = PROTOCOL_V2_PHASE_BATCH_HEADER_SIZE;
        batch.payload[phase_offset] = 201u;
        ProtocolV2_WriteU16LE(&batch.payload[phase_offset + 9u], 0u);
        ProtocolV2_WriteU16LE(&batch.payload[phase_offset + 11u], 20000u);
        batch.payload[phase_offset + 15u] = ROBOT_ARM_PHASE_FLAG_XY_ENABLE;
        phase_offset = (uint8_t)(phase_offset + PROTOCOL_V2_PHASE_SIZE);
        batch.payload[phase_offset + 3u] = 20u;
        ProtocolV2_WriteU16LE(&batch.payload[phase_offset + 9u], 20000u);
        ProtocolV2_WriteU16LE(&batch.payload[phase_offset + 11u], 0u);
        batch.payload[phase_offset + 15u] = ROBOT_ARM_PHASE_FLAG_XY_ENABLE;
        s_next_result = ROBOT_ARM_OK;
        s_status.arm_state = ROBOT_ARM_IDLE;
        phase_before = s_phase_start_count;
        before = s_tx_count;
        RobotArmProtocol_HandlePhaseBatch(&batch);
        TEST_CHECK(s_phase_start_count == (uint8_t)(phase_before + 1u));
        TEST_CHECK(s_last_phase.delta_x == 201);
        TEST_CHECK(s_last_phase.f0 == 0u && s_last_phase.f1 == 20000u);
        TEST_CHECK(s_tx_count == (uint8_t)(before + 1u));
        TEST_CHECK(s_tx[before].data[1] == ROBOT_ARM_ACK_ACCEPTED);

        /* page4 必须给出当前批次、1 起始的正在执行 Phase 和包含当前条的剩余数量。 */
        TestClearFrame(&request, ROBOT_ARM_CMD_STATUS, 0x392u);
        request.data[0] = 4u;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_STATUS_RSP);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[0] == 4u &&
                   s_tx[s_tx_count - 1u].data[1] == 1u);
        TEST_CHECK(ProtocolV2_ReadU16LE(&s_tx[s_tx_count - 1u].data[2]) ==
                   0x1234u);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[4] == 1u &&
                   s_tx[s_tx_count - 1u].data[5] == 2u &&
                   s_tx[s_tx_count - 1u].data[6] == 2u);

        /* 相同 SEQ 只重发 ACK，不得再次启动第一条。 */
        RobotArmProtocol_HandlePhaseBatch(&batch);
        TEST_CHECK(s_phase_start_count == (uint8_t)(phase_before + 1u));

        TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
        RobotArmProtocol_Task();
        TEST_CHECK(s_phase_start_count == (uint8_t)(phase_before + 2u));
        TEST_CHECK(s_last_phase.delta_y == 20);
        TEST_CHECK(s_last_phase.f0 == 20000u && s_last_phase.f1 == 0u);

        /* 首条完成并启动第二条后，当前编号和剩余条数必须同步推进。 */
        TestClearFrame(&request, ROBOT_ARM_CMD_STATUS, 0x393u);
        request.data[0] = 4u;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[0] == 4u &&
                   s_tx[s_tx_count - 1u].data[1] == 1u);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[4] == 2u &&
                   s_tx[s_tx_count - 1u].data[5] == 2u &&
                   s_tx[s_tx_count - 1u].data[6] == 1u);

        TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
        RobotArmProtocol_Task();
        TestClearFrame(&request, ROBOT_ARM_CMD_STATUS, 0x391u);
        request.data[0] = 3u;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_CMD_PHASE_BATCH);
        TEST_CHECK(s_tx[s_tx_count - 1u].data[6] == ROBOT_ARM_EVENT_COMPLETED);
    }

    /* homeAcceleration=0 必须原样传给执行层，由对应轴回退默认值。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 0x12Fu);
    request.data[0] = ROBOT_AXIS_Z;
    ProtocolV2_WriteU16LE(&request.data[1], 500u);
    RobotArmProtocol_HandleFrame(&request);
    TEST_CHECK(s_last_call == 2u && s_last_axis == ROBOT_AXIS_Z);
    TEST_CHECK(s_last_speed == 500u && s_last_acceleration == 0u);
    TestSetAsyncResult(ROBOT_MOVE_END_COMPLETED, ROBOT_ARM_OK);
    RobotArmProtocol_Task();
    return s_failure;
}
