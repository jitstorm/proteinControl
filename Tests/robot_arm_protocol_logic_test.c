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

    for (axis = ROBOT_AXIS_X; axis < ROBOT_AXIS_COUNT; axis++)
    {
        TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS,
                       (uint16_t)(0x120u + axis));
        request.data[0] = axis;
        before = s_tx_count;
        RobotArmProtocol_HandleFrame(&request);
        TEST_CHECK(s_last_call == 2u && s_last_axis == axis);
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

    /* 活动单轴 Home 未完成时，下一条动作必须得到 BUSY ACK，不能覆盖原 SEQ。 */
    TestClearFrame(&request, ROBOT_ARM_CMD_HOME_AXIS, 0x330u);
    request.data[0] = ROBOT_AXIS_X;
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
    return s_failure;
}
