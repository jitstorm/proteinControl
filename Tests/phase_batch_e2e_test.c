#include <stdint.h>
#include "protocol_v2.h"
#include "robot_arm_protocol.h"
#include "robot_arm.h"
#include "robot_arm_sensor.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_failure == 0)) s_failure = __LINE__; } while (0)

static int s_failure;
static uint32_t s_now_ms;
static uint8_t s_busy[ROBOT_AXIS_COUNT];
static uint32_t s_remaining[ROBOT_AXIS_COUNT];
static uint32_t s_completed[ROBOT_AXIS_COUNT];
static uint32_t s_start_count[ROBOT_AXIS_COUNT];
static uint32_t s_phase_start[ROBOT_AXIS_COUNT];
static uint32_t s_phase_end[ROBOT_AXIS_COUNT];
static ProtocolV2Frame_t s_tx[64];
static uint8_t s_tx_count;
uint8_t g_robot_arm_logic_test_pose_safety_blocked;

/** 为无运行库 host 测试提供最小内存函数，避免链接到宿主 CRT。 */
void *memset(void *destination, int value, __SIZE_TYPE__ count)
{
    unsigned char *bytes = (unsigned char *)destination;
    __SIZE_TYPE__ index;
    for (index = 0u; index < count; index++) bytes[index] = (unsigned char)value;
    return destination;
}
void *memcpy(void *destination, const void *source, __SIZE_TYPE__ count)
{
    unsigned char *to = (unsigned char *)destination;
    const unsigned char *from = (const unsigned char *)source;
    __SIZE_TYPE__ index;
    for (index = 0u; index < count; index++) to[index] = from[index];
    return destination;
}
uint32_t millis(void) { return s_now_ms; }

/** 模拟真实 DMA 启动；计数器用于确认每条 Phase 只启动一次。 */
uint8_t RobotArmDriver_Start(RobotAxisId_t axis, int8_t direction,
                             uint32_t steps, uint32_t speed)
{
    (void)direction; (void)speed;
    if (s_busy[axis] || steps == 0u) return 0u;
    s_busy[axis] = 1u; s_remaining[axis] = steps; s_completed[axis] = 0u;
    s_start_count[axis]++;
    return 1u;
}
/** 模拟 X/Y Phase DMA 启动并记录真正传入的比例频率。 */
uint8_t RobotArmDriver_StartPhase(RobotAxisId_t axis, int8_t direction,
                                  uint32_t steps, uint32_t start_frequency,
                                  uint32_t end_frequency)
{
    (void)direction;
    if ((axis == ROBOT_AXIS_Z) || s_busy[axis] || steps == 0u) return 0u;
    s_busy[axis] = 1u; s_remaining[axis] = steps; s_completed[axis] = 0u;
    s_start_count[axis]++;
    s_phase_start[axis] = start_frequency;
    s_phase_end[axis] = end_frequency;
    return 1u;
}
void RobotArmDriver_Stop(RobotAxisId_t axis) { s_busy[axis] = 0u; }
uint8_t RobotArmDriver_IsBusy(RobotAxisId_t axis) { return s_busy[axis]; }
uint32_t RobotArmDriver_GetRemainingSteps(RobotAxisId_t axis) { return s_remaining[axis]; }
uint32_t RobotArmDriver_GetCompletedSteps(RobotAxisId_t axis) { return s_completed[axis]; }
uint8_t RobotArmDriver_ShouldStopForNegativeLimit(RobotAxisId_t axis)
{ (void)axis; return 0u; }

/** 记录 MCU 对每个请求产生的固定 24B ACK/STATUS，模拟串口物理发送完成。 */
static uint8_t TestTx(const uint8_t *raw, uint8_t length)
{
    TEST_CHECK(length == PROTOCOL_V2_FRAME_SIZE);
    TEST_CHECK(s_tx_count < 64u);
    if (s_tx_count < 64u)
    {
        TEST_CHECK(ProtocolV2_Decode(raw, &s_tx[s_tx_count]) == 1u);
        s_tx_count++;
    }
    return 1u;
}

/** 初始化真实 parser、协议层和 RobotArm 状态机，并提供未触发的 S1/S2/S3 快照。 */
static void TestReset(void)
{
    uint8_t sensors[2] = {0u, 0u};
    uint8_t axis;
    for (axis = 0u; axis < ROBOT_AXIS_COUNT; axis++)
    {
        s_busy[axis] = 0u; s_remaining[axis] = 0u; s_completed[axis] = 0u;
        s_start_count[axis] = 0u; s_phase_start[axis] = 0u; s_phase_end[axis] = 0u;
    }
    s_now_ms = 0u; s_tx_count = 0u;
    ProtocolV2_Init();
    RobotArmProtocol_Init(TestTx);
    RobotArm_Init();
    RobotArmSensor_UpdateSnapshot(sensors);
}

/** 复用主循环的 V2 出队顺序，将已从 UART ring 取出的字节真正分发到 0x39 执行层。 */
static void TestDispatch(void)
{
    ProtocolV2PhaseBatchFrame_t batch;
    ProtocolV2Frame_t frame;
    while (RobotArmProtocol_CanAcceptRequest() && ProtocolV2_TakePhaseBatchFrame(&batch))
        RobotArmProtocol_HandlePhaseBatch(&batch);
    while (RobotArmProtocol_CanAcceptRequest() && ProtocolV2_TakeFrame(&frame))
        RobotArmProtocol_HandleFrame(&frame);
}

/** 从 UART ring 后的实际逐字节 parser 入口喂入一段数据，并按主循环节奏分发。 */
static void TestFeed(const uint8_t *data, uint16_t length, uint8_t dispatch_each_byte)
{
    uint16_t index;
    for (index = 0u; index < length; index++)
    {
        ProtocolV2_InputByte(data[index]);
        if (dispatch_each_byte) TestDispatch();
    }
    TestDispatch();
}

/** 模拟 main.c 每轮最多从 UART ring 取 64B 后再分发，覆盖连续粘包的真实节奏。 */
static void TestFeedMainCadence(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    for (index = 0u; index < length; index++)
    {
        ProtocolV2_InputByte(data[index]);
        if (((index + 1u) % 64u) == 0u) TestDispatch();
    }
    TestDispatch();
}

/** 构造一帧旧固定 24B STOP，用于验证 0x39 运行途中清批次。 */
static void TestBuildStop(uint8_t *raw, uint16_t seq)
{
    ProtocolV2Frame_t frame;
    uint8_t index;
    frame.cmd = ROBOT_ARM_CMD_STOP;
    frame.seq = seq;
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++) frame.data[index] = 0u;
    ProtocolV2_Encode(&frame, raw);
}

/** 构造 Page3 STATUS，读取真实 Batch 终态缓存而不直接观察协议私有变量。 */
static void TestBuildStatusPage3(uint8_t *raw, uint16_t seq)
{
    ProtocolV2Frame_t frame;
    uint8_t index;
    frame.cmd = ROBOT_ARM_CMD_STATUS;
    frame.seq = seq;
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++) frame.data[index] = 0u;
    frame.data[0] = 3u;
    ProtocolV2_Encode(&frame, raw);
}

/** 推进 RobotArm 与协议轮询，使已受理 Phase 输出实际模拟 DMA 启动。 */
static void TestTick(void)
{
    RobotArm_Task();
    RobotArmProtocol_Task();
}

/** 模拟指定轴 DMA 已输出全部步数，并从 TC 上下文进入 0x39 完成屏障。 */
static void TestCompleteAxis(RobotAxisId_t axis)
{
    s_completed[axis] += s_remaining[axis];
    s_remaining[axis] = 0u;
    s_busy[axis] = 0u;
    RobotArmProtocol_OnPhaseAxisDmaCompleted(axis);
}

/** 完成当前 Phase 的所有已启动 DMA；下一条必须已由最后一个 TC 直接启动。 */
static void TestCompleteCurrent(void)
{
    uint8_t axis;
    for (axis = 0u; axis < ROBOT_AXIS_COUNT; axis++)
        if (s_busy[axis]) TestCompleteAxis((RobotAxisId_t)axis);
}

/** 将有符号 int24 写入真实 payload 布局。 */
static void TestWriteI24(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
}

/** 构造真实 0x39 线格式；每条 Phase 使用不同字段模式以发现偏移或符号错误。 */
static uint16_t TestBuildBatch(uint8_t *raw, uint16_t seq, uint16_t run_id,
                               uint8_t count, uint8_t mixed)
{
    uint16_t payload_length = (uint16_t)(4u + (uint16_t)count * 16u);
    uint16_t frame_length = (uint16_t)(payload_length + 10u);
    uint16_t offset;
    uint16_t crc;
    uint8_t index;
    raw[0] = 0xAAu; raw[1] = 0xFEu; raw[2] = 0x39u;
    ProtocolV2_WriteU16LE(&raw[3], seq);
    ProtocolV2_WriteU16LE(&raw[5], payload_length);
    ProtocolV2_WriteU16LE(&raw[7], run_id);
    raw[9] = count; raw[10] = 0u;
    for (index = 0u; index < count; index++)
    {
        offset = (uint16_t)(11u + (uint16_t)index * 16u);
        if (mixed && ((index % 4u) == 1u))
        {
            TestWriteI24(&raw[offset], 0); TestWriteI24(&raw[offset + 3u], 0);
            TestWriteI24(&raw[offset + 6u], (int32_t)(5u + index));
            ProtocolV2_WriteU16LE(&raw[offset + 9u], 0u);
            ProtocolV2_WriteU16LE(&raw[offset + 11u], 0u);
            ProtocolV2_WriteU16LE(&raw[offset + 13u], 300u + index);
            raw[offset + 15u] = ROBOT_ARM_PHASE_FLAG_Z_ENABLE;
        }
        else
        {
            TestWriteI24(&raw[offset], (int32_t)(10u + index));
            TestWriteI24(&raw[offset + 3u], (int32_t)(20u + index));
            TestWriteI24(&raw[offset + 6u], (mixed && ((index % 4u) == 2u)) ?
                         (int32_t)(5u + index) : 0);
            ProtocolV2_WriteU16LE(&raw[offset + 9u], (index == 0u) ? 0u : 1000u + index);
            ProtocolV2_WriteU16LE(&raw[offset + 11u], 2000u + index);
            ProtocolV2_WriteU16LE(&raw[offset + 13u], 400u + index);
            raw[offset + 15u] = (mixed && ((index % 4u) == 2u)) ?
                                  (ROBOT_ARM_PHASE_FLAG_XY_ENABLE | ROBOT_ARM_PHASE_FLAG_Z_ENABLE |
                                   ROBOT_ARM_PHASE_FLAG_SYNC_END) : ROBOT_ARM_PHASE_FLAG_XY_ENABLE;
        }
    }
    raw[frame_length - 3u] = 0x55u;
    crc = ProtocolV2_CalculateCrc(raw, (uint16_t)(frame_length - 2u));
    ProtocolV2_WriteU16LE(&raw[frame_length - 2u], crc);
    return frame_length;
}

/** 返回当前已发送 ACK 数量。 */
static uint8_t TestAckCount(void)
{
    uint8_t index, count = 0u;
    for (index = 0u; index < s_tx_count; index++) if (s_tx[index].cmd == ROBOT_ARM_CMD_ACK) count++;
    return count;
}

/** 运行真实字节输入、Batch 提交、DMA 完成和终态回归。 */
int main(void)
{
    struct { uint8_t before; uint8_t raw[PROTOCOL_V2_PHASE_BATCH_MAX_FRAME_SIZE]; uint8_t after; } guarded;
    uint8_t raw2[PROTOCOL_V2_PHASE_BATCH_MAX_FRAME_SIZE];
    uint16_t length;
    uint16_t index;
    uint16_t offset;
    uint8_t chunks[] = {1u, 7u, 13u, 2u, 31u, 5u, 64u, 3u, 17u};

    /* Test 1：206B 一次性输入。 */
    TestReset(); guarded.before = 0xA5u; guarded.after = 0x5Au;
    length = TestBuildBatch(guarded.raw, 100u, 0x1234u, 12u, 0u);
    TEST_CHECK(length == 206u); TestFeed(guarded.raw, length, 0u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && s_start_count[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(guarded.before == 0xA5u && guarded.after == 0x5Au);

    /* Test 2：同一 206B 帧每次只进入一个 UART 字节。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 101u, 0x1234u, 12u, 0u);
    TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && s_start_count[ROBOT_AXIS_Y] == 1u);

    /* Test 3：确定性随机分块，分块边界覆盖 LEN、payload、TAIL 与 CRC。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 102u, 0x1234u, 12u, 0u); index = 0u;
    while (index < length)
    {
        uint16_t take = chunks[(index / 7u) % (uint16_t)(sizeof(chunks) / sizeof(chunks[0]))];
        if (take > (uint16_t)(length - index)) take = (uint16_t)(length - index);
        TestFeed(&guarded.raw[index], take, 0u); index = (uint16_t)(index + take);
    }
    TestTick(); TEST_CHECK(TestAckCount() == 1u && s_start_count[ROBOT_AXIS_X] == 1u);

    /* Test 4：最大 16 Phase / 260B payload / 270B frame 和哨兵边界。 */
    TestReset(); guarded.before = 0xA5u; guarded.after = 0x5Au;
    length = TestBuildBatch(guarded.raw, 103u, 0x5678u, 16u, 1u);
    TEST_CHECK(length == 270u); TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && guarded.before == 0xA5u && guarded.after == 0x5Au);

    /* Test 5：两帧粘包按 main.c 的 64B 消费节奏处理，B 因 A 活动得到 BUSY。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 104u, 1u, 12u, 0u);
    { uint16_t length2 = TestBuildBatch(raw2, 105u, 2u, 12u, 0u);
      uint8_t joined[412];
      for (index = 0u; index < length; index++) joined[index] = guarded.raw[index];
      for (index = 0u; index < length2; index++) joined[length + index] = raw2[index];
      TestFeedMainCadence(joined, (uint16_t)(length + length2)); }
    TestTick();
    TEST_CHECK(TestAckCount() == 2u && s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_BUSY);

    /* Test 6：坏 CRC 后的合法 206B 帧必须重新同步并接受。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 106u, 1u, 12u, 0u);
    guarded.raw[length - 1u] ^= 1u; TestFeed(guarded.raw, length, 1u);
    length = TestBuildBatch(raw2, 107u, 2u, 12u, 0u); TestFeed(raw2, length, 1u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].seq == 107u);

    /* Test 7：超长 LEN、截断、LEN/count 不匹配、count=17 均不得启动 Phase0。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 108u, 1u, 12u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[5], 300u); TestFeed(guarded.raw, 7u, 1u);
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 0u);
    TestReset(); length = TestBuildBatch(guarded.raw, 109u, 1u, 16u, 0u);
    TestFeed(guarded.raw, 100u, 1u); TEST_CHECK(TestAckCount() == 0u);
    TestReset(); length = TestBuildBatch(guarded.raw, 110u, 1u, 16u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[5], 244u); TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED);
    TestReset(); length = TestBuildBatch(guarded.raw, 111u, 1u, 16u, 0u);
    guarded.raw[9] = 17u; TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED);

    /* Test 8：16 条混合 Phase 必须严格顺序执行到最终终态。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 112u, 1u, 16u, 1u);
    TestFeed(guarded.raw, length, 1u); TestTick();
    for (index = 0u; index < 16u; index++) TestCompleteCurrent();
    TEST_CHECK(RobotArm_IsBusy() == 0u && s_start_count[ROBOT_AXIS_X] == 12u &&
               s_start_count[ROBOT_AXIS_Y] == 12u && s_start_count[ROBOT_AXIS_Z] == 8u);
    TestBuildStatusPage3(raw2, 118u); TestFeed(raw2, PROTOCOL_V2_FRAME_SIZE, 1u);
    TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_STATUS_RSP &&
               s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_CMD_PHASE_BATCH &&
               s_tx[s_tx_count - 1u].data[6] == ROBOT_ARM_EVENT_COMPLETED);

    /* Test 9：XYZ 的任一轴未完成时，不得越过当前 Phase 启动下一条。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 113u, 1u, 2u, 0u);
    offset = 11u; guarded.raw[offset + 15u] = (ROBOT_ARM_PHASE_FLAG_XY_ENABLE |
                                                ROBOT_ARM_PHASE_FLAG_Z_ENABLE |
                                                ROBOT_ARM_PHASE_FLAG_SYNC_END);
    TestWriteI24(&guarded.raw[offset + 6u], 5);
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u); TestTick();
    TestCompleteAxis(ROBOT_AXIS_X); TestCompleteAxis(ROBOT_AXIS_Y); TestTick();
    TEST_CHECK(s_busy[ROBOT_AXIS_Z] == 1u && s_start_count[ROBOT_AXIS_X] == 1u);
    TestCompleteAxis(ROBOT_AXIS_Z); TestTick(); TestTick();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 2u && s_start_count[ROBOT_AXIS_Y] == 2u &&
               s_start_count[ROBOT_AXIS_Z] == 1u);

    /* Test 10：真实三段速度边界数据，经完整 0x39 输入后检查驱动参数。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 114u, 3u, 3u, 0u);
    /* 建立足够的 Y 正坐标，使真实样例中的三段负向 Y 增量仍通过行程校验。 */
    TEST_CHECK(RobotArm_MoveY(20000, 1000u) == ROBOT_ARM_OK);
    TestCompleteAxis(ROBOT_AXIS_Y); TestTick();
    for (index = 0u; index < 3u; index++)
    {
        offset = (uint16_t)(11u + index * 16u);
        TestWriteI24(&guarded.raw[offset], (index == 1u) ? 1189 : 201);
        TestWriteI24(&guarded.raw[offset + 3u], (index == 1u) ? -9855 : -1667);
        ProtocolV2_WriteU16LE(&guarded.raw[offset + 9u], (index == 0u) ? 0u : 20000u);
        ProtocolV2_WriteU16LE(&guarded.raw[offset + 11u], (index == 2u) ? 0u : 20000u);
    }
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(s_phase_start[ROBOT_AXIS_X] == 60u && s_phase_start[ROBOT_AXIS_Y] == 500u &&
               s_phase_end[ROBOT_AXIS_X] == 2411u && s_phase_end[ROBOT_AXIS_Y] == 20000u);

    /* Test 11：相同 SEQ 只 ACK，不能二次启动或覆盖活动 Batch。 */
    TestFeed(guarded.raw, length, 1u); TEST_CHECK(TestAckCount() == 2u && s_start_count[ROBOT_AXIS_X] == 1u);

    /* Test 12：Phase5 中 STOP 必须停止当前 DMA、废弃后续条目，并允许新的 Batch。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 115u, 1u, 16u, 1u);
    TestFeed(guarded.raw, length, 1u); TestTick();
    for (index = 0u; index < 5u; index++) TestCompleteCurrent();
    { uint32_t starts_before = s_start_count[ROBOT_AXIS_X] + s_start_count[ROBOT_AXIS_Y] + s_start_count[ROBOT_AXIS_Z];
      uint8_t stop_raw[PROTOCOL_V2_FRAME_SIZE];
      TestBuildStop(stop_raw, 116u); TestFeed(stop_raw, PROTOCOL_V2_FRAME_SIZE, 1u); TestTick();
      TEST_CHECK(s_busy[ROBOT_AXIS_X] == 0u && s_busy[ROBOT_AXIS_Y] == 0u && s_busy[ROBOT_AXIS_Z] == 0u);
      TestTick(); TestTick();
      TEST_CHECK((s_start_count[ROBOT_AXIS_X] + s_start_count[ROBOT_AXIS_Y] + s_start_count[ROBOT_AXIS_Z]) == starts_before);
      length = TestBuildBatch(raw2, 117u, 2u, 1u, 0u); TestFeed(raw2, length, 1u); TestTick();
       TEST_CHECK(s_start_count[ROBOT_AXIS_X] + s_start_count[ROBOT_AXIS_Y] + s_start_count[ROBOT_AXIS_Z] > starts_before); }

    /* Test 13：三条同方向 Phase 的下一条必须由最后一个 DMA TC 直接启动，不调用 TestTick。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 119u, 3u, 3u, 0u);
    TestFeed(guarded.raw, length, 1u); TestTick();
    TestCompleteCurrent();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 2u && s_start_count[ROBOT_AXIS_Y] == 2u);
    TestCompleteCurrent();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 3u && s_start_count[ROBOT_AXIS_Y] == 3u);
    TestCompleteCurrent(); TestTick();
    TestBuildStatusPage3(raw2, 120u); TestFeed(raw2, PROTOCOL_V2_FRAME_SIZE, 1u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[6] == ROBOT_ARM_EVENT_COMPLETED);

    /* Test 14：同一轴跨条反向必须在启动前 CONFIG 拒绝，不能先输出首条 STEP。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 121u, 4u, 2u, 0u);
    TestWriteI24(&guarded.raw[27u], -11); /* Phase1 delta_x 位于固定 Batch payload 的第二条起点。 */
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 0u && TestAckCount() == 1u &&
               s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED && s_tx[0].data[2] == ROBOT_ARM_ERR_CONFIG);

    return s_failure;
}
