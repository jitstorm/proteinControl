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
/** 记录协议层主动要求底层停机的次数，用于验证同向续段不会调用 STOP。 */
static uint32_t s_stop_count[ROBOT_AXIS_COUNT];
static uint32_t s_phase_start[ROBOT_AXIS_COUNT];
static uint32_t s_phase_end[ROBOT_AXIS_COUNT];
/** 记录 X/Y 当前 Phase 实际下传的加速时间，单位毫秒；0 表示匀速。 */
static uint32_t s_phase_acceleration_time_ms[ROBOT_AXIS_COUNT];
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
/**
 * 提供未触发的 Home 驱动桩，保证 Phase 端到端测试可独立链接。
 *
 * @param axis 模拟启动的机械轴。
 * @param direction 模拟运动方向。
 * @param steps 模拟输出的 STEP 脉冲数。
 * @param speed 模拟速度，单位 steps/s。
 * @param acceleration Home 加速度，本测试不模拟加速轮廓。
 * @return 驱动桩受理结果，与普通启动桩一致。
 */
uint8_t RobotArmDriver_StartWithAcceleration(
    RobotAxisId_t axis, int8_t direction, uint32_t steps, uint32_t speed,
    uint32_t acceleration)
{
    (void)acceleration;
    return RobotArmDriver_Start(axis, direction, steps, speed);
}
/**
 * 提供未触发的 Home 末段驱动桩，不模拟寻零速度轮廓。
 *
 * @param axis 模拟启动的机械轴。
 * @param direction 模拟运动方向。
 * @param steps 模拟输出的 STEP 脉冲数。
 * @param fast_speed 快速段速度，单位 steps/s。
 * @param slow_speed 慢速段速度，单位 steps/s；本测试不模拟切换。
 * @param slow_zone_steps 慢速段长度，单位 STEP 脉冲；本测试不模拟切换。
 * @param acceleration Home 加速度，本测试不模拟加速轮廓。
 * @return 驱动桩受理结果，与普通启动桩一致。
 */
uint8_t RobotArmDriver_StartHomeApproach(
    RobotAxisId_t axis, int8_t direction, uint32_t steps, uint32_t fast_speed,
    uint32_t slow_speed, uint32_t slow_zone_steps, uint32_t acceleration)
{
    (void)slow_speed; (void)slow_zone_steps; (void)acceleration;
    return RobotArmDriver_Start(axis, direction, steps, fast_speed);
}
/** 模拟 X/Y Phase DMA 启动并记录真正传入的各轴独立频率。 */
uint8_t RobotArmDriver_StartPhase(RobotAxisId_t axis, int8_t direction,
                                  uint32_t steps, uint32_t start_frequency,
                                  uint32_t terminal_frequency,
                                  uint32_t acceleration_time_ms)
{
    (void)direction;
    if ((axis == ROBOT_AXIS_Z) || s_busy[axis] || steps == 0u) return 0u;
    s_busy[axis] = 1u; s_remaining[axis] = steps; s_completed[axis] = 0u;
    s_start_count[axis]++;
    s_phase_start[axis] = start_frequency;
    s_phase_end[axis] = terminal_frequency;
    s_phase_acceleration_time_ms[axis] = acceleration_time_ms;
    return 1u;
}
/** 模拟实际 DMA 停机，并记录是否在同向连续 Phase 的中间边界错误停止。 */
void RobotArmDriver_Stop(RobotAxisId_t axis)
{
    s_stop_count[axis]++;
    s_busy[axis] = 0u;
}
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
        s_start_count[axis] = 0u; s_stop_count[axis] = 0u;
        s_phase_start[axis] = 0u; s_phase_end[axis] = 0u;
        s_phase_acceleration_time_ms[axis] = 0u;
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
    uint16_t payload_length = (uint16_t)(4u + (uint16_t)count * PROTOCOL_V2_PHASE_SIZE);
    uint16_t frame_length = (uint16_t)(payload_length + 10u);
    uint16_t offset;
    uint16_t crc;
    uint8_t index;
    int32_t x_target = 0;
    int32_t y_target = 0;
    int32_t z_target = 0;
    raw[0] = 0xAAu; raw[1] = 0xFEu; raw[2] = 0x39u;
    ProtocolV2_WriteU16LE(&raw[3], seq);
    ProtocolV2_WriteU16LE(&raw[5], payload_length);
    ProtocolV2_WriteU16LE(&raw[7], run_id);
    raw[9] = count; raw[10] = 0u;
    for (index = 0u; index < count; index++)
    {
        offset = (uint16_t)(11u + (uint16_t)index * PROTOCOL_V2_PHASE_SIZE);
        if (mixed && ((index % 4u) == 1u))
        {
            z_target += (int32_t)(5u + index);
            TestWriteI24(&raw[offset], x_target); TestWriteI24(&raw[offset + 3u], y_target);
            TestWriteI24(&raw[offset + 6u], z_target);
            ProtocolV2_WriteU16LE(&raw[offset + 9u], 0u); ProtocolV2_WriteU16LE(&raw[offset + 11u], 0u);
            ProtocolV2_WriteU16LE(&raw[offset + 13u], 0u); ProtocolV2_WriteU16LE(&raw[offset + 15u], 0u);
            ProtocolV2_WriteU16LE(&raw[offset + 17u], 300u + index);
        }
        else
        {
            x_target += (int32_t)(10u + index);
            y_target += (int32_t)(20u + index);
            if (mixed && ((index % 4u) == 2u)) z_target += (int32_t)(5u + index);
            TestWriteI24(&raw[offset], x_target);
            TestWriteI24(&raw[offset + 3u], y_target);
            TestWriteI24(&raw[offset + 6u], z_target);
            ProtocolV2_WriteU16LE(&raw[offset + 9u], 2000u + index);
            ProtocolV2_WriteU16LE(&raw[offset + 11u], 1000u);
            ProtocolV2_WriteU16LE(&raw[offset + 13u], 1500u + index);
            ProtocolV2_WriteU16LE(&raw[offset + 15u], 1000u);
            ProtocolV2_WriteU16LE(&raw[offset + 17u], 400u + index);
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

    /* Test 1：242B 一次性输入。 */
    TestReset(); guarded.before = 0xA5u; guarded.after = 0x5Au;
    length = TestBuildBatch(guarded.raw, 100u, 0x1234u, 12u, 0u);
    TEST_CHECK(length == 242u); TestFeed(guarded.raw, length, 0u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && s_start_count[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(guarded.before == 0xA5u && guarded.after == 0x5Au);

    /* Test 2：同一 242B 帧每次只进入一个 UART 字节。 */
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

    /* Test 4：最大 16 Phase / 308B payload / 318B frame 和哨兵边界。 */
    TestReset(); guarded.before = 0xA5u; guarded.after = 0x5Au;
    length = TestBuildBatch(guarded.raw, 103u, 0x5678u, 16u, 1u);
    TEST_CHECK(length == 318u); TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && guarded.before == 0xA5u && guarded.after == 0x5Au);

    /* Test 5：两帧粘包按 main.c 的 64B 消费节奏处理，B 因 A 活动得到 BUSY。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 104u, 1u, 12u, 0u);
    { uint16_t length2 = TestBuildBatch(raw2, 105u, 2u, 12u, 0u);
      uint8_t joined[PROTOCOL_V2_PHASE_BATCH_MAX_FRAME_SIZE * 2u];
      for (index = 0u; index < length; index++) joined[index] = guarded.raw[index];
      for (index = 0u; index < length2; index++) joined[length + index] = raw2[index];
      TestFeedMainCadence(joined, (uint16_t)(length + length2)); }
    TestTick();
    TEST_CHECK(TestAckCount() == 2u && s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_ERR_BUSY);

    /* Test 6：坏 CRC 后的合法 242B 帧必须重新同步并接受。 */
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
    guarded.raw[9] = 15u;
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED);
    TestReset(); length = TestBuildBatch(guarded.raw, 111u, 1u, 16u, 0u);
    guarded.raw[9] = 17u;
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED);

    /* Test 8：16 条混合 Phase 必须严格顺序执行到最终终态。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 112u, 1u, 16u, 1u);
    TestFeed(guarded.raw, length, 1u); TestTick();
    for (index = 0u; index < 16u; index++) TestCompleteCurrent();
    TestTick();
    TEST_CHECK(RobotArm_IsBusy() == 0u && s_start_count[ROBOT_AXIS_X] == 12u &&
               s_start_count[ROBOT_AXIS_Y] == 12u && s_start_count[ROBOT_AXIS_Z] == 8u);
    TestBuildStatusPage3(raw2, 118u); TestFeed(raw2, PROTOCOL_V2_FRAME_SIZE, 1u);
    TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_STATUS_RSP &&
               s_tx[s_tx_count - 1u].data[2] == ROBOT_ARM_CMD_PHASE_BATCH &&
               s_tx[s_tx_count - 1u].data[6] == ROBOT_ARM_EVENT_COMPLETED);

    /* Test 9：XYZ 的任一轴未完成时，不得越过当前 Phase 启动下一条。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 113u, 1u, 2u, 0u);
    offset = 11u;
    TestWriteI24(&guarded.raw[offset + 6u], 5);
    /* 第二条 Z 目标保持首条终点，自动判定该轴静止。 */
    TestWriteI24(&guarded.raw[(uint16_t)(offset + PROTOCOL_V2_PHASE_SIZE + 6u)], 5);
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u); TestTick();
    TestCompleteAxis(ROBOT_AXIS_X); TestCompleteAxis(ROBOT_AXIS_Y); TestTick();
    TEST_CHECK(s_busy[ROBOT_AXIS_Z] == 1u && s_start_count[ROBOT_AXIS_X] == 1u);
    TestCompleteAxis(ROBOT_AXIS_Z); TestTick(); TestTick();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 2u && s_start_count[ROBOT_AXIS_Y] == 2u &&
               s_start_count[ROBOT_AXIS_Z] == 1u);

    /* Test 10：真实三段速度边界数据，经完整 0x39 输入后检查驱动参数。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 114u, 3u, 3u, 0u);
    /* 建立足够的 Y 正坐标，使三条绝对目标均在合法行程内。 */
    TEST_CHECK(RobotArm_MoveY(20000, 1000u) == ROBOT_ARM_OK);
    TestCompleteAxis(ROBOT_AXIS_Y); TestTick();
    for (index = 0u; index < 3u; index++)
    {
        offset = (uint16_t)(11u + index * PROTOCOL_V2_PHASE_SIZE);
        TestWriteI24(&guarded.raw[offset], (index == 0u) ? 201 :
                     ((index == 1u) ? 1390 : 1591));
        TestWriteI24(&guarded.raw[offset + 3u], (index == 0u) ? 18333 :
                     ((index == 1u) ? 8478 : 6811));
        ProtocolV2_WriteU16LE(&guarded.raw[offset + 9u], 20000u);
        ProtocolV2_WriteU16LE(&guarded.raw[offset + 11u], 1000u);
        ProtocolV2_WriteU16LE(&guarded.raw[offset + 13u], 15000u);
        ProtocolV2_WriteU16LE(&guarded.raw[offset + 15u], 1000u);
    }
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(s_phase_start[ROBOT_AXIS_X] == 500u && s_phase_start[ROBOT_AXIS_Y] == 500u &&
               s_phase_end[ROBOT_AXIS_X] == 20000u && s_phase_end[ROBOT_AXIS_Y] == 15000u &&
               s_phase_acceleration_time_ms[ROBOT_AXIS_X] == 1000u &&
               s_phase_acceleration_time_ms[ROBOT_AXIS_Y] == 1000u);

    /* Test 11：0ms 必须直达终止速度匀速运行，而不是退化为 1ms 加速。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 126u, 9u, 1u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[20u], 3000u);
    ProtocolV2_WriteU16LE(&guarded.raw[22u], 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[24u], 2800u);
    ProtocolV2_WriteU16LE(&guarded.raw[26u], 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_ACCEPTED &&
               s_phase_start[ROBOT_AXIS_X] == 500u && s_phase_end[ROBOT_AXIS_X] == 3000u &&
               s_phase_acceleration_time_ms[ROBOT_AXIS_X] == 0u &&
               s_phase_start[ROBOT_AXIS_Y] == 500u && s_phase_end[ROBOT_AXIS_Y] == 2800u &&
               s_phase_acceleration_time_ms[ROBOT_AXIS_Y] == 0u);

    /* Test 12：相同 SEQ 只 ACK，不能二次启动或覆盖活动 Batch。 */
    TestFeed(guarded.raw, length, 1u); TEST_CHECK(TestAckCount() == 2u && s_start_count[ROBOT_AXIS_X] == 1u);

    /* Test 13：Phase5 中 STOP 必须停轴并废弃后续条目；位置失效前不能再启动 Batch。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 115u, 1u, 16u, 1u);
    TestFeed(guarded.raw, length, 1u); TestTick();
    for (index = 0u; index < 5u; index++) TestCompleteCurrent();
    { uint32_t starts_before = s_start_count[ROBOT_AXIS_X] + s_start_count[ROBOT_AXIS_Y] + s_start_count[ROBOT_AXIS_Z];
      uint8_t stop_raw[PROTOCOL_V2_FRAME_SIZE];
      TestBuildStop(stop_raw, 116u); TestFeed(stop_raw, PROTOCOL_V2_FRAME_SIZE, 1u); TestTick();
      TEST_CHECK(s_busy[ROBOT_AXIS_X] == 0u && s_busy[ROBOT_AXIS_Y] == 0u && s_busy[ROBOT_AXIS_Z] == 0u);
      TestTick(); TestTick();
      TEST_CHECK((s_start_count[ROBOT_AXIS_X] + s_start_count[ROBOT_AXIS_Y] + s_start_count[ROBOT_AXIS_Z]) == starts_before);
      /* 先消费原批次 STOPPED 终态，避免新请求覆盖原 CMD/SEQ。 */
      TestBuildStatusPage3(raw2, 116u); TestFeed(raw2, PROTOCOL_V2_FRAME_SIZE, 1u);
      length = TestBuildBatch(raw2, 117u, 2u, 1u, 0u);
      TestWriteI24(&raw2[11u], RobotArm_GetX() + 10);
      TestWriteI24(&raw2[14u], RobotArm_GetY() + 20);
      ProtocolV2_WriteU16LE(&raw2[length - 2u],
                            ProtocolV2_CalculateCrc(raw2, (uint16_t)(length - 2u)));
      TestFeed(raw2, length, 1u); TestTick();
       TEST_CHECK((s_start_count[ROBOT_AXIS_X] + s_start_count[ROBOT_AXIS_Y] + s_start_count[ROBOT_AXIS_Z]) == starts_before);
       TEST_CHECK(s_tx[s_tx_count - 1u].cmd == ROBOT_ARM_CMD_ACK &&
                  s_tx[s_tx_count - 1u].data[1] == ROBOT_ARM_ACK_REJECTED); }

    /* Test 14：三条同方向 Phase 的下一条必须由最后一个 DMA TC 直接启动，不调用 TestTick；
     * 下一条起始速度必须自动继承上一条终止速度，不能被距离比例改写。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 119u, 3u, 3u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[39u], 2200u);
    ProtocolV2_WriteU16LE(&guarded.raw[41u], 1000u);
    ProtocolV2_WriteU16LE(&guarded.raw[43u], 1700u);
    ProtocolV2_WriteU16LE(&guarded.raw[45u], 1000u);
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u); TestTick();
    TestCompleteCurrent();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 2u && s_start_count[ROBOT_AXIS_Y] == 2u);
    TEST_CHECK(s_stop_count[ROBOT_AXIS_X] == 0u && s_stop_count[ROBOT_AXIS_Y] == 0u);
    TEST_CHECK(s_phase_start[ROBOT_AXIS_X] == 2000u && s_phase_end[ROBOT_AXIS_X] == 2200u &&
               s_phase_start[ROBOT_AXIS_Y] == 1500u && s_phase_end[ROBOT_AXIS_Y] == 1700u);
    TestCompleteCurrent();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 3u && s_start_count[ROBOT_AXIS_Y] == 3u);
    TEST_CHECK(s_stop_count[ROBOT_AXIS_X] == 0u && s_stop_count[ROBOT_AXIS_Y] == 0u);
    TestCompleteCurrent(); TestTick();
    TEST_CHECK(s_stop_count[ROBOT_AXIS_X] == 1u && s_stop_count[ROBOT_AXIS_Y] == 1u);
    TestBuildStatusPage3(raw2, 120u); TestFeed(raw2, PROTOCOL_V2_FRAME_SIZE, 1u);
    TEST_CHECK(s_tx[s_tx_count - 1u].data[6] == ROBOT_ARM_EVENT_COMPLETED);

    /* Test 15：同一轴跨条反向必须在启动前 CONFIG 拒绝，不能先输出首条 STEP。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 121u, 4u, 2u, 0u);
    TestWriteI24(&guarded.raw[30u], -11); /* Phase1 target_x 位于固定 Batch payload 的第二条起点。 */
    { uint16_t crc = ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)); ProtocolV2_WriteU16LE(&guarded.raw[length - 2u], crc); }
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == 0u && TestAckCount() == 1u &&
               s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED && s_tx[0].data[2] == ROBOT_ARM_ERR_CONFIG);

    /* Test 16：三轴目标均不变时无 DMA 终态，必须在启动前拒绝。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 122u, 5u, 1u, 0u);
    TestWriteI24(&guarded.raw[11u], 0);
    TestWriteI24(&guarded.raw[14u], 0);
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[2] == ROBOT_ARM_ERR_CONFIG &&
               s_start_count[ROBOT_AXIS_X] == 0u);

    /* Test 17：仅 Z 目标变化时自动启动 Z；零速必须拒绝。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 123u, 6u, 1u, 0u);
    TestWriteI24(&guarded.raw[11u], 0);
    TestWriteI24(&guarded.raw[14u], 0);
    TestWriteI24(&guarded.raw[17u], 5);
    ProtocolV2_WriteU16LE(&guarded.raw[28u], 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[2] == ROBOT_ARM_ERR_CONFIG &&
               s_start_count[ROBOT_AXIS_Z] == 0u);
    TestReset(); length = TestBuildBatch(guarded.raw, 124u, 7u, 1u, 0u);
    TestWriteI24(&guarded.raw[11u], 0);
    TestWriteI24(&guarded.raw[14u], 0);
    TestWriteI24(&guarded.raw[17u], 5);
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u); TestTick();
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_ACCEPTED &&
               s_start_count[ROBOT_AXIS_X] == 0u && s_start_count[ROBOT_AXIS_Y] == 0u &&
               s_start_count[ROBOT_AXIS_Z] == 1u);

    /* Test 18：实际运动的 X/Y 终止速度为 0 必须在输出 STEP 前拒绝。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 127u, 10u, 1u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[20u], 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED &&
               s_tx[0].data[2] == ROBOT_ARM_ERR_CONFIG && s_start_count[ROBOT_AXIS_X] == 0u);

    /* Test 19：超过 PB10/PB11 50000 steps/s 上限的终止速度必须拒绝。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 128u, 11u, 1u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[20u], 50001u);
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED &&
               s_tx[0].data[2] == ROBOT_ARM_ERR_CONFIG && s_start_count[ROBOT_AXIS_X] == 0u);

    /* Test 20：旧版 20 字节 Phase 即使 CRC 正确，也因长度不匹配拒绝。 */
    TestReset(); length = TestBuildBatch(guarded.raw, 125u, 8u, 1u, 0u);
    ProtocolV2_WriteU16LE(&guarded.raw[5u], 24u);
    guarded.raw[30u] = 0x01u;
    guarded.raw[31u] = 0x55u;
    length++;
    ProtocolV2_WriteU16LE(&guarded.raw[length - 2u],
                          ProtocolV2_CalculateCrc(guarded.raw, (uint16_t)(length - 2u)));
    TestFeed(guarded.raw, length, 1u);
    TEST_CHECK(TestAckCount() == 1u && s_tx[0].data[1] == ROBOT_ARM_ACK_REJECTED &&
               s_start_count[ROBOT_AXIS_X] == 0u);

    return s_failure;
}
