#include <stdint.h>
#include "robot_arm.h"
#include "robot_arm_config.h"
#include "robot_arm_sensor.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_test_failure == 0)) s_test_failure = __LINE__; } while (0)

static uint32_t s_now_ms;
static uint8_t s_busy[ROBOT_AXIS_COUNT];
static uint32_t s_completed[ROBOT_AXIS_COUNT];
static uint32_t s_remaining[ROBOT_AXIS_COUNT];
static int8_t s_direction[ROBOT_AXIS_COUNT];
static uint32_t s_start_count[ROBOT_AXIS_COUNT];
static uint32_t s_stop_count[ROBOT_AXIS_COUNT];
static uint32_t s_last_start_speed[ROBOT_AXIS_COUNT];
static uint8_t s_start_enters_busy[ROBOT_AXIS_COUNT] = {1u, 1u, 1u};
static int s_test_failure;
uint8_t g_robot_arm_logic_test_pose_safety_blocked;

/** 为无 C 运行库的测试可执行文件提供最小内存清零实现。 */
void *memset(void *destination, int value, __SIZE_TYPE__ count)
{
    unsigned char *bytes = (unsigned char *)destination;
    __SIZE_TYPE__ index;
    for (index = 0u; index < count; index++)
    {
        bytes[index] = (unsigned char)value;
    }
    return destination;
}

/** 提供状态机逻辑测试使用的毫秒时钟。 */
uint32_t millis(void) { return s_now_ms; }

/** 模拟统一驱动启动并记录命令。 */
uint8_t RobotArmDriver_Start(RobotAxisId_t axis, int8_t direction,
                             uint32_t steps, uint32_t speed)
{
    if (s_busy[axis] || (steps == 0u)) return 0u;
    s_direction[axis] = direction;
    s_last_start_speed[axis] = speed;
    s_start_count[axis]++;
    if (!s_start_enters_busy[axis])
    {
        /* 模拟底层错误返回成功，但实际没有进入运行态。 */
        return 1u;
    }
    s_busy[axis] = 1u;
    s_completed[axis] = 0u;
    s_remaining[axis] = steps;
    return 1u;
}

/** 模拟统一驱动停止。 */
void RobotArmDriver_Stop(RobotAxisId_t axis)
{
    s_stop_count[axis]++;
    s_busy[axis] = 0u;
}
/** 查询模拟轴是否忙碌。 */
uint8_t RobotArmDriver_IsBusy(RobotAxisId_t axis) { return s_busy[axis]; }
/** 查询模拟轴剩余步数。 */
uint32_t RobotArmDriver_GetRemainingSteps(RobotAxisId_t axis) { return s_remaining[axis]; }
/** 查询模拟轴完成步数。 */
uint32_t RobotArmDriver_GetCompletedSteps(RobotAxisId_t axis) { return s_completed[axis]; }
/** 模拟 DMA 续段前的负向限位检查，确保 Home 命中传感器立即停轴。 */
uint8_t RobotArmDriver_ShouldStopForNegativeLimit(RobotAxisId_t axis)
{
    return (s_direction[axis] < 0 && RobotArmSensor_IsTriggered(
        (RobotArmSensorId_t)(ROBOT_ARM_SENSOR_X_HOME + axis))) ? 1u : 0u;
}

static void TestSensorSnapshot(uint8_t s1, uint8_t s2, uint8_t s3)
{
    uint8_t data[2] = {0u, 0u};
    /* 与生产 ShiftRegisterInput_ReadAll() 交给 RobotArm 的 inputData[0] bit0..2 一致。 */
    data[0] = (uint8_t)((s1 << 0) | (s2 << 1) | (s3 << 2));
    RobotArmSensor_UpdateSnapshot(data);
}

static void TestDriverComplete(RobotAxisId_t axis)
{
    s_completed[axis] += s_remaining[axis];
    s_remaining[axis] = 0u;
    s_busy[axis] = 0u;
}

static void TestHomeAxis(RobotAxisId_t axis, uint8_t home_bit);

static void TestResetAndHomeAll(void)
{
    RobotArm_Init();
    g_robot_arm_logic_test_pose_safety_blocked = 0u;
    TestSensorSnapshot(0u, 0u, 0u);
    TestHomeAxis(ROBOT_AXIS_X, 0u);
    TestHomeAxis(ROBOT_AXIS_Y, 0u);
    TestHomeAxis(ROBOT_AXIS_Z, 0u);
    TestSensorSnapshot(0u, 0u, 0u);
}

#ifndef ROBOT_ARM_SAFE_DISABLED_TEST
static void TestSetPose(int32_t x, int32_t y, int32_t z)
{
    if (x != RobotArm_GetX())
    {
        TEST_CHECK(RobotArm_MoveX(x, 100u) == ROBOT_ARM_OK);
        TestDriverComplete(ROBOT_AXIS_X);
        RobotArm_Task();
    }
    if (y != RobotArm_GetY())
    {
        TEST_CHECK(RobotArm_MoveY(y, 100u) == ROBOT_ARM_OK);
        TestDriverComplete(ROBOT_AXIS_Y);
        RobotArm_Task();
    }
    if (z != RobotArm_GetZ())
    {
        TEST_CHECK(RobotArm_MoveZ(z, 100u) == ROBOT_ARM_OK);
        TestDriverComplete(ROBOT_AXIS_Z);
        RobotArm_Task();
    }
}
#endif

static void TestHomeAxis(RobotAxisId_t axis, uint8_t home_bit)
{
    uint8_t s1 = 0u, s2 = 0u, s3 = 0u;
    if (axis == ROBOT_AXIS_X) s1 = home_bit;
    if (axis == ROBOT_AXIS_Y) s2 = home_bit;
    if (axis == ROBOT_AXIS_Z) s3 = home_bit;
    TestSensorSnapshot(s1, s2, s3);
    TEST_CHECK(RobotArm_HomeAxis(axis) == ROBOT_ARM_OK);
    RobotArm_Task();
    if (home_bit)
    {
        /* 初始 Active 直接置零，当前单阶段流程绝不反向脱离或二次寻零。 */
        TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
        TEST_CHECK(RobotArm_IsPositionValid(axis) == 1u);
        return;
    }
    TEST_CHECK(s_direction[axis] == -1);
    if (axis == ROBOT_AXIS_X) s1 = 1u;
    if (axis == ROBOT_AXIS_Y) s2 = 1u;
    if (axis == ROBOT_AXIS_Z) s3 = 1u;
    TestSensorSnapshot(s1, s2, s3);
    RobotArm_Task();
    TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
    TEST_CHECK(RobotArm_IsPositionValid(axis) == 1u);
}

/**
 * 验证单轴 Home 只驱动被请求的实际轴，并在该轴完成后立即结束。
 *
 * @param axis 本次需要验证的 X、Y 或 Z 机械轴。
 */
static void TestSingleAxisHomeDoesNotChain(RobotAxisId_t axis)
{
    uint32_t x_starts = s_start_count[ROBOT_AXIS_X];
    uint32_t y_starts = s_start_count[ROBOT_AXIS_Y];
    uint32_t z_starts = s_start_count[ROBOT_AXIS_Z];

    TestHomeAxis(axis, 0u);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TEST_CHECK((s_start_count[ROBOT_AXIS_X] > x_starts) ==
               (axis == ROBOT_AXIS_X));
    TEST_CHECK((s_start_count[ROBOT_AXIS_Y] > y_starts) ==
               (axis == ROBOT_AXIS_Y));
    TEST_CHECK((s_start_count[ROBOT_AXIS_Z] > z_starts) ==
               (axis == ROBOT_AXIS_Z));
}

#ifndef ROBOT_ARM_SAFE_DISABLED_TEST
static void TestFinishHomeAllAxis(RobotAxisId_t axis)
{
    uint8_t s1 = 0u, s2 = 0u, s3 = 0u;
    static const uint32_t expected_fast_speed[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_HOME_FAST_SPEED_X,
        ROBOT_ARM_HOME_FAST_SPEED_Y,
        ROBOT_ARM_HOME_FAST_SPEED_Z};
    TestSensorSnapshot(0u, 0u, 0u);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == -1);
    TEST_CHECK(s_last_start_speed[axis] == expected_fast_speed[axis]);
    if (axis == ROBOT_AXIS_X) s1 = 1u;
    if (axis == ROBOT_AXIS_Y) s2 = 1u;
    if (axis == ROBOT_AXIS_Z) s3 = 1u;
    TestSensorSnapshot(s1, s2, s3);
    RobotArm_Task();
    TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
}
#endif

/** 运行 RobotArm 的纯逻辑回归场景。 */
#ifdef ROBOT_ARM_SINGLE_AXIS_HOME_TEST
/** 验证 X/Y/Z 单轴置零、完成收尾和 Busy 拒绝的独立回归入口。 */
int main(void)
{
    RobotArm_Init();
    TestSensorSnapshot(0u, 0u, 0u);
    TestSingleAxisHomeDoesNotChain(ROBOT_AXIS_X);
    TestSingleAxisHomeDoesNotChain(ROBOT_AXIS_Y);
    TestSingleAxisHomeDoesNotChain(ROBOT_AXIS_Z);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_X) == ROBOT_ARM_OK);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_Y) == ROBOT_ARM_ERR_BUSY);
    RobotArm_Stop();
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    return s_test_failure;
}
#elif defined(ROBOT_ARM_HOME_LIMIT_CHAIN_TEST)
/** 验证 S1/S2/S3 的完整快照映射及三轴 Home、运行中物理零点完成语义。 */
int main(void)
{
    RobotAxisId_t axis;
    RobotArmStatus_t status;
    uint8_t bitmap;

    RobotArm_Init();
    /* 0x00、0x01、0x02、0x04 及所有 S1/S2/S3 组合必须和 STATUS D9 一致。 */
    for (bitmap = 0u; bitmap < 8u; bitmap++)
    {
        TestSensorSnapshot((uint8_t)(bitmap & 1u),
                           (uint8_t)((bitmap >> 1) & 1u),
                           (uint8_t)((bitmap >> 2) & 1u));
        RobotArm_GetStatus(&status);
        TEST_CHECK(status.s1_x_home == (uint8_t)(bitmap & 1u));
        TEST_CHECK(status.s2_y_home == (uint8_t)((bitmap >> 1) & 1u));
        TEST_CHECK(status.s3_z_home == (uint8_t)((bitmap >> 2) & 1u));
    }

    for (axis = ROBOT_AXIS_X; axis < ROBOT_AXIS_COUNT; axis++)
    {
        uint32_t starts = s_start_count[axis];
        uint32_t stops;

        /* 对应 S1/S2/S3 已 Active 时，Home 绝不能产生负方向 STEP。 */
        TestHomeAxis(axis, 1u);
        TEST_CHECK(s_start_count[axis] == starts);
        TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
        TEST_CHECK(RobotArm_IsPositionValid(axis) == 1u);

        /* Active 只阻止轴继续压向零点外侧，正方向必须能脱离传感器。 */
        if (axis == ROBOT_AXIS_X)
        {
            TEST_CHECK(RobotArm_MoveXRelative(1, 100u) == ROBOT_ARM_OK);
            TestDriverComplete(axis); RobotArm_Task();
        }
        if (axis == ROBOT_AXIS_Y) {
            TEST_CHECK(RobotArm_MoveYRelative(1, 100u) == ROBOT_ARM_OK);
            TestDriverComplete(axis); RobotArm_Task();
        }
        if (axis == ROBOT_AXIS_Z) {
            TEST_CHECK(RobotArm_MoveZRelative(1, 100u) == ROBOT_ARM_OK);
            TestDriverComplete(axis); RobotArm_Task();
        }

        TestSensorSnapshot(0u, 0u, 0u);
        if (axis == ROBOT_AXIS_X) TEST_CHECK(RobotArm_MoveXRelative(-1, 100u) == ROBOT_ARM_OK);
        if (axis == ROBOT_AXIS_Y) TEST_CHECK(RobotArm_MoveYRelative(-1, 100u) == ROBOT_ARM_OK);
        if (axis == ROBOT_AXIS_Z) TEST_CHECK(RobotArm_MoveZRelative(-1, 100u) == ROBOT_ARM_OK);
        TEST_CHECK(s_busy[axis] == 1u);
        stops = s_stop_count[axis];
        TestSensorSnapshot(axis == ROBOT_AXIS_X, axis == ROBOT_AXIS_Y,
                           axis == ROBOT_AXIS_Z);
        /* 新 HC165 完整快照到达的同一调用栈必须停止当前 DMA，不能等 RobotArm_Task。 */
        TEST_CHECK(s_busy[axis] == 0u);
        TEST_CHECK(s_stop_count[axis] == stops + 1u);
        RobotArm_GetStatus(&status);
        TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_COMPLETED);
        TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
        TEST_CHECK(RobotArm_IsPositionValid(axis) == 1u);
        if (axis == ROBOT_AXIS_X) TEST_CHECK(RobotArm_GetX() == 0);
        if (axis == ROBOT_AXIS_Y) TEST_CHECK(RobotArm_GetY() == 0);
        if (axis == ROBOT_AXIS_Z) TEST_CHECK(RobotArm_GetZ() == 0);

        /* 已经在物理零点还请求继续负向才是真正的越零 LIMIT，且不得启动 STEP。 */
        starts = s_start_count[axis];
        if (axis == ROBOT_AXIS_X) TEST_CHECK(RobotArm_MoveXRelative(-5000, 100u) == ROBOT_ARM_ERR_LIMIT);
        if (axis == ROBOT_AXIS_Y) TEST_CHECK(RobotArm_MoveYRelative(-5000, 100u) == ROBOT_ARM_ERR_LIMIT);
        if (axis == ROBOT_AXIS_Z) TEST_CHECK(RobotArm_MoveZRelative(-5000, 100u) == ROBOT_ARM_ERR_LIMIT);
        TEST_CHECK(s_start_count[axis] == starts);
        TestSensorSnapshot(0u, 0u, 0u);
    }
    return s_test_failure;
}
#elif defined(ROBOT_ARM_HOME_CONFIG_REJECT_TEST)
/** 验证生产默认 Home 参数未标定时会拒绝单轴置零，且不产生 STEP 启动。 */
int main(void)
{
    uint32_t starts;
    RobotArm_Init();
    TestSensorSnapshot(0u, 0u, 0u);
    starts = s_start_count[ROBOT_AXIS_X] +
             s_start_count[ROBOT_AXIS_Y] +
             s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_X) == ROBOT_ARM_ERR_CONFIG);
    TEST_CHECK((s_start_count[ROBOT_AXIS_X] +
                s_start_count[ROBOT_AXIS_Y] +
                s_start_count[ROBOT_AXIS_Z]) == starts);
    return s_test_failure;
}
#elif defined(ROBOT_ARM_HOME_REQUEST_SPEED_TEST)
/**
 * 验证 0x31 单轴 Home 仅依赖请求中的快速速度。
 *
 * 测试配置故意将 HomeAll 的三轴默认快速速度置为 0；三轴仍应使用传入的 500 steps/s
 * 启动实际驱动，证明默认速度不是单轴 Home 的隐藏配置前置条件。
 */
int main(void)
{
    RobotAxisId_t axis;
    for (axis = ROBOT_AXIS_X; axis < ROBOT_AXIS_COUNT; axis++)
    {
        TestSensorSnapshot(0u, 0u, 0u);
        TEST_CHECK(RobotArm_HomeAxisWithSpeed(axis, 500u) == ROBOT_ARM_OK);
        RobotArm_Task();
        TEST_CHECK(s_start_count[axis] == 1u);
        TEST_CHECK(s_last_start_speed[axis] == 500u);
        TEST_CHECK(s_direction[axis] == -1);
        RobotArm_Stop();
    }
    return s_test_failure;
}
#elif defined(ROBOT_ARM_SAFE_DISABLED_TEST)
int main(void)
{
    uint32_t starts;
    TestResetAndHomeAll();
    starts = s_start_count[ROBOT_AXIS_X] +
             s_start_count[ROBOT_AXIS_Y] +
             s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(1, 1, 1) == ROBOT_ARM_ERR_CONFIG);
    TEST_CHECK((s_start_count[ROBOT_AXIS_X] +
                s_start_count[ROBOT_AXIS_Y] +
                s_start_count[ROBOT_AXIS_Z]) == starts);
    return s_test_failure;
}
#else
int main(void)
{
    RobotArmStatus_t status;
    RobotArmMoveDebug_t move_debug;
    uint32_t x_starts;
    uint32_t y_starts;
    uint32_t z_starts;
    RobotArm_Init();
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveTo(10, 20, 30) == ROBOT_ARM_ERR_POSITION_UNKNOWN);

    /* X/Y/Z 单轴 Home 分别只能启动自身，完成后不得串行进入其他轴。 */
    TestSingleAxisHomeDoesNotChain(ROBOT_AXIS_X);
    TestSingleAxisHomeDoesNotChain(ROBOT_AXIS_Y);
    TestSingleAxisHomeDoesNotChain(ROBOT_AXIS_Z);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_X) == ROBOT_ARM_OK);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_Y) == ROBOT_ARM_ERR_BUSY);
    RobotArm_Stop();
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_Home() == ROBOT_ARM_OK);
    TestFinishHomeAllAxis(ROBOT_AXIS_Z);
    TestFinishHomeAllAxis(ROBOT_AXIS_Y);
    TestFinishHomeAllAxis(ROBOT_AXIS_X);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TestSensorSnapshot(0u, 0u, 0u);

    TEST_CHECK(RobotArm_MoveTo(10, 20, 30) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] > 0u);
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    /* 最后一轴完成的同一轮必须释放组合状态，避免 page0 捕获伪 Busy。 */
    TEST_CHECK(s_busy[ROBOT_AXIS_X] == 0u);
    TEST_CHECK(s_busy[ROBOT_AXIS_Y] == 0u);
    TEST_CHECK(s_busy[ROBOT_AXIS_Z] == 0u);
    TEST_CHECK(RobotArm_GetX() == 10);
    TEST_CHECK(RobotArm_GetY() == 20);
    TEST_CHECK(RobotArm_GetZ() == 30);
    TEST_CHECK(status.x_state == ROBOT_AXIS_IDLE);
    TEST_CHECK(status.y_state == ROBOT_AXIS_IDLE);
    TEST_CHECK(status.z_state == ROBOT_AXIS_IDLE);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TEST_CHECK(status.operation == ROBOT_OP_NONE);
    TEST_CHECK(status.move_to_state == ROBOT_MOVE_TO_IDLE);
    TEST_CHECK(RobotArm_IsBusy() == 0u);

    /* X 从正坐标负向目标 0 首次命中 S1 后，X 必须归零完成，Y/Z 仍继续完成整条 MOVE_TO。 */
    TestSetPose(1, 1, 1);
    TEST_CHECK(RobotArm_MoveTo(0, 2, 3) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_busy[ROBOT_AXIS_X] == 1u);
    TestSensorSnapshot(1u, 0u, 0u);
    TEST_CHECK(s_busy[ROBOT_AXIS_X] == 0u);
    TEST_CHECK(RobotArm_GetX() == 0);
    TEST_CHECK(RobotArm_IsHomed(ROBOT_AXIS_X) == 1u);
    TEST_CHECK(RobotArm_IsPositionValid(ROBOT_AXIS_X) == 1u);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_busy[ROBOT_AXIS_Y] == 1u);
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_busy[ROBOT_AXIS_Z] == 1u);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TEST_CHECK(RobotArm_GetX() == 0 && RobotArm_GetY() == 2 && RobotArm_GetZ() == 3);
    TEST_CHECK(RobotArm_IsPositionValid(ROBOT_AXIS_X) == 1u);
    TestSensorSnapshot(0u, 0u, 0u);

    /* speed=0 必须继续使用每轴既有默认速度，不改变旧 MOVE_TO 行为。 */
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_X] == ROBOT_ARM_X_DEFAULT_SPEED);
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_Y] == ROBOT_ARM_Y_DEFAULT_SPEED);
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_Z] == ROBOT_ARM_Z_DEFAULT_SPEED);

    /* 临时速度只作用于当前 MOVE_TO；协议最大值仍必须按三轴上限传递给实际驱动。 */
    TEST_CHECK(RobotArm_MoveToWithSpeed(20, 30, 40, 500u, 500u, 500u) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_X] == 500u);
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_Y] == 500u);
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_Z] == 500u);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    TEST_CHECK(RobotArm_MoveToWithSpeed(30, 40, 50, 65535u, 65535u, 65535u) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_X] == ROBOT_ARM_X_MAX_SPEED);
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_Y] == ROBOT_ARM_Y_MAX_SPEED);
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_last_start_speed[ROBOT_AXIS_Z] == ROBOT_ARM_Z_MAX_SPEED);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();

    /* 后续既有回归仍从原来的已完成坐标开始，避免扩展用例改变历史场景前置条件。 */
    TEST_CHECK(RobotArm_MoveTo(10, 20, 30) == ROBOT_ARM_OK);
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();

    /* 三轴目标均等于当前位置时必须直接完成，不能向底层发送新命令。 */
    x_starts = s_start_count[ROBOT_AXIS_X];
    y_starts = s_start_count[ROBOT_AXIS_Y];
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveTo(10, 20, 30) == ROBOT_ARM_OK);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == x_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);

    /* WAIT_START 阶段 STOP：没有输出脉冲，坐标有效性保留且立即允许新任务。 */
    TEST_CHECK(RobotArm_MoveTo(11, 21, 31) == ROBOT_ARM_OK);
    RobotArm_Stop();
    RobotArm_GetStatus(&status);
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(RobotArm_IsBusy() == 0u);
    TEST_CHECK(status.arm_state == ROBOT_ARM_IDLE);
    TEST_CHECK(status.operation == ROBOT_OP_NONE);
    TEST_CHECK(status.move_to_state == ROBOT_MOVE_TO_IDLE);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_STOPPED);
    TEST_CHECK(status.x_valid == 1u && status.y_valid == 1u && status.z_valid == 1u);
    TEST_CHECK(RobotArm_GetX() == 10 && RobotArm_GetY() == 20 && RobotArm_GetZ() == 30);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_X] == ROBOT_MOVE_AXIS_NOT_REQUIRED);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_Y] == ROBOT_MOVE_AXIS_NOT_REQUIRED);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_Z] == ROBOT_MOVE_AXIS_NOT_REQUIRED);
    TEST_CHECK(move_debug.finalize_reason == ROBOT_MOVE_FINALIZE_STOPPED);
    TEST_CHECK(RobotArm_MoveTo(12, 22, 32) == ROBOT_ARM_OK);
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == 0);
    TEST_CHECK(RobotArm_IsBusy() == 1u);
    RobotArm_Stop();

    /* MoveTo 期间拒绝插入任务，Stop 后不得继续启动后续轴。 */
    {
        uint32_t y_starts = s_start_count[ROBOT_AXIS_Y];
        uint32_t z_starts = s_start_count[ROBOT_AXIS_Z];
        TEST_CHECK(RobotArm_MoveTo(11, 21, 31) == ROBOT_ARM_OK);
        RobotArm_Task();
        TEST_CHECK(RobotArm_MoveTo(12, 22, 32) == ROBOT_ARM_ERR_BUSY);
        TEST_CHECK(RobotArm_MoveYRelative(1, 100u) == ROBOT_ARM_ERR_BUSY);
        RobotArm_Stop();
        RobotArm_GetStatus(&status);
        RobotArm_GetMoveDebug(&move_debug);
        TEST_CHECK(s_busy[ROBOT_AXIS_X] == 0u);
        TEST_CHECK(s_busy[ROBOT_AXIS_Y] == 0u);
        TEST_CHECK(s_busy[ROBOT_AXIS_Z] == 0u);
        TEST_CHECK(status.x_state == ROBOT_AXIS_IDLE);
        TEST_CHECK(status.y_state == ROBOT_AXIS_IDLE);
        TEST_CHECK(status.z_state == ROBOT_AXIS_IDLE);
        TEST_CHECK(status.arm_state == ROBOT_ARM_IDLE);
        TEST_CHECK(status.operation == ROBOT_OP_NONE);
        TEST_CHECK(status.move_to_state == ROBOT_MOVE_TO_IDLE);
        TEST_CHECK(status.error_code == ROBOT_ARM_ERR_STOPPED);
        TEST_CHECK(move_debug.finalize_reason == ROBOT_MOVE_FINALIZE_STOPPED);
        TEST_CHECK(RobotArm_IsBusy() == 0u);
        RobotArm_Task();
        TEST_CHECK(RobotArm_IsBusy() == 0u);
        TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
        TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);
        TEST_CHECK(RobotArm_IsHomed(ROBOT_AXIS_X) == 1u);
        TEST_CHECK(RobotArm_IsPositionValid(ROBOT_AXIS_X) == 0u);
        /* STOP 后保留的 X 旧坐标不可用于 V2 绝对或相对目标计算。 */
        TEST_CHECK(RobotArm_MoveTo(12, 22, 32) == ROBOT_ARM_ERR_POSITION_UNKNOWN);
        TEST_CHECK(RobotArm_MoveXRelative(1, 100u) == ROBOT_ARM_ERR_POSITION_UNKNOWN);
    }

    TestHomeAxis(ROBOT_AXIS_X, 0u);
    /* 回零重新建立可信坐标后，相对移动仍应按原语义正常执行。 */
    TEST_CHECK(RobotArm_MoveXRelative(1, 100u) == ROBOT_ARM_OK);
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetX() == 1);
    TestSensorSnapshot(0u, 0u, 0u);

    /* Home 超时必须同时清除 homed 和 position_valid，且不能伪造零点。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_X) == ROBOT_ARM_OK);
    RobotArm_Task();
    s_now_ms += 1000u;
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_HOME_TIMEOUT);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_TIMEOUT);
    TEST_CHECK(status.x_homed == 0u);
    TEST_CHECK(status.x_valid == 0u);

    /* Z Home 从 S3 已触发开始时直接置零，不能产生任何负方向 STEP。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestSensorSnapshot(0u, 0u, 1u);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_Z) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetZ() == 0);
    TEST_CHECK(RobotArm_IsHomed(ROBOT_AXIS_Z) == 1u);
    TEST_CHECK(RobotArm_IsPositionValid(ROBOT_AXIS_Z) == 1u);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);

    /* 为组合运动异常场景重新建立三轴可信坐标。 */
    TestSensorSnapshot(0u, 0u, 0u);
    TestHomeAxis(ROBOT_AXIS_X, 0u);
    TestHomeAxis(ROBOT_AXIS_Y, 0u);
    TestHomeAxis(ROBOT_AXIS_Z, 0u);

    /* MoveTo 的 Y 阶段超时后不得启动 Z，已完成或未参与的轴保持有效。 */
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveTo(0, 1, 1) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    s_now_ms += 31000u;
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_MOVE_TIMEOUT);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_TIMEOUT);
    TEST_CHECK(status.y_valid == 0u);
    TEST_CHECK(status.z_valid == 1u);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);

    /* MoveTo 的 Z- 阶段目标为 0 时，S3 命中必须立即归零并正常完成。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestHomeAxis(ROBOT_AXIS_Y, 0u);
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveTo(0, 0, 1) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    RobotArm_Task();
    TestSensorSnapshot(0u, 0u, 1u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_COMPLETED);
    TEST_CHECK(status.z == 0);
    TEST_CHECK(status.z_valid == 1u);
    TEST_CHECK(status.z_homed == 1u);

    /* 三轴 MOVE_TO 中 Z=0 命中 S3 后仍应完成该轴并收尾整条命令。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestHomeAxis(ROBOT_AXIS_Z, 0u);
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveZ(1, 100u) == ROBOT_ARM_OK);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    TEST_CHECK(RobotArm_MoveTo(0, 0, 0) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    RobotArm_Task();
    TestSensorSnapshot(0u, 0u, 1u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_COMPLETED);
    TEST_CHECK(status.z == 0);
    TEST_CHECK(status.z_valid == 1u);
    TEST_CHECK(status.z_homed == 1u);

    /* SafeMove 启用时，位置未知必须在产生任何 STEP 之前拒绝任务。 */
    RobotArm_Init();
    TestSensorSnapshot(0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveToSafe(1, 1, 1) == ROBOT_ARM_ERR_POSITION_UNKNOWN);

    /* 仅 Z 改变时直接执行 Final Z，不额外执行 Raise Z。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 5);
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(0, 0, 20) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.safe_move_state == ROBOT_SAFE_MOVE_FINAL_Z_START);
    RobotArm_Task();
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts + 1u);
    TEST_CHECK(s_direction[ROBOT_AXIS_Z] == 1);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetZ() == 20);

    /* 当前 Z 低于安全高度时，严格执行 Raise Z、X、Y、Final Z。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 20);
    TEST_CHECK(RobotArm_MoveToSafe(5, 6, 30) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.safe_move_state == ROBOT_SAFE_MOVE_RAISE_Z_START);
    RobotArm_Task();
    TEST_CHECK(s_direction[ROBOT_AXIS_Z] == -1);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_direction[ROBOT_AXIS_X] == 1);
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_direction[ROBOT_AXIS_Y] == 1);
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(s_direction[ROBOT_AXIS_Z] == 1);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetX() == 5);
    TEST_CHECK(RobotArm_GetY() == 6);
    TEST_CHECK(RobotArm_GetZ() == 30);

    /* 当前 Z 已处于安全高度时跳过 Raise Z，直接从 X 开始。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 5);
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(5, 6, 20) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.safe_move_state == ROBOT_SAFE_MOVE_X_START);
    RobotArm_Task();
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);
    RobotArm_Stop();

    /* XYZ 均等于目标时立即成功且不得产生 STEP。 */
    TestResetAndHomeAll();
    x_starts = s_start_count[ROBOT_AXIS_X];
    y_starts = s_start_count[ROBOT_AXIS_Y];
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(0, 0, 0) == ROBOT_ARM_OK);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == x_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);

    /* Raise Z 期间 Stop 后，X/Y 和 Final Z 永远不得继续启动。 */
    TestSetPose(0, 0, 20);
    x_starts = s_start_count[ROBOT_AXIS_X];
    y_starts = s_start_count[ROBOT_AXIS_Y];
    TEST_CHECK(RobotArm_MoveToSafe(5, 6, 30) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    RobotArm_Stop();
    RobotArm_Task();
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == x_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.safe_move_state == ROBOT_SAFE_MOVE_IDLE);

    /* X Timeout 后不得启动 Y 或 Final Z。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 5);
    y_starts = s_start_count[ROBOT_AXIS_Y];
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(1, 1, 20) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    s_now_ms += 31000u;
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_TIMEOUT);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);

    /* SafeMove 的 Y=0 命中 S2 后，该轴正常归零并继续执行 Final Z。 */
    TestResetAndHomeAll();
    TestSetPose(0, 1, 5);
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(1, 0, 20) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    TestSensorSnapshot(0u, 1u, 0u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.y == 0);
    TEST_CHECK(status.y_valid == 1u);
    TEST_CHECK(status.y_homed == 1u);
    RobotArm_Task();
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts + 1u);

    /* Final Z 前重新检查真实 XY 姿态，互锁失败时不得启动 Z。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 5);
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveToSafe(1, 1, 20) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    g_robot_arm_logic_test_pose_safety_blocked = 1u;
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_INTERLOCK);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_INTERLOCK);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);

    /* Final Z 的目标为 0 时，S3 命中是准确到位，SafeMove 应正常完成。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 5);
    TEST_CHECK(RobotArm_MoveToSafe(1, 1, 0) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    RobotArm_Task();
    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    RobotArm_Task();
    TestSensorSnapshot(0u, 0u, 1u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_COMPLETED);
    TEST_CHECK(status.z == 0);
    TEST_CHECK(status.z_valid == 1u);
    TEST_CHECK(status.z_homed == 1u);

    /* SafeMove 独占期间拒绝 Relative 和普通 MoveTo。 */
    TestResetAndHomeAll();
    TestSetPose(0, 0, 5);
    TEST_CHECK(RobotArm_MoveToSafe(1, 1, 20) == ROBOT_ARM_OK);
    TEST_CHECK(RobotArm_MoveXRelative(1, 100u) == ROBOT_ARM_ERR_BUSY);
    TEST_CHECK(RobotArm_MoveTo(1, 1, 20) == ROBOT_ARM_ERR_BUSY);
    RobotArm_Stop();
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.operation == ROBOT_OP_NONE);
    TEST_CHECK(status.safe_move_state == ROBOT_SAFE_MOVE_IDLE);

    /* 复现现场目标，逐轴证明 Start、方向、步数和运行态后才提交 current。 */
    TestResetAndHomeAll();
    TestSetPose(123, 123, 123);
    TEST_CHECK(RobotArm_MoveTo(1550, 100, 100) == ROBOT_ARM_OK);
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(move_debug.received_current[ROBOT_AXIS_X] == 123);
    TEST_CHECK(move_debug.received_target[ROBOT_AXIS_X] == 1550);
    TEST_CHECK(move_debug.received_delta[ROBOT_AXIS_X] == 1427);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_X] ==
               ROBOT_MOVE_AXIS_WAIT_START);
    RobotArm_Task();
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(move_debug.start_called[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(move_debug.start_steps[ROBOT_AXIS_X] == 1427u);
    TEST_CHECK(move_debug.start_result[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(move_debug.busy_before[ROBOT_AXIS_X] == 0u);
    TEST_CHECK(move_debug.busy_after[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_X] ==
               ROBOT_MOVE_AXIS_RUNNING);
    TEST_CHECK(s_direction[ROBOT_AXIS_X] == 1);
    TEST_CHECK(s_remaining[ROBOT_AXIS_X] == 1427u);
    TEST_CHECK(RobotArm_GetX() == 123);

    TestDriverComplete(ROBOT_AXIS_X);
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetX() == 1550);
    RobotArm_Task();
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(move_debug.start_called[ROBOT_AXIS_Y] == 1u);
    TEST_CHECK(move_debug.start_steps[ROBOT_AXIS_Y] == 23u);
    TEST_CHECK(move_debug.busy_after[ROBOT_AXIS_Y] == 1u);
    TEST_CHECK(s_direction[ROBOT_AXIS_Y] == -1);
    TEST_CHECK(RobotArm_GetY() == 123);

    TestDriverComplete(ROBOT_AXIS_Y);
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetY() == 100);
    RobotArm_Task();
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(move_debug.start_called[ROBOT_AXIS_Z] == 1u);
    TEST_CHECK(move_debug.start_steps[ROBOT_AXIS_Z] == 23u);
    TEST_CHECK(move_debug.busy_after[ROBOT_AXIS_Z] == 1u);
    TEST_CHECK(s_direction[ROBOT_AXIS_Z] == -1);
    TEST_CHECK(RobotArm_GetZ() == 123);

    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(RobotArm_GetX() == 1550);
    TEST_CHECK(RobotArm_GetY() == 100);
    TEST_CHECK(RobotArm_GetZ() == 100);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_X] ==
               ROBOT_MOVE_AXIS_COMPLETED);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_Y] ==
               ROBOT_MOVE_AXIS_COMPLETED);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_Z] ==
               ROBOT_MOVE_AXIS_COMPLETED);
    TEST_CHECK(move_debug.finalize_reason == ROBOT_MOVE_FINALIZE_COMPLETED);
    TEST_CHECK(RobotArm_IsBusy() == 0u);

    /* Start 虚假成功但 Busy 未置位时必须报错，三个 current 均保持原值。 */
    TestResetAndHomeAll();
    TestSetPose(123, 123, 123);
    s_start_enters_busy[ROBOT_AXIS_X] = 0u;
    TEST_CHECK(RobotArm_MoveTo(1550, 100, 100) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_GetMoveDebug(&move_debug);
    TEST_CHECK(move_debug.start_called[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(move_debug.start_result[ROBOT_AXIS_X] == 1u);
    TEST_CHECK(move_debug.busy_after[ROBOT_AXIS_X] == 0u);
    TEST_CHECK(move_debug.axis_progress[ROBOT_AXIS_X] ==
               ROBOT_MOVE_AXIS_WAIT_START);
    TEST_CHECK(move_debug.finalize_reason ==
               ROBOT_MOVE_FINALIZE_START_FAILED);
    TEST_CHECK(RobotArm_GetX() == 123);
    TEST_CHECK(RobotArm_GetY() == 123);
    TEST_CHECK(RobotArm_GetZ() == 123);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_ERROR);
    TEST_CHECK(RobotArm_IsBusy() == 0u);
    s_start_enters_busy[ROBOT_AXIS_X] = 1u;
    return s_test_failure;
}
#endif
