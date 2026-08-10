#include <stdint.h>
#include "robot_arm.h"
#include "robot_arm_sensor.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_test_failure == 0)) s_test_failure = __LINE__; } while (0)

static uint32_t s_now_ms;
static uint8_t s_busy[ROBOT_AXIS_COUNT];
static uint32_t s_completed[ROBOT_AXIS_COUNT];
static uint32_t s_remaining[ROBOT_AXIS_COUNT];
static int8_t s_direction[ROBOT_AXIS_COUNT];
static uint32_t s_start_count[ROBOT_AXIS_COUNT];
static int s_test_failure;

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
    (void)speed;
    if (s_busy[axis] || (steps == 0u)) return 0u;
    s_busy[axis] = 1u;
    s_completed[axis] = 0u;
    s_remaining[axis] = steps;
    s_direction[axis] = direction;
    s_start_count[axis]++;
    return 1u;
}

/** 模拟统一驱动停止。 */
void RobotArmDriver_Stop(RobotAxisId_t axis) { s_busy[axis] = 0u; }
/** 查询模拟轴是否忙碌。 */
uint8_t RobotArmDriver_IsBusy(RobotAxisId_t axis) { return s_busy[axis]; }
/** 查询模拟轴剩余步数。 */
uint32_t RobotArmDriver_GetRemainingSteps(RobotAxisId_t axis) { return s_remaining[axis]; }
/** 查询模拟轴完成步数。 */
uint32_t RobotArmDriver_GetCompletedSteps(RobotAxisId_t axis) { return s_completed[axis]; }

static void TestSensorSnapshot(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
{
    uint8_t data[2] = {0u, 0u};
    data[1] = (uint8_t)((s1 << 0) | (s2 << 1) | (s3 << 2) | (s4 << 3));
    RobotArmSensor_UpdateSnapshot(data);
}

static void TestDriverComplete(RobotAxisId_t axis)
{
    s_completed[axis] += s_remaining[axis];
    s_remaining[axis] = 0u;
    s_busy[axis] = 0u;
}

static void TestHomeAxis(RobotAxisId_t axis, uint8_t home_bit)
{
    uint8_t s1 = 0u, s2 = 0u, s3 = 0u;
    if (axis == ROBOT_AXIS_X) s1 = home_bit;
    if (axis == ROBOT_AXIS_Y) s2 = home_bit;
    if (axis == ROBOT_AXIS_Z) s3 = home_bit;
    TestSensorSnapshot(s1, s2, s3, 0u);
    TEST_CHECK(RobotArm_HomeAxis(axis) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == (home_bit ? 1 : -1));

    /* 初始已触发时先退出；未触发时直接快速找边。 */
    if (home_bit)
    {
        TestSensorSnapshot(0u, 0u, 0u, 0u);
        RobotArm_Task();
        TEST_CHECK(s_direction[axis] == -1);
    }
    if (axis == ROBOT_AXIS_X) s1 = 1u;
    if (axis == ROBOT_AXIS_Y) s2 = 1u;
    if (axis == ROBOT_AXIS_Z) s3 = 1u;
    TestSensorSnapshot(s1, s2, s3, 0u);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == 1);

    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TestDriverComplete(axis);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == -1);

    if (axis == ROBOT_AXIS_X) s1 = 1u;
    if (axis == ROBOT_AXIS_Y) s2 = 1u;
    if (axis == ROBOT_AXIS_Z) s3 = 1u;
    TestSensorSnapshot(s1, s2, s3, 0u);
    RobotArm_Task();
    TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
    TEST_CHECK(RobotArm_IsPositionValid(axis) == 1u);
}

static void TestFinishHomeAllAxis(RobotAxisId_t axis)
{
    uint8_t s1 = 0u, s2 = 0u, s3 = 0u;
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == -1);
    if (axis == ROBOT_AXIS_X) s1 = 1u;
    if (axis == ROBOT_AXIS_Y) s2 = 1u;
    if (axis == ROBOT_AXIS_Z) s3 = 1u;
    TestSensorSnapshot(s1, s2, s3, 0u);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == 1);
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TestDriverComplete(axis);
    RobotArm_Task();
    TEST_CHECK(s_direction[axis] == -1);
    if (axis == ROBOT_AXIS_X) s1 = 1u;
    if (axis == ROBOT_AXIS_Y) s2 = 1u;
    if (axis == ROBOT_AXIS_Z) s3 = 1u;
    TestSensorSnapshot(s1, s2, s3, 0u);
    RobotArm_Task();
    TEST_CHECK(RobotArm_IsHomed(axis) == 1u);
}

/** 运行 RobotArm 的纯逻辑回归场景。 */
int main(void)
{
    RobotArmStatus_t status;
    uint32_t x_starts;
    uint32_t y_starts;
    uint32_t z_starts;
    RobotArm_Init();
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveTo(10, 20, 30) == ROBOT_ARM_ERR_POSITION_UNKNOWN);

    TestHomeAxis(ROBOT_AXIS_X, 1u);
    TestHomeAxis(ROBOT_AXIS_Y, 0u);
    TestHomeAxis(ROBOT_AXIS_Z, 0u);
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TEST_CHECK(RobotArm_Home() == ROBOT_ARM_OK);
    TestFinishHomeAllAxis(ROBOT_AXIS_Z);
    TestFinishHomeAllAxis(ROBOT_AXIS_Y);
    TestFinishHomeAllAxis(ROBOT_AXIS_X);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TestSensorSnapshot(0u, 0u, 0u, 0u);

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
    RobotArm_Task();
    TEST_CHECK(RobotArm_GetX() == 10);
    TEST_CHECK(RobotArm_GetY() == 20);
    TEST_CHECK(RobotArm_GetZ() == 30);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);

    /* 三轴目标均等于当前位置时必须直接完成，不能向底层发送新命令。 */
    x_starts = s_start_count[ROBOT_AXIS_X];
    y_starts = s_start_count[ROBOT_AXIS_Y];
    z_starts = s_start_count[ROBOT_AXIS_Z];
    TEST_CHECK(RobotArm_MoveTo(10, 20, 30) == ROBOT_ARM_OK);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_IDLE);
    TEST_CHECK(s_start_count[ROBOT_AXIS_X] == x_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
    TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);

    /* MoveTo 期间拒绝插入任务，Stop 后不得继续启动后续轴。 */
    {
        uint32_t y_starts = s_start_count[ROBOT_AXIS_Y];
        uint32_t z_starts = s_start_count[ROBOT_AXIS_Z];
        TEST_CHECK(RobotArm_MoveTo(11, 21, 31) == ROBOT_ARM_OK);
        RobotArm_Task();
        TEST_CHECK(RobotArm_MoveTo(12, 22, 32) == ROBOT_ARM_ERR_BUSY);
        TEST_CHECK(RobotArm_MoveYRelative(1, 100u) == ROBOT_ARM_ERR_BUSY);
        RobotArm_Stop();
        RobotArm_Task();
        TEST_CHECK(s_start_count[ROBOT_AXIS_Y] == y_starts);
        TEST_CHECK(s_start_count[ROBOT_AXIS_Z] == z_starts);
        TEST_CHECK(RobotArm_IsHomed(ROBOT_AXIS_X) == 1u);
        TEST_CHECK(RobotArm_IsPositionValid(ROBOT_AXIS_X) == 0u);
    }

    TestHomeAxis(ROBOT_AXIS_X, 0u);
    TestSensorSnapshot(0u, 0u, 0u, 0u);

    /* S4 必须中止 Z+，且不得把 current 提交为 target。 */
    TEST_CHECK(RobotArm_MoveZ(40, 100u) == ROBOT_ARM_OK);
    TestSensorSnapshot(0u, 0u, 0u, 1u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_LIMIT);
    TEST_CHECK(status.z == 30);
    TEST_CHECK(status.z_valid == 0u);
    TEST_CHECK(status.z_homed == 1u);

    /* Home 超时必须同时清除 homed 和 position_valid，且不能伪造零点。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_X) == ROBOT_ARM_OK);
    RobotArm_Task();
    s_now_ms += 1000u;
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.error_code == ROBOT_ARM_ERR_HOME_TIMEOUT);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_TIMEOUT);
    TEST_CHECK(status.x_homed == 0u);
    TEST_CHECK(status.x_valid == 0u);

    /* Z Home 朝上运动时，即使 S4 已触发也必须允许离开下限位。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestSensorSnapshot(0u, 0u, 0u, 1u);
    TEST_CHECK(RobotArm_HomeAxis(ROBOT_AXIS_Z) == ROBOT_ARM_OK);
    RobotArm_Task();
    TEST_CHECK(s_direction[ROBOT_AXIS_Z] == -1);
    TEST_CHECK(RobotArm_GetState() == ROBOT_ARM_HOMING);
    RobotArm_Stop();

    /* 为组合运动异常场景重新建立三轴可信坐标。 */
    TestSensorSnapshot(0u, 0u, 0u, 0u);
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

    /* MoveTo 的 Z+ 阶段遇到 S4 时停止且不提交 target。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestHomeAxis(ROBOT_AXIS_Y, 0u);
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveTo(0, 0, 1) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    RobotArm_Task();
    TestSensorSnapshot(0u, 0u, 0u, 1u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_LIMIT);
    TEST_CHECK(status.z == 0);
    TEST_CHECK(status.z_valid == 0u);

    /* MoveTo 的 Z- 阶段遇到 S3 时采用同样的硬限位中断语义。 */
    TEST_CHECK(RobotArm_ClearError() == ROBOT_ARM_OK);
    TestHomeAxis(ROBOT_AXIS_Z, 0u);
    TestSensorSnapshot(0u, 0u, 0u, 0u);
    TEST_CHECK(RobotArm_MoveZ(1, 100u) == ROBOT_ARM_OK);
    TestDriverComplete(ROBOT_AXIS_Z);
    RobotArm_Task();
    TEST_CHECK(RobotArm_MoveTo(0, 0, 0) == ROBOT_ARM_OK);
    RobotArm_Task();
    RobotArm_Task();
    RobotArm_Task();
    TestSensorSnapshot(0u, 0u, 1u, 0u);
    RobotArm_Task();
    RobotArm_GetStatus(&status);
    TEST_CHECK(status.last_move_end_reason == ROBOT_MOVE_END_LIMIT);
    TEST_CHECK(status.z == 1);
    TEST_CHECK(status.z_valid == 0u);
    return s_test_failure;
}
