#include <stdint.h>
#include "robot_arm_driver.h"
#include "robot_arm_sensor.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_failure == 0)) s_failure = __LINE__; } while (0)

uint8_t HC595Data[4];
static int s_failure;
static uint8_t s_pb11_running;
static uint8_t s_pb10_running;
static uint8_t s_z_running;
static uint8_t s_allow_start = 1u;
static uint32_t s_pb11_steps;
static uint32_t s_pb10_steps;
static uint32_t s_z_steps;
static uint8_t s_pb10_direction;
static uint8_t s_z_direction;

/** 模拟 595 写入；方向影子值由测试直接检查。 */
void ShiftRegister_WriteAll(uint8_t *data) { (void)data; }

/** 为驱动适配层测试提供未触发的 Home 传感器状态。 */
uint8_t RobotArmSensor_IsTriggered(RobotArmSensorId_t sensor)
{
    (void)sensor;
    return 0u;
}

/** 模拟实际 PU2（PB11）梯形 DMA 请求并记录步数。 */
void stepdma_pb11_request_trap(uint32_t steps, uint32_t start,
                              uint32_t maximum, uint32_t acceleration)
{
    (void)start;
    (void)maximum;
    (void)acceleration;
    s_pb11_steps = steps;
    s_pb11_running = s_allow_start;
}

/** 模拟 PB11 停止。 */
void stepdma_pb11_stop(void) { s_pb11_running = 0u; }
/** 返回 PB11 模拟运行态。 */
uint8_t stepdma_pb11_is_running(void) { return s_pb11_running; }
/** 返回 PB11 模拟剩余步数。 */
uint32_t stepdma_pb11_get_remaining_steps(void) { return s_pb11_steps; }
/** 返回 PB11 模拟完成步数。 */
uint32_t stepdma_pb11_get_completed_steps(void) { return 0u; }

/** 模拟实际 PU1（PB10）启动并记录方向和步数。 */
uint8_t Stepper2_Start(uint8_t direction, uint32_t steps,
                       uint32_t target_frequency)
{
    (void)target_frequency;
    s_pb10_direction = direction;
    s_pb10_steps = steps;
    s_pb10_running = s_allow_start;
    return 1u;
}

/** 模拟 PB10 停止。 */
void Stepper2_Stop(void) { s_pb10_running = 0u; }
/** 返回 PB10 模拟运行态。 */
uint8_t Stepper2_IsBusy(void) { return s_pb10_running; }
/** 返回 PB10 模拟剩余步数。 */
uint32_t Stepper2_GetRemainingSteps(void) { return s_pb10_steps; }
/** 返回 PB10 模拟完成步数。 */
uint32_t Stepper2_GetCompletedSteps(void) { return 0u; }

/** 模拟 PB13 启动并记录方向和步数。 */
uint8_t PU3_Stepper_Start(uint32_t steps, uint8_t direction,
                          uint32_t start_frequency,
                          uint32_t maximum_frequency,
                          uint32_t acceleration)
{
    (void)start_frequency;
    (void)maximum_frequency;
    (void)acceleration;
    s_z_direction = direction;
    s_z_steps = steps;
    s_z_running = s_allow_start;
    return 1u;
}

/** 模拟 PB13 停止。 */
void PU3_Stepper_Stop(void) { s_z_running = 0u; }
/** 返回 PB13 模拟运行态。 */
uint8_t PU3_Stepper_IsRunning(void) { return s_z_running; }
/** 返回 PB13 模拟剩余步数。 */
uint32_t PU3_Stepper_GetRemainingSteps(void) { return s_z_steps; }
/** 返回 PB13 模拟完成步数。 */
uint32_t PU3_Stepper_GetCompletedSteps(void) { return 0u; }

/** 验证逻辑 XYZ 到已确认 STEP/DIR 驱动的绑定、步数和真实运行态校验。 */
int main(void)
{
    /* 现场确认物理 X 使用 PB10 与 DIR1(Q4)。 */
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_X, 1, 1427u, 1000u) == 1u);
    TEST_CHECK(s_pb10_steps == 1427u);
    TEST_CHECK(s_pb10_direction == 1u);
    TEST_CHECK(RobotArmDriver_IsBusy(ROBOT_AXIS_X) == 1u);
    TEST_CHECK(RobotArmDriver_GetRemainingSteps(ROBOT_AXIS_X) == 1427u);
    RobotArmDriver_Stop(ROBOT_AXIS_X);
    TEST_CHECK(s_pb10_running == 0u);

    /* 现场确认物理 Y 使用 PB11 与 DIR2(Q5)。 */
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Y, -1, 23u, 1000u) == 1u);
    TEST_CHECK(s_pb11_steps == 23u);
    TEST_CHECK((HC595Data[1] & (1u << 5)) == 0u);
    TEST_CHECK(RobotArmDriver_IsBusy(ROBOT_AXIS_Y) == 1u);
    TEST_CHECK(RobotArmDriver_GetRemainingSteps(ROBOT_AXIS_Y) == 23u);
    RobotArmDriver_Stop(ROBOT_AXIS_Y);
    TEST_CHECK(s_pb11_running == 0u);

    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Y, 1, 23u, 1000u) == 1u);
    TEST_CHECK((HC595Data[1] & (1u << 5)) != 0u);
    RobotArmDriver_Stop(ROBOT_AXIS_Y);

    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Z, -1, 23u, 1000u) == 1u);
    TEST_CHECK(s_z_steps == 23u);
    TEST_CHECK(s_z_direction == 0u);
    RobotArmDriver_Stop(ROBOT_AXIS_Z);

    /* 底层函数即使返回成功，未进入 running 也必须由适配器判为失败。 */
    s_allow_start = 0u;
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_X, 1, 1u, 1000u) == 0u);
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Y, 1, 1u, 1000u) == 0u);
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Z, 1, 1u, 1000u) == 0u);
    return s_failure;
}
