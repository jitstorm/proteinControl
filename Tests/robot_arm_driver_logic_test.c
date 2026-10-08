#include <stdint.h>
#include "robot_arm_driver.h"
#include "robot_arm_config.h"
#include "robot_arm_sensor.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_failure == 0)) s_failure = __LINE__; } while (0)

/** 模拟管理层是否处于同步零目标第一阶段。 */
static uint8_t s_defer_home;
/** 模拟 Home 传感器触发状态。 */
static uint8_t s_home_triggered;
/**
 * 返回测试指定的同步阶段，验证真实驱动的 DMA 续段保护判断。
 * @param axis 模拟机械轴，本夹具对各轴使用同一阶段。
 * @return 延后处理 Home 返回 1，否则返回 0。
 */
uint8_t RobotArm_ShouldDeferHome(RobotAxisId_t axis) { (void)axis; return s_defer_home; }

uint8_t HC595Data[4];
static int s_failure;
static uint8_t s_pb11_running;
static uint8_t s_pb10_running;
static uint8_t s_z_running;
static uint8_t s_allow_start = 1u;
static uint32_t s_pb11_steps;
static uint32_t s_pb10_steps;
static uint32_t s_z_steps;
static uint32_t s_pb11_acceleration;
static uint32_t s_z_acceleration;
static uint8_t s_pb10_direction;
static uint8_t s_z_direction;
static uint32_t s_phase_start_frequency;
static uint32_t s_phase_end_frequency;

/** 模拟 595 写入；方向影子值由测试直接检查。 */
void ShiftRegister_WriteAll(uint8_t *data) { (void)data; }

/**
 * 提供可切换的 Home 传感器状态。
 * @param sensor 模拟传感器，本夹具对各传感器使用同一触发值。
 * @return 触发时返回 1，否则返回 0。
 */
uint8_t RobotArmSensor_IsTriggered(RobotArmSensorId_t sensor)
{
    (void)sensor;
    return s_home_triggered;
}

/** 模拟实际 PU2（PB11）梯形 DMA 请求并记录步数。 */
void stepdma_pb11_request_trap(uint32_t steps, uint32_t start,
                              uint32_t maximum, uint32_t acceleration)
{
    (void)start;
    (void)maximum;
    s_pb11_steps = steps;
    s_pb11_acceleration = acceleration;
    s_pb11_running = s_allow_start;
}
/** 模拟 PB11 Phase 边界频率启动，验证驱动层未退回旧梯形入口。 */
void stepdma_pb11_move_phase(uint32_t steps, uint32_t start, uint32_t terminal,
                             uint32_t acceleration_time_ms)
{
    (void)acceleration_time_ms;
    s_pb11_steps = steps;
    s_phase_start_frequency = start;
    s_phase_end_frequency = terminal;
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
/** 模拟 PB10 Phase 边界频率启动，验证方向和频率原样下传。 */
uint8_t Stepper2_StartPhase(uint8_t direction, uint32_t steps,
                            uint32_t start, uint32_t terminal,
                            uint32_t acceleration_time_ms)
{
    (void)acceleration_time_ms;
    s_pb10_direction = direction;
    s_pb10_steps = steps;
    s_phase_start_frequency = start;
    s_phase_end_frequency = terminal;
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
    s_z_direction = direction;
    s_z_steps = steps;
    s_z_acceleration = acceleration;
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
    /* 1000 step 不得因固定 32-step 分段被向上补齐。 */
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_X, 1, 1000u, 1000u) == 1u);
    TEST_CHECK(s_pb10_steps == 1000u);
    TEST_CHECK(s_pb10_direction == 1u);
    TEST_CHECK(RobotArmDriver_IsBusy(ROBOT_AXIS_X) == 1u);
    TEST_CHECK(RobotArmDriver_GetRemainingSteps(ROBOT_AXIS_X) == 1000u);
    RobotArmDriver_Stop(ROBOT_AXIS_X);

    /* Phase 入口只用于 X/Y，必须把终止速度和加速时间直接交给对应独立 DMA。 */
    TEST_CHECK(RobotArmDriver_StartPhase(ROBOT_AXIS_X, 1, 201u, 500u, 2412u, 1000u) == 1u);
    TEST_CHECK(s_pb10_steps == 201u && s_phase_start_frequency == 500u &&
               s_phase_end_frequency == 2412u);
    RobotArmDriver_Stop(ROBOT_AXIS_X);
    TEST_CHECK(RobotArmDriver_StartPhase(ROBOT_AXIS_Y, -1, 1667u, 500u, 20000u, 1000u) == 1u);
    TEST_CHECK(s_pb11_steps == 1667u && s_phase_start_frequency == 500u &&
               s_phase_end_frequency == 20000u);
    RobotArmDriver_Stop(ROBOT_AXIS_Y);
    TEST_CHECK(RobotArmDriver_StartPhase(ROBOT_AXIS_Z, 1, 100u, 500u, 1000u, 1000u) == 0u);
    TEST_CHECK(s_pb10_running == 0u);

    /* 现场确认物理 Y 使用 PB11 与 DIR2(Q5)。 */
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Y, -1, 1000u, 1000u) == 1u);
    TEST_CHECK(s_pb11_steps == 1000u);
    TEST_CHECK(s_pb11_acceleration == ROBOT_ARM_Y_ACCELERATION);
    TEST_CHECK(ROBOT_ARM_Y_ACCELERATION == 6000u);
    TEST_CHECK((HC595Data[1] & (1u << 5)) == 0u);
    TEST_CHECK(RobotArmDriver_IsBusy(ROBOT_AXIS_Y) == 1u);
    TEST_CHECK(RobotArmDriver_GetRemainingSteps(ROBOT_AXIS_Y) == 1000u);
    RobotArmDriver_Stop(ROBOT_AXIS_Y);
    TEST_CHECK(s_pb11_running == 0u);

    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Y, 1, 23u, 1000u) == 1u);
    TEST_CHECK((HC595Data[1] & (1u << 5)) != 0u);
    RobotArmDriver_Stop(ROBOT_AXIS_Y);

    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Z, -1, 1000u, 1000u) == 1u);
    TEST_CHECK(s_z_steps == 1000u);
    TEST_CHECK(s_z_acceleration == ROBOT_ARM_Z_ACCELERATION);
    TEST_CHECK(ROBOT_ARM_Z_ACCELERATION == 3000u);
    TEST_CHECK(s_z_direction == 0u);
    RobotArmDriver_Stop(ROBOT_AXIS_Z);

    /* 真实驱动判断：第一阶段延后 Home，进入搜索后同一触发必须恢复停轴。 */
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_X, -1, 20000u, 1000u) == 1u);
    s_home_triggered = 1u;
    s_defer_home = 1u;
    TEST_CHECK(RobotArmDriver_ShouldStopForNegativeLimit(ROBOT_AXIS_X) == 0u);
    s_defer_home = 0u;
    TEST_CHECK(RobotArmDriver_ShouldStopForNegativeLimit(ROBOT_AXIS_X) == 1u);
    RobotArmDriver_Stop(ROBOT_AXIS_X);
    s_home_triggered = 0u;

    /* 底层函数即使返回成功，未进入 running 也必须由适配器判为失败。 */
    s_allow_start = 0u;
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_X, 1, 1u, 1000u) == 0u);
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Y, 1, 1u, 1000u) == 0u);
    TEST_CHECK(RobotArmDriver_Start(ROBOT_AXIS_Z, 1, 1u, 1000u) == 0u);
    return s_failure;
}
