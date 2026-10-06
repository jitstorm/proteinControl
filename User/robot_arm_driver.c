#include "robot_arm_driver.h"
#include "robot_arm.h"
#include "robot_arm_config.h"
#include "robot_arm_sensor.h"
#ifdef ROBOT_ARM_DRIVER_LOGIC_TEST
extern uint8_t HC595Data[4];
void ShiftRegister_WriteAll(uint8_t *data);
void stepdma_pb11_request_trap(uint32_t steps, uint32_t start,
                              uint32_t maximum, uint32_t acceleration);
void stepdma_pb11_move_home_approach(uint32_t steps, uint32_t start,
                                     uint32_t fast, uint32_t slow,
                                     uint32_t slow_zone, uint32_t acceleration);
void stepdma_pb11_stop(void);
uint8_t stepdma_pb11_is_running(void);
uint32_t stepdma_pb11_get_remaining_steps(void);
uint32_t stepdma_pb11_get_completed_steps(void);
uint8_t Stepper2_Start(uint8_t direction, uint32_t steps,
                       uint32_t target_frequency);
uint8_t Stepper2_StartWithAcceleration(uint8_t direction, uint32_t steps,
                                       uint32_t target_frequency,
                                       uint32_t acceleration);
void stepdma_pb10_move_home_approach(uint32_t steps, uint32_t start,
                                     uint32_t fast, uint32_t slow,
                                     uint32_t slow_zone, uint32_t acceleration);
void Stepper2_Stop(void);
uint8_t Stepper2_IsBusy(void);
uint32_t Stepper2_GetRemainingSteps(void);
uint32_t Stepper2_GetCompletedSteps(void);
uint8_t PU3_Stepper_Start(uint32_t steps, uint8_t direction,
                          uint32_t start_frequency,
                          uint32_t maximum_frequency,
                          uint32_t acceleration);
uint8_t PU3_Stepper_StartHomeApproach(uint32_t steps, uint8_t direction,
                                      uint32_t start, uint32_t fast,
                                      uint32_t slow, uint32_t slow_zone,
                                      uint32_t acceleration);
void PU3_Stepper_Stop(void);
uint8_t PU3_Stepper_IsRunning(void);
uint32_t PU3_Stepper_GetRemainingSteps(void);
uint32_t PU3_Stepper_GetCompletedSteps(void);
#else
#include "step_dma.h"
#include "shift_register.h"
#endif

#define ROBOT_ARM_Y_DIR_595_INDEX 1u
#define ROBOT_ARM_Y_DIR_595_MASK (1u << 5)
#define ROBOT_ARM_DRIVER_START_FREQUENCY 500u

/*
 * 三轴逻辑正方向对应的驱动器 DIR 电平。
 * 实机确认某轴运动方向相反时，只修改对应宏为 0u；不要改坐标、复位状态机或 DMA。
 */
#define ROBOT_ARM_X_POSITIVE_DIR_LEVEL 1u
#define ROBOT_ARM_Y_POSITIVE_DIR_LEVEL 1u
#define ROBOT_ARM_Z_POSITIVE_DIR_LEVEL 1u

/* 保存逻辑方向供 DMA 分段续传和传感器快照更新使用。 */
static int8_t s_robot_arm_driver_direction[ROBOT_AXIS_COUNT];
/* 已实际写入驱动器的 DIR 电平；同一 0x39 Batch 固定方向时禁止重复移位写入。 */
static int8_t s_robot_arm_driver_dir_level[ROBOT_AXIS_COUNT] = {-1, -1, -1};

/**
 * 将 RobotArm 的逻辑正负方向转换为驱动器实际 DIR 电平。
 *
 * 复位状态机仍以逻辑负方向寻找 Home 传感器；该转换仅适配电机接线或机构方向，
 * 不得用于改变逻辑坐标系。
 *
 * @param direction RobotArm 请求的逻辑方向，正数表示逻辑正方向。
 * @param positive_dir_level 该轴逻辑正方向应输出的 DIR 电平。
 * @return 本次运动应输出给步进驱动器的 0 或 1 DIR 电平。
 */
static uint8_t RobotArmDriver_GetDirectionLevel(int8_t direction,
                                                uint8_t positive_dir_level)
{
    return (direction > 0) ? positive_dir_level :
                             (positive_dir_level ? 0u : 1u);
}

/**
 * 设置实际 Y 轴（PB11/TIM5/DMA2 通道2）的 DIR2 电平。
 *
 * 仅修改第二片 74HC595 的 Q5，避免覆盖同片移位寄存器的其他硬件输出。
 *
 * @param direction_level 已完成逻辑方向转换的实际 DIR 电平。
 */
static void RobotArmDriver_SetPb11Direction(uint8_t direction_level)
{
    if (s_robot_arm_driver_dir_level[ROBOT_AXIS_Y] == (int8_t)direction_level)
    {
        return;
    }
    if (direction_level)
    {
        HC595Data[ROBOT_ARM_Y_DIR_595_INDEX] |= (uint8_t)ROBOT_ARM_Y_DIR_595_MASK;
    }
    else
    {
        HC595Data[ROBOT_ARM_Y_DIR_595_INDEX] &= (uint8_t)~ROBOT_ARM_Y_DIR_595_MASK;
    }
    ShiftRegister_WriteAll(HC595Data);
    s_robot_arm_driver_dir_level[ROBOT_AXIS_Y] = (int8_t)direction_level;
}

/**
 * 启动指定逻辑轴的既有 DMA 步进驱动。
 *
 * 在统一坐标和复位状态机请求方向后，才在此处转换为各轴实际驱动器 DIR 电平；
 * 方向配置集中于本文件，避免机械接线差异扩散到坐标、Home 或 DMA 实现。
 *
 * @param axis 要启动的实际机械轴。
 * @param direction 逻辑运动方向，正数表示坐标增加方向。
 * @param steps 本次需要输出的 STEP 脉冲数，必须大于 0。
 * @param speed 目标速度，单位为 steps/s；0 时按最低安全速度启动。
 * @return 底层驱动成功进入运行状态时返回 1；轴忙、参数无效或驱动未运行时返回 0。
 */
uint8_t RobotArmDriver_Start(RobotAxisId_t axis, int8_t direction,
                              uint32_t steps, uint32_t speed)
{
    static const uint32_t default_acceleration[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_ACCELERATION,
        ROBOT_ARM_Y_ACCELERATION,
        ROBOT_ARM_Z_ACCELERATION};
    if (axis >= ROBOT_AXIS_COUNT)
    {
        return 0u;
    }
    return RobotArmDriver_StartWithAcceleration(
        axis, direction, steps, speed, default_acceleration[axis]);
}

/**
 * 使用调用方给定的加速度启动指定逻辑轴。
 *
 * 单轴 Home 的请求加速度必须在此处分发到 X 的 PB10、Y 的 PB11 或 Z 的 PB13
 * DMA 入口；不得仅保存协议字段而仍使用固定加速度。
 *
 * @param axis 要启动的实际机械轴。
 * @param direction 逻辑正负运动方向。
 * @param steps 需要输出的 STEP 上升沿数量。
 * @param speed 目标速度，单位 steps/s。
 * @param acceleration 本次加速度，单位 steps/s^2。
 * @return 驱动已真实进入运行态返回 1，否则返回 0。
 */
uint8_t RobotArmDriver_StartWithAcceleration(
    RobotAxisId_t axis, int8_t direction, uint32_t steps, uint32_t speed,
    uint32_t acceleration)
{
    uint8_t start_result;
    if ((axis >= ROBOT_AXIS_COUNT) || (steps == 0u) || (acceleration == 0u) ||
        RobotArmDriver_IsBusy(axis))
    {
        return 0u;
    }

    if (speed == 0u)
    {
        speed = 1u;
    }

    s_robot_arm_driver_direction[axis] = direction;

    switch (axis)
    {
    case ROBOT_AXIS_X:
        /* X 轴使用实际 PB10/TIM6/DMA2 通道3，Stepper2 内部负责 DIR1(Q4)。 */
        start_result = Stepper2_StartWithAcceleration(
            RobotArmDriver_GetDirectionLevel(
                direction, ROBOT_ARM_X_POSITIVE_DIR_LEVEL), steps, speed,
            acceleration);
        return (start_result && Stepper2_IsBusy()) ? 1u : 0u;
    case ROBOT_AXIS_Y:
        /* Y 轴使用实际 PB11/TIM5/DMA2 通道2，仅在此处翻译 DIR2(Q5)。 */
        RobotArmDriver_SetPb11Direction(RobotArmDriver_GetDirectionLevel(
            direction, ROBOT_ARM_Y_POSITIVE_DIR_LEVEL));
        stepdma_pb11_request_trap(steps, ROBOT_ARM_DRIVER_START_FREQUENCY,
                                  speed, acceleration);
        return stepdma_pb11_is_running();
    case ROBOT_AXIS_Z:
        /* Z 轴沿用 PB13/TIM7/DMA2 通道4，PU3 内部负责 DIR3。 */
        start_result = PU3_Stepper_Start(
            steps, RobotArmDriver_GetDirectionLevel(
                       direction, ROBOT_ARM_Z_POSITIVE_DIR_LEVEL),
            ROBOT_ARM_DRIVER_START_FREQUENCY, speed,
            acceleration);
        return (start_result && PU3_Stepper_IsRunning()) ? 1u : 0u;
    default:
        return 0u;
    }
}

/**
 * 使用单条 DMA 速度轮廓执行已知距离的 Home 接近动作。
 *
 * 此接口仅服务 0x31 单轴 Home：PB10/PB11/PB13 分别在自己的 DMA/Timer 链路中
 * 保持连续脉冲，并在理论零点前的配置区域以低速接近 S1/S2/S3。传感器的即时
 * 停机仍由 RobotArm_OnSensorSnapshotUpdated() 统一处理。
 */
uint8_t RobotArmDriver_StartHomeApproach(
    RobotAxisId_t axis, int8_t direction, uint32_t steps, uint32_t fast_speed,
    uint32_t slow_speed, uint32_t slow_zone_steps, uint32_t acceleration)
{
    if ((axis >= ROBOT_AXIS_COUNT) || (steps == 0u) || (slow_speed == 0u) ||
        (acceleration == 0u) || RobotArmDriver_IsBusy(axis))
    {
        return 0u;
        
    }
    s_robot_arm_driver_direction[axis] = direction;
    switch (axis)
    {
    case ROBOT_AXIS_X:
        Stepper2_SetDirection(RobotArmDriver_GetDirectionLevel(
            direction, ROBOT_ARM_X_POSITIVE_DIR_LEVEL));
        stepdma_pb10_move_home_approach(steps, ROBOT_ARM_DRIVER_START_FREQUENCY,
                                        fast_speed, slow_speed, slow_zone_steps,
                                        acceleration);
        return stepdma_pb10_is_running();
    case ROBOT_AXIS_Y:
        RobotArmDriver_SetPb11Direction(RobotArmDriver_GetDirectionLevel(
            direction, ROBOT_ARM_Y_POSITIVE_DIR_LEVEL));
        stepdma_pb11_move_home_approach(steps, ROBOT_ARM_DRIVER_START_FREQUENCY,
                                        fast_speed, slow_speed, slow_zone_steps,
                                        acceleration);
        return stepdma_pb11_is_running();
    case ROBOT_AXIS_Z:
        return PU3_Stepper_StartHomeApproach(
            steps, RobotArmDriver_GetDirectionLevel(
                       direction, ROBOT_ARM_Z_POSITIVE_DIR_LEVEL),
            ROBOT_ARM_DRIVER_START_FREQUENCY, fast_speed, slow_speed,
            slow_zone_steps, acceleration);
    default:
        return 0u;
    }
}

/**
 * 按规划 Phase 的本轴起止频率启动 X/Y DMA。
 *
 * 两个频率均已由上层用 XY 主导轴比例换算。0Hz 是通信层静止边界，底层 DMA 入口
 * 会替换为可生成 ARR 的最小频率；Z 不允许进入该接口，防止误把 XY 频率施加给 Z。
 */
uint8_t RobotArmDriver_StartPhase(RobotAxisId_t axis, int8_t direction,
                                  uint32_t steps, uint32_t start_frequency,
                                  uint32_t end_frequency)
{
    if ((steps == 0u) || RobotArmDriver_IsBusy(axis))
    {
        return 0u;
    }
    if (axis == ROBOT_AXIS_X)
    {
        s_robot_arm_driver_direction[axis] = direction;
        return Stepper2_StartPhase(
            RobotArmDriver_GetDirectionLevel(
                direction, ROBOT_ARM_X_POSITIVE_DIR_LEVEL), steps,
            start_frequency, end_frequency);
    }
    if (axis == ROBOT_AXIS_Y)
    {
        s_robot_arm_driver_direction[axis] = direction;
        RobotArmDriver_SetPb11Direction(RobotArmDriver_GetDirectionLevel(
            direction, ROBOT_ARM_Y_POSITIVE_DIR_LEVEL));
        stepdma_pb11_move_phase(steps, start_frequency, end_frequency);
        return stepdma_pb11_is_running();
    }
    return 0u;
}

/** 停止指定逻辑轴的既有 DMA 步进驱动。 */
void RobotArmDriver_Stop(RobotAxisId_t axis)
{
    switch (axis)
    {
    case ROBOT_AXIS_X:
        Stepper2_Stop();
        break;
    case ROBOT_AXIS_Y:
        stepdma_pb11_stop();
        break;
    case ROBOT_AXIS_Z:
        PU3_Stepper_Stop();
        break;
    default:
        break;
    }
    if (axis < ROBOT_AXIS_COUNT)
    {
        s_robot_arm_driver_direction[axis] = 0;
    }
}

/** 查询指定逻辑轴的既有 DMA 步进驱动是否忙碌。 */
uint8_t RobotArmDriver_IsBusy(RobotAxisId_t axis)
{
    switch (axis)
    {
    case ROBOT_AXIS_X:
        return Stepper2_IsBusy();
    case ROBOT_AXIS_Y:
        return stepdma_pb11_is_running();
    case ROBOT_AXIS_Z:
        return PU3_Stepper_IsRunning();
    default:
        return 0u;
    }
}


/** 获取指定轴尚未完成的整段步数。 */
uint32_t RobotArmDriver_GetRemainingSteps(RobotAxisId_t axis)
{
    switch (axis)
    {
    case ROBOT_AXIS_X:
        return Stepper2_GetRemainingSteps();
    case ROBOT_AXIS_Y:
        return stepdma_pb11_get_remaining_steps();
    case ROBOT_AXIS_Z:
        return PU3_Stepper_GetRemainingSteps();
    default:
        return 0u;
    }
}

/** 获取指定轴已经完成的整段步数。 */
uint32_t RobotArmDriver_GetCompletedSteps(RobotAxisId_t axis)
{
    switch (axis)
    {
    case ROBOT_AXIS_X:
        return Stepper2_GetCompletedSteps();
    case ROBOT_AXIS_Y:
        return stepdma_pb11_get_completed_steps();
    case ROBOT_AXIS_Z:
        return PU3_Stepper_GetCompletedSteps();
    default:
        return 0u;
    }
}

/**
 * 检查负向轴是否应因 Home 命中停止，供三轴 DMA 续段与传感器回调共用。
 * 同步零目标第一阶段必须完整输出计算步数；搜索阶段恢复立即停止。
 * @param axis 实际 X/Y/Z 机械轴。
 * @return 应停止返回 1，正向或限定的同步第一阶段返回 0。
 */
uint8_t RobotArmDriver_ShouldStopForNegativeLimit(RobotAxisId_t axis)
{
    if ((axis >= ROBOT_AXIS_COUNT) || RobotArm_ShouldDeferHome(axis) ||
        (s_robot_arm_driver_direction[axis] >= 0))
    {
        return 0u;
    }
    return RobotArmSensor_IsTriggered((RobotArmSensorId_t)(
        ROBOT_ARM_SENSOR_X_HOME + axis));
}
