#include "robot_arm_driver.h"
#ifdef ROBOT_ARM_DRIVER_LOGIC_TEST
extern uint8_t HC595Data[4];
void ShiftRegister_WriteAll(uint8_t *data);
void stepdma_pb11_request_trap(uint32_t steps, uint32_t start,
                              uint32_t maximum, uint32_t acceleration);
void stepdma_pb11_stop(void);
uint8_t stepdma_pb11_is_running(void);
uint32_t stepdma_pb11_get_remaining_steps(void);
uint32_t stepdma_pb11_get_completed_steps(void);
uint8_t Stepper2_Start(uint8_t direction, uint32_t steps,
                       uint32_t target_frequency);
void Stepper2_Stop(void);
uint8_t Stepper2_IsBusy(void);
uint32_t Stepper2_GetRemainingSteps(void);
uint32_t Stepper2_GetCompletedSteps(void);
uint8_t PU3_Stepper_Start(uint32_t steps, uint8_t direction,
                          uint32_t start_frequency,
                          uint32_t maximum_frequency,
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
#define ROBOT_ARM_DRIVER_ACCELERATION 100000u

/*
 * 三轴逻辑正方向对应的驱动器 DIR 电平。
 * 实机确认某轴运动方向相反时，只修改对应宏为 0u；不要改坐标、复位状态机或 DMA。
 */
#define ROBOT_ARM_X_POSITIVE_DIR_LEVEL 1u
#define ROBOT_ARM_Y_POSITIVE_DIR_LEVEL 1u
#define ROBOT_ARM_Z_POSITIVE_DIR_LEVEL 1u

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
 * 设置实际 PU2（PB11）的 DIR2 电平。
 *
 * 仅修改第二片 74HC595 的 Q5，避免覆盖同片移位寄存器的其他硬件输出。
 *
 * @param direction_level 已完成逻辑方向转换的实际 DIR 电平。
 */
static void RobotArmDriver_SetPb11Direction(uint8_t direction_level)
{
    if (direction_level)
    {
        HC595Data[ROBOT_ARM_Y_DIR_595_INDEX] |= (uint8_t)ROBOT_ARM_Y_DIR_595_MASK;
    }
    else
    {
        HC595Data[ROBOT_ARM_Y_DIR_595_INDEX] &= (uint8_t)~ROBOT_ARM_Y_DIR_595_MASK;
    }
    ShiftRegister_WriteAll(HC595Data);
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
    uint8_t start_result;
    if ((axis >= ROBOT_AXIS_COUNT) || (steps == 0u) || RobotArmDriver_IsBusy(axis))
    {
        return 0u;
    }

    if (speed == 0u)
    {
        speed = 1u;
    }

    switch (axis)
    {
    case ROBOT_AXIS_X:
        /* X 轴使用实际 PU1：PB10/TIM6/DMA2 通道3，Stepper2 内部负责 DIR1。 */
        start_result = Stepper2_Start(
            RobotArmDriver_GetDirectionLevel(
                direction, ROBOT_ARM_X_POSITIVE_DIR_LEVEL), steps, speed);
        return (start_result && Stepper2_IsBusy()) ? 1u : 0u;
    case ROBOT_AXIS_Y:
        /* Y 轴使用实际 PU2：PB11/TIM5/DMA2 通道2，仅在此处翻译 DIR2。 */
        RobotArmDriver_SetPb11Direction(RobotArmDriver_GetDirectionLevel(
            direction, ROBOT_ARM_Y_POSITIVE_DIR_LEVEL));
        stepdma_pb11_request_trap(steps, ROBOT_ARM_DRIVER_START_FREQUENCY,
                                  speed, ROBOT_ARM_DRIVER_ACCELERATION);
        return stepdma_pb11_is_running();
    case ROBOT_AXIS_Z:
        /* Z 轴沿用 PB13/TIM7/DMA2 通道4，PU3 内部负责 DIR3。 */
        start_result = PU3_Stepper_Start(
            steps, RobotArmDriver_GetDirectionLevel(
                       direction, ROBOT_ARM_Z_POSITIVE_DIR_LEVEL),
            ROBOT_ARM_DRIVER_START_FREQUENCY, speed,
            ROBOT_ARM_DRIVER_ACCELERATION);
        return (start_result && PU3_Stepper_IsRunning()) ? 1u : 0u;
    default:
        return 0u;
    }
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
