#include "robot_arm_driver.h"
#include "step_dma.h"
#include "shift_register.h"

#define ROBOT_ARM_X_DIR_595_INDEX 1u
#define ROBOT_ARM_X_DIR_595_MASK (1u << 4)
#define ROBOT_ARM_DRIVER_START_FREQUENCY 500u
#define ROBOT_ARM_DRIVER_ACCELERATION 100000u

/* 此处集中保留既有三轴硬件差异，RobotArm 核心不接触 GPIO、DMA 或定时器。 */
static void RobotArmDriver_SetXDirection(int8_t direction)
{
    if (direction > 0)
    {
        HC595Data[ROBOT_ARM_X_DIR_595_INDEX] |= (uint8_t)ROBOT_ARM_X_DIR_595_MASK;
    }
    else
    {
        HC595Data[ROBOT_ARM_X_DIR_595_INDEX] &= (uint8_t)~ROBOT_ARM_X_DIR_595_MASK;
    }
    ShiftRegister_WriteAll(HC595Data);
}

/** 启动指定逻辑轴的既有 DMA 步进驱动。 */
uint8_t RobotArmDriver_Start(RobotAxisId_t axis, int8_t direction,
                             uint32_t steps, uint32_t speed)
{
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
        /* X 轴沿用 PB11/TIM5/DMA2 通道2，仅在此处翻译 DIR1。 */
        RobotArmDriver_SetXDirection(direction);
        stepdma_pb11_request_trap(steps, ROBOT_ARM_DRIVER_START_FREQUENCY,
                                  speed, ROBOT_ARM_DRIVER_ACCELERATION);
        return 1u;
    case ROBOT_AXIS_Y:
        /* Y 轴沿用 PB10/TIM6/DMA2 通道3，Stepper2 内部负责 DIR2。 */
        return Stepper2_Start((direction > 0) ? 1u : 0u, steps, speed);
    case ROBOT_AXIS_Z:
        /* Z 轴沿用 PB13/TIM7/DMA2 通道4，PU3 内部负责 DIR3。 */
        return PU3_Stepper_Start(steps, (direction > 0) ? 1u : 0u,
                                 ROBOT_ARM_DRIVER_START_FREQUENCY, speed,
                                 ROBOT_ARM_DRIVER_ACCELERATION);
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
        stepdma_pb11_stop();
        break;
    case ROBOT_AXIS_Y:
        Stepper2_Stop();
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
        return stepdma_pb11_is_running();
    case ROBOT_AXIS_Y:
        return Stepper2_IsBusy();
    case ROBOT_AXIS_Z:
        return PU3_Stepper_IsRunning();
    default:
        return 0u;
    }
}
