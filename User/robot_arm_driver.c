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
        /* X 轴沿用 PB11/TIM5/DMA2 通道2，仅在此处翻译 DIR1。 */
        RobotArmDriver_SetXDirection(direction);
        stepdma_pb11_request_trap(steps, ROBOT_ARM_DRIVER_START_FREQUENCY,
                                  speed, ROBOT_ARM_DRIVER_ACCELERATION);
        return stepdma_pb11_is_running();
    case ROBOT_AXIS_Y:
        /* Y 轴沿用 PB10/TIM6/DMA2 通道3，Stepper2 内部负责 DIR2。 */
        start_result = Stepper2_Start((direction > 0) ? 1u : 0u, steps, speed);
        return (start_result && Stepper2_IsBusy()) ? 1u : 0u;
    case ROBOT_AXIS_Z:
        /* Z 轴沿用 PB13/TIM7/DMA2 通道4，PU3 内部负责 DIR3。 */
        start_result = PU3_Stepper_Start(
            steps, (direction > 0) ? 1u : 0u,
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

/** 获取指定轴尚未完成的整段步数。 */
uint32_t RobotArmDriver_GetRemainingSteps(RobotAxisId_t axis)
{
    switch (axis)
    {
    case ROBOT_AXIS_X:
        return stepdma_pb11_get_remaining_steps();
    case ROBOT_AXIS_Y:
        return Stepper2_GetRemainingSteps();
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
        return stepdma_pb11_get_completed_steps();
    case ROBOT_AXIS_Y:
        return Stepper2_GetCompletedSteps();
    case ROBOT_AXIS_Z:
        return PU3_Stepper_GetCompletedSteps();
    default:
        return 0u;
    }
}
