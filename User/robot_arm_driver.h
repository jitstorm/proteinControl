#ifndef __ROBOT_ARM_DRIVER_H__
#define __ROBOT_ARM_DRIVER_H__

#include <stdint.h>

typedef enum
{
    ROBOT_AXIS_X = 0,
    ROBOT_AXIS_Y,
    ROBOT_AXIS_Z,
    ROBOT_AXIS_COUNT
} RobotAxisId_t;

/** 启动指定逻辑轴的既有 DMA 步进驱动。 */
uint8_t RobotArmDriver_Start(RobotAxisId_t axis, int8_t direction,
                             uint32_t steps, uint32_t speed);
/** 停止指定逻辑轴的既有 DMA 步进驱动。 */
void RobotArmDriver_Stop(RobotAxisId_t axis);
/** 查询指定逻辑轴的既有 DMA 步进驱动是否忙碌。 */
uint8_t RobotArmDriver_IsBusy(RobotAxisId_t axis);
/** 获取指定轴尚未完成的整段步数。 */
uint32_t RobotArmDriver_GetRemainingSteps(RobotAxisId_t axis);
/** 获取指定轴已经完成的整段步数。 */
uint32_t RobotArmDriver_GetCompletedSteps(RobotAxisId_t axis);

#endif
