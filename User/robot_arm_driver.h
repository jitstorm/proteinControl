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
/**
 * 查询指定轴是否正在朝已触发的负方向 Home 限位运动。
 *
 * S1/S2/S3 触发后只禁止继续压向对应负限位，正方向必须继续允许脱离传感器。
 * DMA 分段续传中调用此函数可阻止触发后装载下一段脉冲。
 *
 * @param axis 要检查的实际机械轴。
 * @return 1 表示必须立即停止该轴，0 表示可继续当前方向。
 */
uint8_t RobotArmDriver_ShouldStopForNegativeLimit(RobotAxisId_t axis);

#endif
