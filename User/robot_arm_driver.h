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
/**
 * 按本 Phase 的起止频率启动 X/Y 轴。
 *
 * 该接口仅用于已经由机械臂管理层完成坐标、限位和传感器校验的 XY Phase；Z 继续
 * 使用 RobotArmDriver_Start 的独立恒定目标速度入口。
 *
 * @param axis 目标逻辑轴，只允许 X 或 Y。
 * @param direction 已按目标增量确定的正负运动方向。
 * @param steps 本轴需要输出的 STEP 上升沿数量。
 * @param start_frequency 本轴起始频率，单位 steps/s，0 由底层映射为安全低速。
 * @param end_frequency 本轴结束频率，单位 steps/s，0 表示最后一脉冲后停止。
 * @return DMA 已真实进入运行态返回 1；轴不支持、忙碌或启动失败返回 0。
 */
uint8_t RobotArmDriver_StartPhase(RobotAxisId_t axis, int8_t direction,
                                  uint32_t steps, uint32_t start_frequency,
                                  uint32_t end_frequency);
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
