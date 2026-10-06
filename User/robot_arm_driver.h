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
 * 使用本次请求的加速度启动指定逻辑轴的 DMA 步进驱动。
 *
 * 仅由单轴 Home 使用，使 0x31 的加速度字段实际控制 PB10/PB11/PB13 对应的
 * X/Y/Z 机械轴；普通移动仍应调用 RobotArmDriver_Start 保持各轴默认配置。
 *
 * @param axis 要启动的 X、Y 或 Z 机械轴。
 * @param direction 逻辑正负运动方向。
 * @param steps 本次需要输出的 STEP 上升沿数量。
 * @param speed 目标速度，单位 steps/s。
 * @param acceleration 本次加速度，单位 steps/s^2，必须大于 0。
 * @return 底层驱动真实进入运行态时返回 1；参数无效、轴忙或启动失败返回 0。
 */
uint8_t RobotArmDriver_StartWithAcceleration(
    RobotAxisId_t axis, int8_t direction, uint32_t steps, uint32_t speed,
    uint32_t acceleration);
/**
 * 启动单轴 Home 的已知距离接近轮廓。
 *
 * 底层 DMA 在同一条脉冲序列中完成起步、快速段、按 acceleration 平滑降至
 * slow_speed，以及最后 slow_zone_steps 的低速保持；不能用停机重启代替降速。
 *
 * @param axis 实际 X/Y/Z 机械轴。
 * @param direction 指向对应 Home 传感器的逻辑负方向。
 * @param steps 当前可信坐标到理论零点的 STEP 数。
 * @param fast_speed Home 前段最高速度，单位 steps/s。
 * @param slow_speed 末段接近传感器的速度，单位 steps/s。
 * @param slow_zone_steps 理论零点前保持低速的距离，单位 STEP。
 * @param acceleration 本次 Home 加减速率，单位 steps/s^2。
 * @return DMA 已开始输出 STEP 时返回 1，否则返回 0。
 */
uint8_t RobotArmDriver_StartHomeApproach(
    RobotAxisId_t axis, int8_t direction, uint32_t steps, uint32_t fast_speed,
    uint32_t slow_speed, uint32_t slow_zone_steps, uint32_t acceleration);
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
