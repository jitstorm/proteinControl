#ifndef __ROBOT_ARM_SENSOR_H__
#define __ROBOT_ARM_SENSOR_H__

#include <stdint.h>

typedef enum
{
    ROBOT_ARM_SENSOR_X_HOME = 0,
    ROBOT_ARM_SENSOR_Y_HOME,
    ROBOT_ARM_SENSOR_Z_HOME,
    ROBOT_ARM_SENSOR_Z_LOWER_LIMIT,
    ROBOT_ARM_SENSOR_COUNT
} RobotArmSensorId_t;

/** 用一轮完整 HC165 数据刷新机械臂传感器快照。 */
void RobotArmSensor_UpdateSnapshot(const uint8_t *sensor_data);
/** 查询是否已经取得过一轮完整传感器快照。 */
uint8_t RobotArmSensor_IsReady(void);
/** 按 1=触发、0=未触发的统一语义查询传感器。 */
uint8_t RobotArmSensor_IsTriggered(RobotArmSensorId_t sensor);

#endif
