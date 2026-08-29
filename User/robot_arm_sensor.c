#include "robot_arm_sensor.h"
#include "robot_arm.h"
#include "robot_arm_config.h"
#ifdef ROBOT_ARM_LOGIC_TEST
#define SHIFT_REGISTER_INPUT_COUNT 2u
#else
#include "shift_register_input.h"
#endif

static uint8_t s_robot_arm_sensor_snapshot[SHIFT_REGISTER_INPUT_COUNT];
static uint8_t s_robot_arm_sensor_ready;

/* RAM Watch：与 V1 CMD=0x01 D0/D1 相同的低字节在前 HC165 完整输入位图。 */
volatile uint16_t dbg_hc165_input_bitmap;

/*
 * S1/S2/S3 与 V1 CMD=0x01 使用同一份完整 HC165 快照：inputData[0] bit0..bit2。
 * ShiftRegisterInput_ReadAll() 已在采样时做唯一一次低有效取反，因此这里和
 * RobotArm 状态机都只使用统一的“1 为 Active”语义，禁止再次取反或反序索引。
 */
static const uint8_t s_sensor_byte_index[ROBOT_ARM_SENSOR_COUNT] = {0u, 0u, 0u};
static const uint8_t s_sensor_bit_mask[ROBOT_ARM_SENSOR_COUNT] = {
    1u << 0, 1u << 1, 1u << 2};
static const uint8_t s_sensor_triggered_level[ROBOT_ARM_SENSOR_COUNT] = {
    ROBOT_ARM_S1_TRIGGERED_LEVEL,
    ROBOT_ARM_S2_TRIGGERED_LEVEL,
    ROBOT_ARM_S3_TRIGGERED_LEVEL};

/** 用一轮完整 HC165 数据刷新机械臂传感器快照。 */
void RobotArmSensor_UpdateSnapshot(const uint8_t *sensor_data)
{
    uint8_t index;
    if (sensor_data == 0)
    {
        return;
    }
    for (index = 0u; index < SHIFT_REGISTER_INPUT_COUNT; index++)
    {
        s_robot_arm_sensor_snapshot[index] = sensor_data[index];
    }
    dbg_hc165_input_bitmap = (uint16_t)sensor_data[0] |
                            ((uint16_t)sensor_data[1] << 8);
    s_robot_arm_sensor_ready = 1u;
    /* 新快照一到达即检查运行轴，不能等待下一轮 RobotArm_Task 才停止压限位。 */
    RobotArm_OnSensorSnapshotUpdated();
}

/** 查询是否已经取得过一轮完整传感器快照。 */
uint8_t RobotArmSensor_IsReady(void)
{
    return s_robot_arm_sensor_ready;
}

/** 按 1=触发、0=未触发的统一语义查询传感器。 */
uint8_t RobotArmSensor_IsTriggered(RobotArmSensorId_t sensor)
{
    uint8_t level;
    if ((sensor >= ROBOT_ARM_SENSOR_COUNT) || !s_robot_arm_sensor_ready)
    {
        return 0u;
    }
    level = (s_robot_arm_sensor_snapshot[s_sensor_byte_index[sensor]] &
             s_sensor_bit_mask[sensor]) ? 1u : 0u;
    return (level == s_sensor_triggered_level[sensor]) ? 1u : 0u;
}
