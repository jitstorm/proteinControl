#ifndef __SINGLE_MOTOR_H
#define __SINGLE_MOTOR_H

#include <stdint.h>

#define SINGLE_MOTOR_COUNT 19u

/** 单向电机的运行状态，用于区分停止原因。 */
typedef enum
{
    MOTOR_STOPPED = 0,
    MOTOR_RUNNING,
    MOTOR_WAIT_SENSOR,
    MOTOR_TIMEOUT,
    MOTOR_SENSOR_TRIGGERED
} MotorRunState;

/** 单向电机的独立运行任务状态。 */
typedef struct
{
    uint8_t running;
    uint8_t sensor_id;
    uint8_t sensor_level;
    uint32_t remaining_ms;
    uint16_t sensor_stable_ms;
    MotorRunState state;
} SingleMotorControl;

extern SingleMotorControl g_single_motors[SINGLE_MOTOR_COUNT];
extern volatile uint32_t g_single_motor_error_count;
extern volatile uint8_t g_single_motor_last_error;

/** 初始化 19 路单向电机的软件状态。 */
void SingleMotor_Init(void);
/** 按 1 到 19 的统一编号设置单向电机输出。 */
uint8_t SingleMotor_Set(uint8_t motor_id, uint8_t enable);
/** 立即启动或停止指定单向电机，并取消旧任务。 */
uint8_t SingleMotor_Immediate(uint8_t motor_id, uint8_t action);
/** 启动指定单向电机的独立毫秒定时任务。 */
uint8_t SingleMotor_StartTimed(uint8_t motor_id, uint32_t duration_ms);
/** 启动指定单向电机的传感器和超时联合停止任务。 */
uint8_t SingleMotor_StartSensorTimed(uint8_t motor_id, uint8_t sensor_id,
                                     uint8_t sensor_level, uint32_t timeout_ms);
/** 推进所有单向电机任务，必须由主循环周期调用。 */
void SingleMotor_Task(uint32_t elapsed_ms);
/** 生成 19 路单向电机的运行状态位图。 */
void SingleMotor_GetRunningBitmap(uint8_t *bitmap);
/** 记录协议参数错误，供诊断读取。 */
void SingleMotor_RecordError(uint8_t error_code);

#endif
