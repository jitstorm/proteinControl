#ifndef __REVERSIBLE_MOTOR_H
#define __REVERSIBLE_MOTOR_H

#include <stdint.h>

#define REVERSIBLE_MOTOR_COUNT 5u
#define REVERSIBLE_MOTOR_DEADTIME_MS 20u

/** 正反转电机的当前运行状态。 */
typedef enum
{
    REV_MOTOR_STOPPED = 0,
    REV_MOTOR_FORWARD,
    REV_MOTOR_REVERSE,
    REV_MOTOR_DEADTIME
} ReversibleMotorState;

/** 正反转电机最近一次停止或任务覆盖原因。 */
typedef enum
{
    REV_STOP_NONE = 0,
    REV_STOP_TIMEOUT,
    REV_STOP_SENSOR,
    REV_STOP_REPLACED,
    REV_STOP_PARAM_ERROR
} ReversibleMotorStopReason;

/** 正反转电机任务类型，用于区分立即、定时、传感器和换向等待状态。 */
typedef enum
{
    REV_MOTOR_RUN_STOPPED = 0,
    REV_MOTOR_RUN_IMMEDIATE,
    REV_MOTOR_RUN_TIMED,
    REV_MOTOR_RUN_SENSOR,
    REV_MOTOR_RUN_DEADTIME
} ReversibleMotorRunMode;

/** 五路正反转电机共用的运行时状态。 */
typedef struct
{
    ReversibleMotorState state;
    ReversibleMotorState direction;
    ReversibleMotorRunMode run_mode;
    ReversibleMotorState pending_state;
    ReversibleMotorState pending_direction;
    uint32_t duration_ms;
    uint32_t remaining_ms;
    uint8_t sensor_enabled;
    uint8_t sensor_id;
    uint8_t sensor_level;
    uint16_t sensor_stable_ms;
    uint8_t pending_start;
    uint32_t direction_dead_time_ms;
    ReversibleMotorStopReason stop_reason;
} ReversibleMotorRuntime;

typedef ReversibleMotorRuntime ReversibleMotorControl;

extern ReversibleMotorRuntime g_reversible_motors[REVERSIBLE_MOTOR_COUNT];
extern volatile uint32_t g_reversible_motor_error_count;
extern volatile uint8_t g_reversible_motor_last_error;

/** 初始化五路正反转电机，并确保所有后端输出停止。 */
void ReversibleMotor_Init(void);
/** 设置指定电机的安全方向输出，禁止两个方向输入同时为高。 */
uint8_t ReversibleMotor_SetOutput(uint8_t motor_id, ReversibleMotorState direction);
/** 立即停止、正转或反转指定电机，并取消原有任务。 */
uint8_t ReversibleMotor_Immediate(uint8_t motor_id, uint8_t action);
/** 按指定方向启动定时任务，换向死区结束后才开始计时。 */
uint8_t ReversibleMotor_StartTimed(uint8_t motor_id, ReversibleMotorState direction,
                                   uint32_t duration_ms);
/** 按指定方向启动传感器或超时停止任务。 */
uint8_t ReversibleMotor_StartSensorTimed(uint8_t motor_id, ReversibleMotorState direction,
                                         uint8_t sensor_id, uint8_t sensor_level,
                                         uint32_t timeout_ms);
/** 推进换向死区、传感器防抖和定时任务。 */
void ReversibleMotor_Task(uint32_t elapsed_ms);
/** 记录协议参数错误，供调试和日志读取。 */
void ReversibleMotor_RecordError(uint8_t error_code);

#endif
