#ifndef __SINGLE_MOTOR_H
#define __SINGLE_MOTOR_H

#include <stdint.h>

#define SINGLE_MOTOR_COUNT 19u

/** 单向电机的运行状态，区分定时、传感器等待和最终停止原因。 */
typedef enum
{
    /** 当前 MT 输出已关闭，没有待执行的运行任务。 */
    MOTOR_STOPPED = 0,
    /** 当前 MT 由 0x05 立即启动，不附带定时或传感器停止条件。 */
    MOTOR_RUNNING,
    /** 0x07 启动时传感器已经处于触发电平，正在等待旧触发先释放。 */
    MOTOR_WAIT_SENSOR_RELEASE,
    /** 已观察到传感器未触发，正在等待一次新的配置电平触发。 */
    MOTOR_SENSOR_ARMED,
    /** 定时最大运行时间到达，为保护机械动作而停止 MT 输出。 */
    MOTOR_TIMEOUT,
    /** 已确认一次新的传感器触发并关闭 MT 输出。 */
    MOTOR_SENSOR_TRIGGERED
} MotorRunState;

/**
 * 单台单向电机的独立非阻塞运行上下文。
 *
 * 每个 MT1~MT19 各自保存 0x07 的传感器和定时参数，避免多台电机同时
 * 等待不同传感器时互相覆盖。remaining_ms 是从电机实际启动开始递减的
 * 最大运行时间保护。
 */
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

/** 初始化 19 路单向电机的软件状态，并取消所有遗留运行任务。 */
void SingleMotor_Init(void);
/** 设置指定 MT1~MT19 的输出，不改变其它单向电机的输出位。 */
uint8_t SingleMotor_Set(uint8_t motor_id, uint8_t enable);
/** 处理 0x05 的立即启动或停止；停止时会清理该 MT 的 0x07 等待状态。 */
uint8_t SingleMotor_Immediate(uint8_t motor_id, uint8_t action);
/** 按 0x06 已定义的时间换算结果启动指定 MT 的独立定时任务。 */
uint8_t SingleMotor_StartTimed(uint8_t motor_id, uint32_t duration_ms);
/**
 * 启动 0x07 的传感器加定时联合停止任务。
 *
 * 启动瞬间已经触发的传感器只代表旧状态，必须先释放后才重新 armed，
 * 因而不会错误地立即停止刚启动的电机。duration_ms 是最大运行时间保护。
 */
uint8_t SingleMotor_StartSensorTimed(uint8_t motor_id, uint8_t sensor_id,
                                     uint8_t sensor_level, uint32_t timeout_ms);
/** 推进所有单向电机的非阻塞任务；新的传感器触发或最大运行时间均经统一停止入口关闭 MT 输出。 */
void SingleMotor_Task(uint32_t elapsed_ms);
/** 生成 0x08 使用的 19 路 MT 运行位图，已停止或已清理任务的 MT 位为 0。 */
void SingleMotor_GetRunningBitmap(uint8_t *bitmap);
/** 记录协议参数错误，供诊断读取，不改变任何 MT 输出。 */
void SingleMotor_RecordError(uint8_t error_code);

#endif
