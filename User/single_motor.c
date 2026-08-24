#include "single_motor.h"
#include "shift_register.h"
#include "shift_register_input.h"

#define SINGLE_MOTOR_SENSOR_DEBOUNCE_MS 30u

SingleMotorControl g_single_motors[SINGLE_MOTOR_COUNT];
volatile uint32_t g_single_motor_error_count = 0u;
volatile uint8_t g_single_motor_last_error = 0u;

/*
 * 逻辑电机编号依次跳过未接入的 MT1 至 MT5，以及 DIR 专用 595。
 * 电机 1 至 8 对应 595[3] bit0 至 bit7；电机 9 至 12 对应 595[2]
 * bit0 至 bit3；电机 13 至 19 对应 595[0] bit1 至 bit7。
 */
static const uint8_t single_motor_register_index[SINGLE_MOTOR_COUNT] = {
    3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u,
    2u, 2u, 2u, 2u,
    0u, 0u, 0u, 0u, 0u, 0u, 0u
};
static const uint8_t single_motor_bit_index[SINGLE_MOTOR_COUNT] = {
    0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u,
    0u, 1u, 2u, 3u,
    1u, 2u, 3u, 4u, 5u, 6u, 7u
};

/** 校验 MT1~MT19 编号，防止访问映射表越界而误改其它输出。 */
static uint8_t SingleMotor_IsValidId(uint8_t motor_id)
{
    return (motor_id >= 1u && motor_id <= SINGLE_MOTOR_COUNT) ? 1u : 0u;
}

/**
 * 清理一台 MT 的运行上下文。
 *
 * 停止、超时、传感器完成及 0x05 覆盖任务都必须调用这里，确保 0x08 不会
 * 留下“输出已关闭但 running 仍为 1”的残余状态。
 */
static void SingleMotor_ClearTask(SingleMotorControl *motor, MotorRunState state)
{
    motor->running = 0u;
    motor->sensor_id = 0u;
    motor->sensor_level = 0u;
    motor->remaining_ms = 0u;
    motor->sensor_stable_ms = 0u;
    motor->state = state;
}

/**
 * 从现有 74HC165 输入快照读取本任务配置的传感器，并按触发电平归一化判断。
 *
 * 协议层不直接读取 GPIO；高、低电平触发共用 is_triggered 判断，避免复制两套
 * 业务状态机。
 */
static uint8_t SingleMotor_IsSensorTriggered(const SingleMotorControl *motor)
{
    uint8_t sensor_index;
    uint8_t sensor_mask;
    uint8_t sensor_level;

    /* 协议与 HC165 回包约定：第一个字节为 S1~S8，第二个字节为 S9~S16，
     * 因此传感器编号直接按 8 路一组定位，不能反转字节索引。 */
    sensor_index = (uint8_t)((motor->sensor_id - 1u) / 8u);
    sensor_mask = (uint8_t)(1u << ((motor->sensor_id - 1u) % 8u));
    sensor_level = (inputData[sensor_index] & sensor_mask) ? 1u : 0u;
    return (sensor_level == motor->sensor_level) ? 1u : 0u;
}

/**
 * 经统一入口停止一台 MT 并清理本次任务。
 *
 * @param motor_id 要停止的 MT1~MT19 编号。
 * @param state 本次停止后的最终状态，用于诊断触发、超时或手动停止原因。
 */
static void SingleMotor_Stop(uint8_t motor_id, MotorRunState state)
{
    SingleMotor_Set(motor_id, 0u);
    SingleMotor_ClearTask(&g_single_motors[motor_id - 1u], state);
}

/** 初始化 19 路单向电机的软件状态。 */
void SingleMotor_Init(void)
{
    uint8_t index;

    for (index = 0u; index < SINGLE_MOTOR_COUNT; index++)
    {
        SingleMotor_ClearTask(&g_single_motors[index], MOTOR_STOPPED);
    }
}

/** 按 1 到 19 的统一编号设置单向电机输出。 */
uint8_t SingleMotor_Set(uint8_t motor_id, uint8_t enable)
{
    uint8_t index;
    uint8_t register_index;
    uint8_t bit_index;

    if (!SingleMotor_IsValidId(motor_id))
    {
        /* 非法编号必须在访问映射表前返回，防止越界改变任何输出。 */
        return 0u;
    }

    index = (uint8_t)(motor_id - 1u);
    register_index = single_motor_register_index[index];
    bit_index = single_motor_bit_index[index];

    /* 只改动目标输出位，确保其它 18 路电机状态保持不变。 */
    if (enable)
    {
        HC595Data[register_index] |= (uint8_t)(1u << bit_index);
    }
    else
    {
        HC595Data[register_index] &= (uint8_t)~(1u << bit_index);
    }
    ShiftRegister_WriteAll(HC595Data);
    return 1u;
}

/** 立即启动或停止指定单向电机，并取消旧任务。 */
uint8_t SingleMotor_Immediate(uint8_t motor_id, uint8_t action)
{
    SingleMotorControl *motor;

    if (!SingleMotor_IsValidId(motor_id) || action > 1u)
    {
        return 0u;
    }

    motor = &g_single_motors[motor_id - 1u];
    /* 新立即命令覆盖该电机原有的定时和传感器任务。 */
    SingleMotor_ClearTask(motor, action ? MOTOR_RUNNING : MOTOR_STOPPED);
    motor->running = action;
    return SingleMotor_Set(motor_id, action);
}

/** 启动指定单向电机的独立毫秒定时任务。 */
uint8_t SingleMotor_StartTimed(uint8_t motor_id, uint32_t duration_ms)
{
    SingleMotorControl *motor;

    if (!SingleMotor_IsValidId(motor_id) || duration_ms == 0u)
    {
        return 0u;
    }

    motor = &g_single_motors[motor_id - 1u];
    /* 新定时任务覆盖旧任务，但不会影响其余 18 路电机。 */
    SingleMotor_ClearTask(motor, MOTOR_RUNNING);
    motor->running = 1u;
    motor->remaining_ms = duration_ms;
    return SingleMotor_Set(motor_id, 1u);
}

/**
 * 启动指定单向电机的传感器和超时联合停止任务。
 *
 * 启动时已经处于触发电平不能作为本次动作的停止条件；必须先观察到释放，
 * 再等待一次新的触发。timeout_ms 始终是最大运行时间保护，避免传感器未产生
 * 新触发时电机持续运行。
 */
uint8_t SingleMotor_StartSensorTimed(uint8_t motor_id, uint8_t sensor_id,
                                     uint8_t sensor_level, uint32_t timeout_ms)
{
    SingleMotorControl *motor;

    if (!SingleMotor_IsValidId(motor_id) || sensor_id < 1u ||
        sensor_id > SHIFT_REGISTER_INPUT_CHANNEL_COUNT || sensor_level > 1u ||
        timeout_ms == 0u)
    {
        return 0u;
    }

    motor = &g_single_motors[motor_id - 1u];
    /* 每台 MT 保存自己的上下文，多个传感器任务不会互相覆盖。 */
    SingleMotor_ClearTask(motor, MOTOR_SENSOR_ARMED);
    motor->running = 1u;
    motor->sensor_id = sensor_id;
    motor->sensor_level = sensor_level;
    motor->remaining_ms = timeout_ms;

    /*
     * 启动即触发只可能是旧的传感器状态，不能立即停机；先等待它释放。
     * 只有启动时未触发，才能直接 armed 并等待本次动作产生的新触发。
     */
    motor->state = SingleMotor_IsSensorTriggered(motor) ?
                   MOTOR_WAIT_SENSOR_RELEASE : MOTOR_SENSOR_ARMED;
    return SingleMotor_Set(motor_id, 1u);
}

/**
 * 推进所有单向电机任务，必须由主循环周期调用。
 *
 * 0x07 在此处非阻塞地等待传感器释放、重新 armed 和新触发；无论是触发还是
 * 超时，均通过统一停止入口清理 running 与传感器状态。
 */
void SingleMotor_Task(uint32_t elapsed_ms)
{
    uint8_t index;
    SingleMotorControl *motor;

    if (elapsed_ms == 0u)
    {
        return;
    }

    for (index = 0u; index < SINGLE_MOTOR_COUNT; index++)
    {
        motor = &g_single_motors[index];
        if (!motor->running)
        {
            continue;
        }

        if (motor->state == MOTOR_WAIT_SENSOR_RELEASE)
        {
            /* 已触发的旧状态释放后才允许后续触发停止本次运行。 */
            if (!SingleMotor_IsSensorTriggered(motor))
            {
                motor->state = MOTOR_SENSOR_ARMED;
                motor->sensor_stable_ms = 0u;
            }
        }
        else if (motor->state == MOTOR_SENSOR_ARMED)
        {
            if (SingleMotor_IsSensorTriggered(motor))
            {
                motor->sensor_stable_ms = (uint16_t)(motor->sensor_stable_ms + elapsed_ms);
                if (motor->sensor_stable_ms >= SINGLE_MOTOR_SENSOR_DEBOUNCE_MS)
                {
                    /* 仅 ARMED 后的稳定触发才是本次动作的有效停止条件。 */
                    SingleMotor_Stop((uint8_t)(index + 1u), MOTOR_SENSOR_TRIGGERED);
                    continue;
                }
            }
            else
            {
                motor->sensor_stable_ms = 0u;
            }
        }

        /* remaining_ms 为 0 表示立即运行模式，不应被当作超时任务停止。 */
        if (motor->remaining_ms > 0u && elapsed_ms >= motor->remaining_ms)
        {
            /* 定时为最大运行时间保护，传感器未形成新触发时也必须关断 MT。 */
            SingleMotor_Stop((uint8_t)(index + 1u), MOTOR_TIMEOUT);
        }
        else if (motor->remaining_ms > 0u)
        {
            motor->remaining_ms -= elapsed_ms;
        }
    }
}

/** 生成 19 路单向电机的运行状态位图。 */
void SingleMotor_GetRunningBitmap(uint8_t *bitmap)
{
    uint8_t index;

    bitmap[0] = 0u;
    bitmap[1] = 0u;
    bitmap[2] = 0u;
    for (index = 0u; index < SINGLE_MOTOR_COUNT; index++)
    {
        if (g_single_motors[index].running)
        {
            bitmap[index / 8u] |= (uint8_t)(1u << (index % 8u));
        }
    }
}

/** 记录协议参数错误，供诊断读取。 */
void SingleMotor_RecordError(uint8_t error_code)
{
    g_single_motor_error_count++;
    g_single_motor_last_error = error_code;
}
