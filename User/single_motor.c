#include "single_motor.h"
#include "shift_register.h"
#include "shift_register_input.h"

#define SINGLE_MOTOR_SENSOR_DEBOUNCE_MS 30u

SingleMotorControl g_single_motors[SINGLE_MOTOR_COUNT];
volatile uint32_t g_single_motor_error_count = 0u;
volatile uint8_t g_single_motor_last_error = 0u;

/*
 * 逻辑电机编号依次跳过未接入的 MT1 至 MT5，以及 DIR 专用 595。
 * 电机 1 至 7 对应 595[0] bit1 至 bit7；电机 8 至 11 对应 595[2]
 * bit0 至 bit3；电机 12 至 19 对应 595[3] bit0 至 bit7。
 */
static const uint8_t single_motor_register_index[SINGLE_MOTOR_COUNT] = {
    0u, 0u, 0u, 0u, 0u, 0u, 0u,
    2u, 2u, 2u, 2u,
    3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u
};
static const uint8_t single_motor_bit_index[SINGLE_MOTOR_COUNT] = {
    1u, 2u, 3u, 4u, 5u, 6u, 7u,
    0u, 1u, 2u, 3u,
    0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u
};

static uint8_t SingleMotor_IsValidId(uint8_t motor_id)
{
    return (motor_id >= 1u && motor_id <= SINGLE_MOTOR_COUNT) ? 1u : 0u;
}

static void SingleMotor_ClearTask(SingleMotorControl *motor, MotorRunState state)
{
    motor->running = 0u;
    motor->sensor_id = 0u;
    motor->sensor_level = 0u;
    motor->remaining_ms = 0u;
    motor->sensor_stable_ms = 0u;
    motor->state = state;
}

static uint8_t SingleMotor_IsSensorTriggered(const SingleMotorControl *motor)
{
    uint8_t sensor_index;
    uint8_t sensor_mask;
    uint8_t sensor_level;

    sensor_index = (uint8_t)((motor->sensor_id - 1u) / 8u);
    sensor_index = (uint8_t)((SHIFT_REGISTER_INPUT_COUNT - 1u) - sensor_index);
    sensor_mask = (uint8_t)(1u << ((motor->sensor_id - 1u) % 8u));
    sensor_level = (inputData[sensor_index] & sensor_mask) ? 1u : 0u;
    return (sensor_level == motor->sensor_level) ? 1u : 0u;
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

/** 启动指定单向电机的传感器和超时联合停止任务。 */
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
    /* 传感器和超时任务共用一个状态对象，先满足的停止条件优先。 */
    SingleMotor_ClearTask(motor, MOTOR_WAIT_SENSOR);
    motor->running = 1u;
    motor->sensor_id = sensor_id;
    motor->sensor_level = sensor_level;
    motor->remaining_ms = timeout_ms;
    return SingleMotor_Set(motor_id, 1u);
}

/** 推进所有单向电机任务，必须由主循环周期调用。 */
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

        /* 传感器优先检查，确保同一调度周期内先触发的传感器优先停机。 */
        if (motor->sensor_id != 0u)
        {
            if (SingleMotor_IsSensorTriggered(motor))
            {
                motor->sensor_stable_ms = (uint16_t)(motor->sensor_stable_ms + elapsed_ms);
                if (motor->sensor_stable_ms >= SINGLE_MOTOR_SENSOR_DEBOUNCE_MS)
                {
                    /* 连续稳定达到防抖时间后，锁定为传感器触发停止。 */
                    SingleMotor_Set((uint8_t)(index + 1u), 0u);
                    SingleMotor_ClearTask(motor, MOTOR_SENSOR_TRIGGERED);
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
            /* 只有传感器尚未先触发时，才按超时原因停止。 */
            SingleMotor_Set((uint8_t)(index + 1u), 0u);
            SingleMotor_ClearTask(motor, MOTOR_TIMEOUT);
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
