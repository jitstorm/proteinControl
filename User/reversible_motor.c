#include "reversible_motor.h"
#include "stm32f10x.h"
#include "shift_register.h"
#include "shift_register_input.h"

#define REVERSIBLE_MOTOR_SENSOR_DEBOUNCE_MS 30u

typedef enum
{
    REV_MOTOR_BACKEND_595 = 0,
    REV_MOTOR_BACKEND_GPIO
} ReversibleMotorBackend;

typedef struct
{
    ReversibleMotorBackend backend;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
} ReversibleMotorHardware;

static const ReversibleMotorHardware g_reversible_motor_hardware[REVERSIBLE_MOTOR_COUNT] =
{
    {REV_MOTOR_BACKEND_595, 0, 0u, 0, 0u, 0, 0u},
    {REV_MOTOR_BACKEND_595, 0, 0u, 0, 0u, 0, 0u},
    {REV_MOTOR_BACKEND_GPIO, GPIOC, GPIO_Pin_14, GPIOC, GPIO_Pin_15, GPIOC, GPIO_Pin_0},
    {REV_MOTOR_BACKEND_GPIO, GPIOC, GPIO_Pin_1, GPIOC, GPIO_Pin_2, GPIOC, GPIO_Pin_3},
    {REV_MOTOR_BACKEND_GPIO, GPIOC, GPIO_Pin_5, GPIOB, GPIO_Pin_0, GPIOB, GPIO_Pin_1}
};

ReversibleMotorRuntime g_reversible_motors[REVERSIBLE_MOTOR_COUNT];
volatile uint32_t g_reversible_motor_error_count = 0u;
volatile uint8_t g_reversible_motor_last_error = 0u;



static uint8_t ReversibleMotor_IsValidId(uint8_t motor_id)
{
    return (motor_id >= 1u && motor_id <= REVERSIBLE_MOTOR_COUNT) ? 1u : 0u;
}

static uint8_t ReversibleMotor_IsDirection(ReversibleMotorState state)
{
    return (state == REV_MOTOR_FORWARD || state == REV_MOTOR_REVERSE) ? 1u : 0u;
}

static ReversibleMotorRunMode ReversibleMotor_GetRunMode(uint32_t duration_ms,
                                                         uint8_t sensor_enabled)
{
    if (sensor_enabled)
    {
        return REV_MOTOR_RUN_SENSOR;
    }
    return (duration_ms > 0u) ? REV_MOTOR_RUN_TIMED : REV_MOTOR_RUN_IMMEDIATE;
}

static const ReversibleMotorHardware *ReversibleMotor_GetHardware(uint8_t motor_id)
{
    return &g_reversible_motor_hardware[motor_id - 1u];
}

static void ReversibleMotor_HardwareEnable(uint8_t motor_id, uint8_t enable)
{
    const ReversibleMotorHardware *hardware = ReversibleMotor_GetHardware(motor_id);

    if (hardware->backend == REV_MOTOR_BACKEND_GPIO)
    {
        if (enable)
        {
            GPIO_SetBits(hardware->enable_port, hardware->enable_pin);
        }
        else
        {
            GPIO_ResetBits(hardware->enable_port, hardware->enable_pin);
        }
    }
}

static void ReversibleMotor_HardwareSetDirection(uint8_t motor_id,
                                                 ReversibleMotorState direction)
{
    const ReversibleMotorHardware *hardware = ReversibleMotor_GetHardware(motor_id);
    uint8_t bit_offset;
    uint8_t direction_mask;

    if (hardware->backend == REV_MOTOR_BACKEND_595)
    {
        bit_offset = (uint8_t)((motor_id - 1u) * 2u);
        direction_mask = (uint8_t)(0x03u << bit_offset);
        /* 先清除两个方向位，再仅置位一个方向位，保证不会出现 11。 */
        HC595Data[1] &= (uint8_t)~direction_mask;
        if (direction == REV_MOTOR_FORWARD)
        {
            HC595Data[1] |= (uint8_t)(1u << bit_offset);
        }
        else if (direction == REV_MOTOR_REVERSE)
        {
            HC595Data[1] |= (uint8_t)(1u << (bit_offset + 1u));
        }
        ShiftRegister_WriteAll(HC595Data);
        return;
    }

    /* GPIO 后端先同时关闭方向输入，再输出唯一有效方向。 */
    GPIO_ResetBits(hardware->in1_port, hardware->in1_pin);
    GPIO_ResetBits(hardware->in2_port, hardware->in2_pin);
    if (direction == REV_MOTOR_FORWARD)
    {
        GPIO_SetBits(hardware->in1_port, hardware->in1_pin);
    }
    else if (direction == REV_MOTOR_REVERSE)
    {
        GPIO_SetBits(hardware->in2_port, hardware->in2_pin);
    }
}

static void ReversibleMotor_HardwareStop(uint8_t motor_id)
{
    /* 停止 GPIO 电机时必须先关闭 EN，避免切换方向时仍驱动电机。 */
    ReversibleMotor_HardwareEnable(motor_id, 0u);
    ReversibleMotor_HardwareSetDirection(motor_id, REV_MOTOR_STOPPED);
}

static void ReversibleMotor_InitGpioHardware(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t index;
    const ReversibleMotorHardware *hardware;

    RCC_LSEConfig(RCC_LSE_OFF);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    for (index = 2u; index < REVERSIBLE_MOTOR_COUNT; index++)
    {
        hardware = &g_reversible_motor_hardware[index];
        /* 上电配置顺序固定为先预置并初始化 EN 为低，再初始化方向引脚。 */
        GPIO_ResetBits(hardware->enable_port, hardware->enable_pin);
        gpio.GPIO_Pin = hardware->enable_pin;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        gpio.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(hardware->enable_port, &gpio);
        GPIO_ResetBits(hardware->enable_port, hardware->enable_pin);

        GPIO_ResetBits(hardware->in1_port, hardware->in1_pin);
        GPIO_ResetBits(hardware->in2_port, hardware->in2_pin);
        gpio.GPIO_Pin = hardware->in1_pin;
        GPIO_Init(hardware->in1_port, &gpio);
        gpio.GPIO_Pin = hardware->in2_pin;
        GPIO_Init(hardware->in2_port, &gpio);
    }
}

static void ReversibleMotor_ClearTask(ReversibleMotorRuntime *motor,
                                      ReversibleMotorStopReason reason)
{
    motor->direction = REV_MOTOR_STOPPED;
    motor->run_mode = REV_MOTOR_RUN_STOPPED;
    motor->pending_state = REV_MOTOR_STOPPED;
    motor->pending_direction = REV_MOTOR_STOPPED;
    motor->pending_start = 0u;
    motor->sensor_enabled = 0u;
    motor->sensor_id = 0u;
    motor->sensor_level = 0u;
    motor->sensor_stable_ms = 0u;
    motor->duration_ms = 0u;
    motor->remaining_ms = 0u;
    motor->direction_dead_time_ms = 0u;
    motor->stop_reason = reason;
}

static uint8_t ReversibleMotor_IsSensorTriggered(const ReversibleMotorRuntime *motor)
{
    uint8_t sensor_index = (uint8_t)((motor->sensor_id - 1u) / 8u);
    uint8_t sensor_mask = (uint8_t)(1u << ((motor->sensor_id - 1u) % 8u));
    uint8_t sensor_level;

    sensor_index = (uint8_t)((SHIFT_REGISTER_INPUT_COUNT - 1u) - sensor_index);
    sensor_level = (inputData[sensor_index] & sensor_mask) ? 1u : 0u;
    return (sensor_level == motor->sensor_level) ? 1u : 0u;
}

static void ReversibleMotor_Stop(uint8_t motor_id, ReversibleMotorStopReason reason)
{
    ReversibleMotorRuntime *motor = &g_reversible_motors[motor_id - 1u];

    ReversibleMotor_HardwareStop(motor_id);
    motor->state = REV_MOTOR_STOPPED;
    ReversibleMotor_ClearTask(motor, reason);
}

static uint8_t ReversibleMotor_ConfigureStart(uint8_t motor_id,
                                              ReversibleMotorState direction,
                                              uint32_t duration_ms,
                                              uint8_t sensor_enabled,
                                              uint8_t sensor_id,
                                              uint8_t sensor_level)
{
    ReversibleMotorRuntime *motor;

    if (!ReversibleMotor_IsValidId(motor_id) || !ReversibleMotor_IsDirection(direction))
    {
        return 0u;
    }

    motor = &g_reversible_motors[motor_id - 1u];
    ReversibleMotor_ClearTask(motor, REV_STOP_REPLACED);
    motor->pending_state = direction;
    motor->pending_direction = direction;
    motor->pending_start = 1u;
    motor->sensor_enabled = sensor_enabled;
    motor->sensor_id = sensor_id;
    motor->sensor_level = sensor_level;
    motor->duration_ms = duration_ms;
    motor->remaining_ms = duration_ms;

    if (motor->state == REV_MOTOR_DEADTIME ||
        (ReversibleMotor_IsDirection(motor->state) && motor->state != direction))
    {
        /* 任务覆盖或反向切换均重新执行完整死区，期间不扣减定时剩余时间。 */
        ReversibleMotor_HardwareStop(motor_id);
        motor->state = REV_MOTOR_DEADTIME;
        motor->run_mode = REV_MOTOR_RUN_DEADTIME;
        motor->direction_dead_time_ms = REVERSIBLE_MOTOR_DEADTIME_MS;
        return 1u;
    }

    ReversibleMotor_HardwareSetDirection(motor_id, direction);
    ReversibleMotor_HardwareEnable(motor_id, 1u);
    motor->state = direction;
    motor->direction = direction;
    motor->run_mode = ReversibleMotor_GetRunMode(duration_ms, sensor_enabled);
    motor->pending_start = 0u;
    motor->pending_state = REV_MOTOR_STOPPED;
    motor->pending_direction = REV_MOTOR_STOPPED;
    motor->stop_reason = REV_STOP_NONE;
    return 1u;
}

/** 初始化五路正反转电机，并确保所有后端输出停止。 */
void ReversibleMotor_Init(void)
{
    uint8_t index;

    ReversibleMotor_InitGpioHardware();
    for (index = 0u; index < REVERSIBLE_MOTOR_COUNT; index++)
    {
        g_reversible_motors[index].state = REV_MOTOR_STOPPED;
        ReversibleMotor_ClearTask(&g_reversible_motors[index], REV_STOP_NONE);
        ReversibleMotor_HardwareStop((uint8_t)(index + 1u));
    }
}

/** 设置指定电机的安全方向输出，禁止两个方向输入同时为高。 */
uint8_t ReversibleMotor_SetOutput(uint8_t motor_id, ReversibleMotorState direction)
{
    if (!ReversibleMotor_IsValidId(motor_id) ||
        (direction != REV_MOTOR_STOPPED && !ReversibleMotor_IsDirection(direction)))
    {
        return 0u;
    }

    if (direction == REV_MOTOR_STOPPED)
    {
        ReversibleMotor_HardwareStop(motor_id);
    }
    else
    {
        ReversibleMotor_HardwareSetDirection(motor_id, direction);
        ReversibleMotor_HardwareEnable(motor_id, 1u);
    }
    return 1u;
}

/** 立即停止、正转或反转指定电机，并取消原有任务。 */
uint8_t ReversibleMotor_Immediate(uint8_t motor_id, uint8_t action)
{
    if (!ReversibleMotor_IsValidId(motor_id) || action > 2u)
    {
        return 0u;
    }
    if (action == 0u)
    {
        ReversibleMotor_Stop(motor_id, REV_STOP_NONE);
        return 1u;
    }
    return ReversibleMotor_ConfigureStart(motor_id,
                                          (action == 1u) ? REV_MOTOR_FORWARD : REV_MOTOR_REVERSE,
                                          0u, 0u, 0u, 0u);
}

/** 按指定方向启动定时任务，换向死区结束后才开始计时。 */
uint8_t ReversibleMotor_StartTimed(uint8_t motor_id, ReversibleMotorState direction,
                                   uint32_t duration_ms)
{
    if (duration_ms == 0u)
    {
        return 0u;
    }
    return ReversibleMotor_ConfigureStart(motor_id, direction, duration_ms, 0u, 0u, 0u);
}

/** 按指定方向启动传感器或超时停止任务。 */
uint8_t ReversibleMotor_StartSensorTimed(uint8_t motor_id, ReversibleMotorState direction,
                                         uint8_t sensor_id, uint8_t sensor_level,
                                         uint32_t timeout_ms)
{
    if (sensor_id < 1u || sensor_id > SHIFT_REGISTER_INPUT_CHANNEL_COUNT ||
        sensor_level > 1u || timeout_ms == 0u)
    {
        return 0u;
    }
    return ReversibleMotor_ConfigureStart(motor_id, direction, timeout_ms,
                                          1u, sensor_id, sensor_level);
}

/** 推进换向死区、传感器防抖和定时任务。 */
void ReversibleMotor_Task(uint32_t elapsed_ms)
{
    uint8_t index;
    ReversibleMotorRuntime *motor;

    if (elapsed_ms == 0u)
    {
        return;
    }

    for (index = 0u; index < REVERSIBLE_MOTOR_COUNT; index++)
    {
        motor = &g_reversible_motors[index];
        if (motor->state == REV_MOTOR_DEADTIME)
        {
            if (elapsed_ms < motor->direction_dead_time_ms)
            {
                motor->direction_dead_time_ms -= elapsed_ms;
                continue;
            }

            /* 定时运行不能使用停止动作，时间从新方向真正输出后开始计算。 */
            motor->direction_dead_time_ms = 0u;
            ReversibleMotor_HardwareSetDirection((uint8_t)(index + 1u), motor->pending_direction);
            ReversibleMotor_HardwareEnable((uint8_t)(index + 1u), 1u);
            motor->state = motor->pending_direction;
            motor->direction = motor->pending_direction;
            motor->run_mode = ReversibleMotor_GetRunMode(motor->duration_ms,
                                                         motor->sensor_enabled);
            motor->pending_state = REV_MOTOR_STOPPED;
            motor->pending_direction = REV_MOTOR_STOPPED;
            motor->pending_start = 0u;
            motor->stop_reason = REV_STOP_NONE;
            continue;
        }

        if (!ReversibleMotor_IsDirection(motor->state))
        {
            continue;
        }

        if (motor->sensor_enabled)
        {
            if (ReversibleMotor_IsSensorTriggered(motor))
            {
                motor->sensor_stable_ms = (uint16_t)(motor->sensor_stable_ms + elapsed_ms);
                if (motor->sensor_stable_ms >= REVERSIBLE_MOTOR_SENSOR_DEBOUNCE_MS)
                {
                    ReversibleMotor_Stop((uint8_t)(index + 1u), REV_STOP_SENSOR);
                    continue;
                }
            }
            else
            {
                motor->sensor_stable_ms = 0u;
            }
        }

        if (motor->remaining_ms > 0u && elapsed_ms >= motor->remaining_ms)
        {
            ReversibleMotor_Stop((uint8_t)(index + 1u), REV_STOP_TIMEOUT);
        }
        else if (motor->remaining_ms > 0u)
        {
            motor->remaining_ms -= elapsed_ms;
        }
    }
}

/** 记录协议参数错误，供调试和日志读取。 */
void ReversibleMotor_RecordError(uint8_t error_code)
{
    g_reversible_motor_error_count++;
    g_reversible_motor_last_error = error_code;
}
