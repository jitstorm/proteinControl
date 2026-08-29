#include "robot_arm.h"
#include "robot_arm_config.h"
#include "robot_arm_sensor.h"
#ifdef ROBOT_ARM_LOGIC_TEST
uint32_t millis(void);
#else
#include "time.h"
#endif

typedef struct
{
    RobotAxisState_t state;
    int32_t current_position;
    int32_t target_position;
    int32_t min_position;
    int32_t max_position;
    uint32_t default_speed;
    uint8_t homed;
    uint8_t position_valid;
    int8_t moving_direction;
    uint32_t command_steps;
    uint32_t move_start_ms;
    uint32_t move_timeout_ms;
    uint8_t active;
    RobotMoveEndReason_t end_reason;
} RobotAxis_t;

typedef struct
{
    RobotAxis_t axis[ROBOT_AXIS_COUNT];
    RobotArmState_t state;
    RobotArmOperation_t operation;
    RobotHomeState_t home_state;
    RobotMoveToState_t move_to_state;
    RobotSafeMoveState_t safe_move_state;
    RobotAxisId_t home_axis;
    uint8_t home_all_active;
    uint32_t home_start_ms;
    int32_t move_to_target[ROBOT_AXIS_COUNT];
    uint16_t move_to_speed[ROBOT_AXIS_COUNT];
    /* 0 表示 HomeAll 使用 MCU 已标定快速速度；非零仅由 0x31 单轴 Home 设置。 */
    uint16_t home_fast_speed;
    RobotMoveAxisProgress_t move_axis_progress[ROBOT_AXIS_COUNT];
    int32_t safe_move_target[ROBOT_AXIS_COUNT];
    uint16_t safe_move_speed[ROBOT_AXIS_COUNT];
    RobotMoveEndReason_t last_move_end_reason;
    int32_t error_code;
} RobotArm_t;

static RobotArm_t s_robot_arm;
static RobotArmMoveDebug_t s_move_debug;

/* RAM Watch：以下变量仅用于现场观察，不参与串口协议和运动决策。 */
volatile uint8_t dbg_robotarm_sensor_flags;
volatile uint8_t dbg_x_home_sensor_active;
volatile uint8_t dbg_y_home_sensor_active;
volatile uint8_t dbg_z_home_sensor_active;
volatile int8_t dbg_x_direction;
volatile int8_t dbg_y_direction;
volatile int8_t dbg_z_direction;
volatile RobotHomeState_t dbg_x_home_stage;
volatile RobotHomeState_t dbg_y_home_stage;
volatile RobotHomeState_t dbg_z_home_stage;
volatile uint32_t dbg_x_limit_stop_count;
volatile uint32_t dbg_y_limit_stop_count;
volatile uint32_t dbg_z_limit_stop_count;
volatile RobotMoveEndReason_t dbg_x_last_stop_reason;
volatile RobotMoveEndReason_t dbg_y_last_stop_reason;
volatile RobotMoveEndReason_t dbg_z_last_stop_reason;

/**
 * 刷新现场 RAM Watch，便于在不增加高频串口输出的前提下核对三轴限位链路。
 *
 * 变量只镜像当前完整 HC165 快照和 RobotArm 状态；不能作为控制输入，避免调试
 * 变量与正式安全决策形成第二套传感器路径。
 */
static void RobotArm_UpdateDebugWatch(void)
{
    uint8_t sensor_flags;

    dbg_x_home_sensor_active = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_X_HOME);
    dbg_y_home_sensor_active = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Y_HOME);
    dbg_z_home_sensor_active = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Z_HOME);
    sensor_flags = (uint8_t)(dbg_x_home_sensor_active |
                             (dbg_y_home_sensor_active << 1) |
                             (dbg_z_home_sensor_active << 2));
    dbg_robotarm_sensor_flags = sensor_flags;
    dbg_x_direction = s_robot_arm.axis[ROBOT_AXIS_X].moving_direction;
    dbg_y_direction = s_robot_arm.axis[ROBOT_AXIS_Y].moving_direction;
    dbg_z_direction = s_robot_arm.axis[ROBOT_AXIS_Z].moving_direction;
    dbg_x_home_stage = (s_robot_arm.home_axis == ROBOT_AXIS_X &&
                        s_robot_arm.state == ROBOT_ARM_HOMING) ?
                           s_robot_arm.home_state : ROBOT_HOME_IDLE;
    dbg_y_home_stage = (s_robot_arm.home_axis == ROBOT_AXIS_Y &&
                        s_robot_arm.state == ROBOT_ARM_HOMING) ?
                           s_robot_arm.home_state : ROBOT_HOME_IDLE;
    dbg_z_home_stage = (s_robot_arm.home_axis == ROBOT_AXIS_Z &&
                        s_robot_arm.state == ROBOT_ARM_HOMING) ?
                           s_robot_arm.home_state : ROBOT_HOME_IDLE;
}

static void RobotArm_ResetMoveDebug(void)
{
    uint8_t index;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        s_move_debug.received_current[index] = 0;
        s_move_debug.received_target[index] = 0;
        s_move_debug.received_delta[index] = 0;
        s_move_debug.axis_progress[index] = ROBOT_MOVE_AXIS_NOT_REQUIRED;
        s_move_debug.start_called[index] = 0u;
        s_move_debug.start_steps[index] = 0u;
        s_move_debug.start_result[index] = 0u;
        s_move_debug.busy_before[index] = 0u;
        s_move_debug.busy_after[index] = 0u;
    }
    s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_NONE;
}

static RobotAxis_t *RobotArm_GetAxis(RobotAxisId_t axis)
{
    if (axis >= ROBOT_AXIS_COUNT)
    {
        return 0;
    }
    return &s_robot_arm.axis[axis];
}

static RobotArmSensorId_t RobotArm_GetHomeSensor(RobotAxisId_t axis)
{
    return (RobotArmSensorId_t)(ROBOT_ARM_SENSOR_X_HOME + axis);
}

static uint8_t RobotArm_IsHomeEnabled(RobotAxisId_t axis)
{
    static const uint8_t enabled[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_ENABLED,
        ROBOT_ARM_Y_HOME_ENABLED,
        ROBOT_ARM_Z_HOME_ENABLED};
    return enabled[axis];
}

static int8_t RobotArm_GetHomeDirection(RobotAxisId_t axis)
{
    static const int8_t direction[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_DIRECTION,
        ROBOT_ARM_Y_HOME_DIRECTION,
        ROBOT_ARM_Z_HOME_DIRECTION};
    return direction[axis];
}

static uint32_t RobotArm_GetHomeMaxSteps(RobotAxisId_t axis)
{
    static const uint32_t steps[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_MAX_STEPS,
        ROBOT_ARM_Y_HOME_MAX_STEPS,
        ROBOT_ARM_Z_HOME_MAX_STEPS};
    return steps[axis];
}

static uint32_t RobotArm_GetHomeTimeout(RobotAxisId_t axis)
{
    static const uint32_t timeout[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_TIMEOUT_MS,
        ROBOT_ARM_Y_HOME_TIMEOUT_MS,
        ROBOT_ARM_Z_HOME_TIMEOUT_MS};
    return timeout[axis];
}

/**
 * 返回 HomeAll 对应轴的 MCU 标定快速寻零速度。
 *
 * @param axis 当前正在置零的 X、Y 或 Z 机械轴。
 * @return 对应轴的快速寻零速度，单位为 steps/s；未配置时返回 0。
 */
static uint32_t RobotArm_GetConfiguredHomeFastSpeed(RobotAxisId_t axis)
{
    static const uint32_t speed[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_HOME_FAST_SPEED_X,
        ROBOT_ARM_HOME_FAST_SPEED_Y,
        ROBOT_ARM_HOME_FAST_SPEED_Z};
    return (axis < ROBOT_AXIS_COUNT) ? speed[axis] : 0u;
}

static int32_t RobotArm_GetHomeOffset(RobotAxisId_t axis)
{
    static const int32_t offset[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_OFFSET,
        ROBOT_ARM_Y_HOME_OFFSET,
        ROBOT_ARM_Z_HOME_OFFSET};
    return offset[axis];
}

static uint32_t RobotArm_CalculateMoveTimeout(uint32_t steps, uint32_t speed)
{
    uint64_t estimated_ms;
    uint64_t timeout_ms;
    if (speed == 0u)
    {
        speed = 1u;
    }
    estimated_ms = ((uint64_t)steps * 1000u + speed - 1u) / speed;
    timeout_ms = estimated_ms * ROBOT_ARM_MOVE_TIMEOUT_MARGIN +
                 ROBOT_ARM_MOVE_TIMEOUT_MIN_MS;
    return (timeout_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)timeout_ms;
}

/**
 * 将本次运动速度限制在对应实际步进轴的最终安全范围内。
 *
 * 0 仅用于调用方请求该轴既有默认速度；非零请求不会写回 default_speed。
 * X(PB10)、Y(PB11)、Z(PB13) 的最高速度由 robot_arm_config.h 保守配置，
 * 以免 Android 的临时调试参数绕过驱动层的加减速保护。
 *
 * @param axis 正在启动的实际机械轴。
 * @param speed 本次请求速度，单位为 steps/s；0 表示使用既有默认速度。
 * @return 传给底层 DMA 步进驱动的安全速度，单位为 steps/s。
 */
static uint32_t RobotArm_ResolveMoveSpeed(RobotAxisId_t axis, uint32_t speed)
{
    static const uint32_t maximum_speed[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_MAX_SPEED,
        ROBOT_ARM_Y_MAX_SPEED,
        ROBOT_ARM_Z_MAX_SPEED};
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return 0u;
    }
    if (speed == 0u)
    {
        return robot_axis->default_speed;
    }
    return (speed > maximum_speed[axis]) ? maximum_speed[axis] : speed;
}

static RobotArmResult_t RobotArm_ResultFromEndReason(RobotMoveEndReason_t reason)
{
    switch (reason)
    {
    case ROBOT_MOVE_END_STOPPED:
        return ROBOT_ARM_ERR_STOPPED;
    case ROBOT_MOVE_END_SENSOR:
        return ROBOT_ARM_ERR_SENSOR;
    case ROBOT_MOVE_END_LIMIT:
        return ROBOT_ARM_ERR_LIMIT;
    case ROBOT_MOVE_END_INTERLOCK:
        return ROBOT_ARM_ERR_INTERLOCK;
    case ROBOT_MOVE_END_TIMEOUT:
        return ROBOT_ARM_ERR_MOVE_TIMEOUT;
    case ROBOT_MOVE_END_DRIVER_ERROR:
        return ROBOT_ARM_ERR_DRIVER;
    default:
        return ROBOT_ARM_OK;
    }
}

static void RobotArm_ResetCombinedStates(void)
{
    uint8_t index;
    s_robot_arm.home_state = ROBOT_HOME_IDLE;
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_IDLE;
    s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_IDLE;
    s_robot_arm.home_all_active = 0u;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_NOT_REQUIRED;
    }
}

static void RobotArm_FailAxisMove(RobotAxisId_t axis,
                                  RobotMoveEndReason_t reason)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    uint8_t was_move_to = (s_robot_arm.operation == ROBOT_OP_MOVE_TO) ? 1u : 0u;
    uint8_t was_safe_move =
        (s_robot_arm.operation == ROBOT_OP_MOVE_TO_SAFE) ? 1u : 0u;
    if (robot_axis == 0)
    {
        return;
    }
    if (RobotArmDriver_IsBusy(axis))
    {
        RobotArmDriver_Stop(axis);
    }
    robot_axis->active = 0u;
    robot_axis->position_valid = 0u;
    /* 普通运动失败只丢失当前位置，保留本次上电曾经成功 Home 的历史。 */
    robot_axis->state = ROBOT_AXIS_ERROR;
    robot_axis->end_reason = reason;
    s_robot_arm.last_move_end_reason = reason;
    s_robot_arm.error_code = (int32_t)RobotArm_ResultFromEndReason(reason);
    s_robot_arm.state = ROBOT_ARM_ERROR;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.move_to_state = was_move_to ?
                                    ROBOT_MOVE_TO_ERROR : ROBOT_MOVE_TO_IDLE;
    s_robot_arm.safe_move_state = was_safe_move ?
                                      ROBOT_SAFE_MOVE_ERROR : ROBOT_SAFE_MOVE_IDLE;
    s_robot_arm.home_state = ROBOT_HOME_IDLE;
    s_robot_arm.home_all_active = 0u;
}

static RobotArmResult_t RobotArm_CheckAxisTarget(RobotAxisId_t axis,
                                                 int32_t target)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    if ((target < robot_axis->min_position) ||
        (target > robot_axis->max_position))
    {
        return ROBOT_ARM_ERR_LIMIT;
    }
    return ROBOT_ARM_OK;
}

static uint8_t RobotArm_IsDirectionBlocked(RobotAxisId_t axis,
                                           int8_t direction)
{
    if (direction < 0 &&
        RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor(axis)))
    {
        return 1u;
    }
    return 0u;
}

static RobotArmResult_t RobotArm_CheckTargetDirection(RobotAxisId_t axis,
                                                       int32_t target)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    int8_t direction;
    if (target == robot_axis->current_position)
    {
        return ROBOT_ARM_OK;
    }
    direction = (target > robot_axis->current_position) ? 1 : -1;
    return RobotArm_IsDirectionBlocked(axis, direction) ?
               ROBOT_ARM_ERR_LIMIT : ROBOT_ARM_OK;
}

static RobotArmResult_t RobotArm_StartAxisMoveInternal(RobotAxisId_t axis,
                                                        int32_t target,
                                                        uint32_t speed)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    int64_t delta;
    uint32_t steps;
    int8_t direction;
    RobotArmResult_t result = ROBOT_ARM_OK;
    uint8_t driver_result;
    uint8_t busy_before;
    uint8_t busy_after;

    if (robot_axis == 0)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    result = RobotArm_CheckAxisTarget(axis, target);
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    delta = (int64_t)target - (int64_t)robot_axis->current_position;
    if (delta == 0)
    {
        return ROBOT_ARM_OK;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    direction = (delta > 0) ? 1 : -1;
    if (RobotArm_IsDirectionBlocked(axis, direction))
    {
        return ROBOT_ARM_ERR_LIMIT;
    }
    steps = (delta > 0) ? (uint32_t)delta : (uint32_t)(-delta);
    speed = RobotArm_ResolveMoveSpeed(axis, speed);
    busy_before = RobotArmDriver_IsBusy(axis);
    if (s_robot_arm.operation == ROBOT_OP_MOVE_TO)
    {
        s_move_debug.start_called[axis] = 1u;
        s_move_debug.start_steps[axis] = steps;
        s_move_debug.busy_before[axis] = busy_before;
    }
    driver_result = RobotArmDriver_Start(axis, direction, steps, speed);
    busy_after = RobotArmDriver_IsBusy(axis);
    if (s_robot_arm.operation == ROBOT_OP_MOVE_TO)
    {
        s_move_debug.start_result[axis] = driver_result;
        s_move_debug.busy_after[axis] = busy_after;
    }
    /* Start 返回后必须同步观察到运行态，否则本轴从未真实启动。 */
    if (!driver_result || busy_before || !busy_after)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }

    /* 只保存命令，必须由完成步数证明确认正常结束后才提交 current。 */
    robot_axis->target_position = target;
    robot_axis->moving_direction = direction;
    robot_axis->command_steps = steps;
    robot_axis->move_start_ms = millis();
    robot_axis->move_timeout_ms = RobotArm_CalculateMoveTimeout(steps, speed);
    robot_axis->state = ROBOT_AXIS_MOVING;
    robot_axis->active = 1u;
    robot_axis->end_reason = ROBOT_MOVE_END_NONE;
    return ROBOT_ARM_OK;
}

static int8_t RobotArm_ProcessAxisMove(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if ((robot_axis == 0) || !robot_axis->active)
    {
        return 0;
    }

    /* Home 状态机主动找边时不走普通限位成功/失败语义。 */
    if (RobotArm_IsDirectionBlocked(axis, robot_axis->moving_direction))
    {
        RobotArm_FailAxisMove(axis, ROBOT_MOVE_END_LIMIT);
        return -1;
    }
    if ((uint32_t)(millis() - robot_axis->move_start_ms) >=
        robot_axis->move_timeout_ms)
    {
        RobotArm_FailAxisMove(axis, ROBOT_MOVE_END_TIMEOUT);
        return -1;
    }
    if (RobotArmDriver_IsBusy(axis))
    {
        return 0;
    }

    /* Busy=0 只有同时满足 remaining=0 才能证明命令步数已经完整执行。 */
    if ((RobotArmDriver_GetRemainingSteps(axis) == 0u) &&
        (RobotArmDriver_GetCompletedSteps(axis) >= robot_axis->command_steps))
    {
        robot_axis->current_position = robot_axis->target_position;
        robot_axis->active = 0u;
        robot_axis->command_steps = 0u;
        robot_axis->state = ROBOT_AXIS_IDLE;
        robot_axis->end_reason = ROBOT_MOVE_END_COMPLETED;
        s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
        return 1;
    }

    RobotArm_FailAxisMove(axis, ROBOT_MOVE_END_DRIVER_ERROR);
    return -1;
}

static RobotArmResult_t RobotArm_StartSingleAbsolute(RobotAxisId_t axis,
                                                      int32_t target,
                                                      uint32_t speed)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    RobotArmResult_t result;
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if (robot_axis == 0)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    if (!robot_axis->position_valid)
    {
        return ROBOT_ARM_ERR_POSITION_UNKNOWN;
    }
    result = RobotArm_StartAxisMoveInternal(axis, target, speed);
    if ((result == ROBOT_ARM_OK) && robot_axis->active)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_MOVE_X + axis);
        s_robot_arm.state = ROBOT_ARM_MOVING;
        s_robot_arm.error_code = 0;
    }
    return result;
}

/**
 * 基于指定轴已确认的逻辑坐标启动相对步进运动。
 *
 * 相对目标必须由可信的 current_position 推导。STOP、限位、超时或驱动异常后，
 * 已输出的部分脉冲无法安全结算，因此 position_valid 为 0 时必须拒绝，不能使用
 * 保留下来的旧坐标继续运动。
 *
 * @param axis 要相对移动的 X、Y 或 Z 逻辑轴。
 * @param delta 相对当前可信坐标的目标增量，单位为步数。
 * @param speed 目标速度，单位为 steps/s；0 表示使用该轴默认速度。
 * @return 已接受时返回 ROBOT_ARM_OK；坐标失效返回 ROBOT_ARM_ERR_POSITION_UNKNOWN，
 * 忙碌、错误、超范围或驱动无法启动时返回对应错误。
 */
static RobotArmResult_t RobotArm_StartSingleRelative(RobotAxisId_t axis,
                                                      int32_t delta,
                                                      uint32_t speed)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    int64_t target;
    RobotArmResult_t result;
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if ((robot_axis == 0) || !robot_axis->position_valid)
    {
        return ROBOT_ARM_ERR_POSITION_UNKNOWN;
    }
    if (delta == 0)
    {
        return ROBOT_ARM_OK;
    }
    target = (int64_t)robot_axis->current_position + (int64_t)delta;
    if ((target < INT32_MIN) || (target > INT32_MAX))
    {
        return ROBOT_ARM_ERR_LIMIT;
    }
    result = RobotArm_StartAxisMoveInternal(axis, (int32_t)target, speed);
    if ((result == ROBOT_ARM_OK) && robot_axis->active)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_MOVE_X + axis);
        s_robot_arm.state = ROBOT_ARM_MOVING;
        s_robot_arm.error_code = 0;
    }
    return result;
}

/**
 * 校验单阶段 Home 实际会使用的轴配置和快速寻零速度。
 *
 * 当前 Home 只会在对应传感器未触发时，以本次生效的快速速度向 Home 方向寻找零点。
 * 因此必须保留轴启用、最大寻零步数和超时保护；已禁用的反向脱离、慢速复找及其
 * 配置不得作为受理条件。0x31 传入的速度优先于 HomeAll 的轴默认快速速度。
 *
 * @param axis 需要执行 Home 的实际机械轴。
 * @param fast_speed 本次快速寻零实际会传给底层驱动的速度，单位为 steps/s。
 * @return 实际配置完整且速度在协议可表示范围内时返回 ROBOT_ARM_OK，否则返回
 *         ROBOT_ARM_ERR_CONFIG。
 */
static RobotArmResult_t RobotArm_ValidateHomeConfig(RobotAxisId_t axis,
                                                     uint32_t fast_speed)
{
    if (!RobotArm_IsHomeEnabled(axis) ||
        (RobotArm_GetHomeMaxSteps(axis) == 0u) ||
        (RobotArm_GetHomeTimeout(axis) == 0u) ||
        (fast_speed == 0u) || (fast_speed > UINT16_MAX))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    return ROBOT_ARM_OK;
}

static RobotArmResult_t RobotArm_StartHomeDriver(RobotAxisId_t axis,
                                                  int8_t direction,
                                                  uint32_t steps,
                                                  uint32_t speed)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (!RobotArmDriver_Start(axis, direction, steps, speed))
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    robot_axis->active = 1u;
    robot_axis->moving_direction = direction;
    robot_axis->command_steps = steps;
    robot_axis->end_reason = ROBOT_MOVE_END_NONE;
    return ROBOT_ARM_OK;
}

/** 返回当前 Home 唯一快速寻零阶段实际使用的速度。 */
static uint32_t RobotArm_GetActiveHomeFastSpeed(RobotAxisId_t axis)
{
    return (s_robot_arm.home_fast_speed != 0u) ?
               s_robot_arm.home_fast_speed : RobotArm_GetConfiguredHomeFastSpeed(axis);
}

static void RobotArm_FailHome(RobotArmResult_t error,
                              RobotMoveEndReason_t reason)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(s_robot_arm.home_axis);
    if (robot_axis != 0)
    {
        if (RobotArmDriver_IsBusy(s_robot_arm.home_axis))
        {
            RobotArmDriver_Stop(s_robot_arm.home_axis);
        }
        robot_axis->active = 0u;
        robot_axis->homed = 0u;
        robot_axis->position_valid = 0u;
        robot_axis->state = ROBOT_AXIS_ERROR;
        robot_axis->end_reason = reason;
    }
    s_robot_arm.last_move_end_reason = reason;
    s_robot_arm.error_code = (int32_t)error;
    s_robot_arm.state = ROBOT_ARM_ERROR;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.home_state = ROBOT_HOME_ERROR;
    s_robot_arm.home_all_active = 0u;
}

/**
 * 将指定轴切换到 Home 状态机的传感器检查阶段。
 *
 * 此处只受理任务，不输出 STEP 脉冲；下一次 RobotArm_Task() 根据 S1/S2/S3 的真实
 * 状态决定直接置零或启动快速寻零。开始前主动清除旧坐标有效性，避免重新 Home 期间
 * 的旧坐标被后续普通运动错误使用。
 *
 * @param axis 需要重新建立零点的实际机械轴。
 * @return 配置和传感器就绪时返回 ROBOT_ARM_OK；否则返回对应错误且不改变为运行态。
 */
static RobotArmResult_t RobotArm_BeginHomeAxis(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    RobotArmResult_t result = RobotArm_ValidateHomeConfig(
        axis, RobotArm_GetActiveHomeFastSpeed(axis));
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    robot_axis->homed = 0u;
    robot_axis->position_valid = 0u;
    robot_axis->state = ROBOT_AXIS_HOMING;
    robot_axis->active = 0u;
    robot_axis->end_reason = ROBOT_MOVE_END_NONE;
    s_robot_arm.home_axis = axis;
    s_robot_arm.home_start_ms = millis();
    s_robot_arm.home_state = ROBOT_HOME_CHECK_SENSOR;
    s_robot_arm.state = ROBOT_ARM_HOMING;
    return ROBOT_ARM_OK;
}

static void RobotArm_CompleteHomeAxis(void)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(s_robot_arm.home_axis);
    RobotAxisId_t completed_axis = s_robot_arm.home_axis;
    RobotArmResult_t result;

    robot_axis->current_position = RobotArm_GetHomeOffset(completed_axis);
    robot_axis->target_position = robot_axis->current_position;
    robot_axis->homed = 1u;
    robot_axis->position_valid = 1u;
    robot_axis->active = 0u;
    robot_axis->state = ROBOT_AXIS_IDLE;
    robot_axis->end_reason = ROBOT_MOVE_END_COMPLETED;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;

    if (!s_robot_arm.home_all_active)
    {
        s_robot_arm.home_state = ROBOT_HOME_DONE;
        s_robot_arm.operation = ROBOT_OP_NONE;
        s_robot_arm.state = ROBOT_ARM_IDLE;
        return;
    }

    if (completed_axis == ROBOT_AXIS_Z)
    {
        result = RobotArm_BeginHomeAxis(ROBOT_AXIS_Y);
    }
    else if (completed_axis == ROBOT_AXIS_Y)
    {
        result = RobotArm_BeginHomeAxis(ROBOT_AXIS_X);
    }
    else
    {
        s_robot_arm.home_all_active = 0u;
        s_robot_arm.home_state = ROBOT_HOME_DONE;
        s_robot_arm.operation = ROBOT_OP_NONE;
        s_robot_arm.state = ROBOT_ARM_IDLE;
        return;
    }
    if (result != ROBOT_ARM_OK)
    {
        RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
    }
}

static void RobotArm_TaskHome(void)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(s_robot_arm.home_axis);
    RobotArmSensorId_t sensor = RobotArm_GetHomeSensor(s_robot_arm.home_axis);
    int8_t home_direction = RobotArm_GetHomeDirection(s_robot_arm.home_axis);
    RobotArmResult_t result = ROBOT_ARM_OK;

    if ((uint32_t)(millis() - s_robot_arm.home_start_ms) >=
        RobotArm_GetHomeTimeout(s_robot_arm.home_axis))
    {
        RobotArm_FailHome(ROBOT_ARM_ERR_HOME_TIMEOUT, ROBOT_MOVE_END_TIMEOUT);
        return;
    }
    switch (s_robot_arm.home_state)
    {
    case ROBOT_HOME_CHECK_SENSOR:
        /*
         * 当前版本 Home 仅执行一次快速寻零。当对应 Home 传感器检测为 Active 时
         * 立即停止并置零；反向脱离和二次慢速寻零暂时禁用。传感器 Active 只禁止
         * 继续向负方向运动，不禁止后续正常正方向离开检测区域。
         */
        if (RobotArmSensor_IsTriggered(sensor))
        {
            RobotArm_CompleteHomeAxis();
        }
        else
        {
            result = RobotArm_StartHomeDriver(s_robot_arm.home_axis,
                                              home_direction,
                                              RobotArm_GetHomeMaxSteps(s_robot_arm.home_axis),
                                              RobotArm_GetActiveHomeFastSpeed(
                                                  s_robot_arm.home_axis));
            s_robot_arm.home_state = ROBOT_HOME_SEEK_FAST;
        }
        if (!RobotArmSensor_IsTriggered(sensor) && (result != ROBOT_ARM_OK))
        {
            RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
        }
        break;

    case ROBOT_HOME_SEEK_FAST:
        if (RobotArmSensor_IsTriggered(sensor))
        {
            RobotArmDriver_Stop(s_robot_arm.home_axis);
            robot_axis->active = 0u;
            RobotArm_CompleteHomeAxis();
        }
        else if (!RobotArmDriver_IsBusy(s_robot_arm.home_axis))
        {
            RobotArm_FailHome(ROBOT_ARM_ERR_HOME_TIMEOUT, ROBOT_MOVE_END_TIMEOUT);
        }
        break;

    default:
        break;
    }
}

static void RobotArm_FailCombinedMoveStart(RobotAxisId_t axis,
                                           RobotArmResult_t error,
                                           uint8_t is_safe_move)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    RobotMoveEndReason_t reason = ROBOT_MOVE_END_DRIVER_ERROR;
    if (error == ROBOT_ARM_ERR_LIMIT)
    {
        reason = ROBOT_MOVE_END_LIMIT;
    }
    else if (error == ROBOT_ARM_ERR_SENSOR)
    {
        reason = ROBOT_MOVE_END_SENSOR;
    }
    else if (error == ROBOT_ARM_ERR_INTERLOCK)
    {
        reason = ROBOT_MOVE_END_INTERLOCK;
    }
    robot_axis->state = ROBOT_AXIS_ERROR;
    robot_axis->end_reason = reason;
    s_robot_arm.last_move_end_reason = reason;
    s_robot_arm.error_code = (int32_t)error;
    s_robot_arm.state = ROBOT_ARM_ERROR;
    s_robot_arm.operation = ROBOT_OP_NONE;
    if (is_safe_move)
    {
        s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_ERROR;
    }
    else
    {
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_ERROR;
        s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_START_FAILED;
    }
}

static void RobotArm_TryCompleteMoveTo(void)
{
    uint8_t index;

    /* 所有必需轴必须真实完成；尚未启动和仍在运行都不能进入坐标校验。 */
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        s_move_debug.axis_progress[index] =
            s_robot_arm.move_axis_progress[index];
        if ((s_robot_arm.move_axis_progress[index] !=
             ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
            (s_robot_arm.move_axis_progress[index] !=
             ROBOT_MOVE_AXIS_COMPLETED))
        {
            s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_WAIT_AXIS;
            return;
        }
    }

    /* 生命周期完成后仍要求管理层和三个底层驱动均已停止。 */
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        if (s_robot_arm.axis[index].active ||
            RobotArmDriver_IsBusy((RobotAxisId_t)index))
        {
            s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_DRIVER_BUSY;
            return;
        }
    }

    /* 三轴已停止但最终坐标不一致属于驱动完成异常，不能伪报成功。 */
    if ((s_robot_arm.axis[ROBOT_AXIS_X].current_position !=
         s_robot_arm.move_to_target[ROBOT_AXIS_X]) ||
        (s_robot_arm.axis[ROBOT_AXIS_Y].current_position !=
         s_robot_arm.move_to_target[ROBOT_AXIS_Y]) ||
        (s_robot_arm.axis[ROBOT_AXIS_Z].current_position !=
         s_robot_arm.move_to_target[ROBOT_AXIS_Z]))
    {
        s_robot_arm.error_code = ROBOT_ARM_ERR_DRIVER;
        s_robot_arm.state = ROBOT_ARM_ERROR;
        s_robot_arm.operation = ROBOT_OP_NONE;
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_ERROR;
        s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_DRIVER_ERROR;
        s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_POSITION_MISMATCH;
        return;
    }

    /* 最终轴确认完成的同一轮立即清理，避免 STATUS 读到短暂的伪 Busy。 */
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_IDLE;
    s_robot_arm.state = ROBOT_ARM_IDLE;
    s_robot_arm.error_code = 0;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
    s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_COMPLETED;
}

static void RobotArm_TaskMoveTo(void)
{
    RobotArmResult_t result;
    int8_t progress;
    switch (s_robot_arm.move_to_state)
    {
    case ROBOT_MOVE_TO_X_START:
        if (s_robot_arm.move_axis_progress[ROBOT_AXIS_X] ==
            ROBOT_MOVE_AXIS_NOT_REQUIRED)
        {
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_Y_START;
            break;
        }
        result = RobotArm_StartAxisMoveInternal(
            ROBOT_AXIS_X, s_robot_arm.move_to_target[ROBOT_AXIS_X],
            s_robot_arm.move_to_speed[ROBOT_AXIS_X]);
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_X, result, 0u);
            break;
        }
        s_robot_arm.move_axis_progress[ROBOT_AXIS_X] = ROBOT_MOVE_AXIS_RUNNING;
        s_move_debug.axis_progress[ROBOT_AXIS_X] = ROBOT_MOVE_AXIS_RUNNING;
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_X_WAIT;
        break;

    case ROBOT_MOVE_TO_X_WAIT:
        if (s_robot_arm.move_axis_progress[ROBOT_AXIS_X] !=
            ROBOT_MOVE_AXIS_RUNNING)
        {
            RobotArm_FailCombinedMoveStart(
                ROBOT_AXIS_X, ROBOT_ARM_ERR_DRIVER, 0u);
            break;
        }
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_X);
        if (progress > 0)
        {
            s_robot_arm.move_axis_progress[ROBOT_AXIS_X] =
                ROBOT_MOVE_AXIS_COMPLETED;
            s_move_debug.axis_progress[ROBOT_AXIS_X] =
                ROBOT_MOVE_AXIS_COMPLETED;
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_Y_START;
        }
        break;

    case ROBOT_MOVE_TO_Y_START:
        if (s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] ==
            ROBOT_MOVE_AXIS_NOT_REQUIRED)
        {
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_Z_START;
            break;
        }
        result = RobotArm_StartAxisMoveInternal(
            ROBOT_AXIS_Y, s_robot_arm.move_to_target[ROBOT_AXIS_Y],
            s_robot_arm.move_to_speed[ROBOT_AXIS_Y]);
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_Y, result, 0u);
            break;
        }
        s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] = ROBOT_MOVE_AXIS_RUNNING;
        s_move_debug.axis_progress[ROBOT_AXIS_Y] = ROBOT_MOVE_AXIS_RUNNING;
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_Y_WAIT;
        break;

    case ROBOT_MOVE_TO_Y_WAIT:
        if (s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] !=
            ROBOT_MOVE_AXIS_RUNNING)
        {
            RobotArm_FailCombinedMoveStart(
                ROBOT_AXIS_Y, ROBOT_ARM_ERR_DRIVER, 0u);
            break;
        }
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_Y);
        if (progress > 0)
        {
            s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] =
                ROBOT_MOVE_AXIS_COMPLETED;
            s_move_debug.axis_progress[ROBOT_AXIS_Y] =
                ROBOT_MOVE_AXIS_COMPLETED;
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_Z_START;
        }
        break;

    case ROBOT_MOVE_TO_Z_START:
        if (s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] ==
            ROBOT_MOVE_AXIS_NOT_REQUIRED)
        {
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_DONE;
            RobotArm_TryCompleteMoveTo();
            break;
        }
        result = RobotArm_StartAxisMoveInternal(
            ROBOT_AXIS_Z, s_robot_arm.move_to_target[ROBOT_AXIS_Z],
            s_robot_arm.move_to_speed[ROBOT_AXIS_Z]);
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_Z, result, 0u);
            break;
        }
        s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] = ROBOT_MOVE_AXIS_RUNNING;
        s_move_debug.axis_progress[ROBOT_AXIS_Z] = ROBOT_MOVE_AXIS_RUNNING;
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_Z_WAIT;
        break;

    case ROBOT_MOVE_TO_Z_WAIT:
        if (s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] !=
            ROBOT_MOVE_AXIS_RUNNING)
        {
            RobotArm_FailCombinedMoveStart(
                ROBOT_AXIS_Z, ROBOT_ARM_ERR_DRIVER, 0u);
            break;
        }
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_Z);
        if (progress > 0)
        {
            s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] =
                ROBOT_MOVE_AXIS_COMPLETED;
            s_move_debug.axis_progress[ROBOT_AXIS_Z] =
                ROBOT_MOVE_AXIS_COMPLETED;
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_DONE;
            RobotArm_TryCompleteMoveTo();
        }
        break;

    case ROBOT_MOVE_TO_DONE:
        RobotArm_TryCompleteMoveTo();
        break;

    default:
        break;
    }
}

static RobotArmResult_t RobotArm_ValidateSafeMoveConfig(void)
{
    RobotAxis_t *z_axis = RobotArm_GetAxis(ROBOT_AXIS_Z);
    if (!ROBOT_ARM_SAFE_MOVE_ENABLED || (z_axis == 0))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    /* Safe Z 即使在普通软限位暂未启用时，也必须处于已声明的 Z 坐标范围内。 */
    if ((ROBOT_ARM_SAFE_Z_POSITION < z_axis->min_position) ||
        (ROBOT_ARM_SAFE_Z_POSITION > z_axis->max_position))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    return ROBOT_ARM_OK;
}

static void RobotArm_TaskSafeMove(void)
{
    RobotArmResult_t result;
    int8_t progress;
    RobotAxis_t *x_axis = RobotArm_GetAxis(ROBOT_AXIS_X);
    RobotAxis_t *y_axis = RobotArm_GetAxis(ROBOT_AXIS_Y);
    RobotAxis_t *z_axis = RobotArm_GetAxis(ROBOT_AXIS_Z);

    switch (s_robot_arm.safe_move_state)
    {
    case ROBOT_SAFE_MOVE_PREPARE:
        if ((x_axis->current_position == s_robot_arm.safe_move_target[ROBOT_AXIS_X]) &&
            (y_axis->current_position == s_robot_arm.safe_move_target[ROBOT_AXIS_Y]))
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_FINAL_Z_START;
        }
        else if (z_axis->current_position > ROBOT_ARM_SAFE_Z_POSITION)
        {
            /* Z 数值越大位置越低，只有当前 Z 大于 Safe Z 时才需要先抬高。 */
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_RAISE_Z_START;
        }
        else
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_X_START;
        }
        break;

    case ROBOT_SAFE_MOVE_RAISE_Z_START:
        if (z_axis->current_position <= ROBOT_ARM_SAFE_Z_POSITION)
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_X_START;
            break;
        }
        result = RobotArm_StartAxisMoveInternal(
            ROBOT_AXIS_Z, ROBOT_ARM_SAFE_Z_POSITION,
            s_robot_arm.safe_move_speed[ROBOT_AXIS_Z]);
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_Z, result, 1u);
            break;
        }
        s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_RAISE_Z_WAIT;
        break;

    case ROBOT_SAFE_MOVE_RAISE_Z_WAIT:
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_Z);
        if (progress > 0)
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_X_START;
        }
        break;

    case ROBOT_SAFE_MOVE_X_START:
        if (x_axis->current_position == s_robot_arm.safe_move_target[ROBOT_AXIS_X])
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_Y_START;
            break;
        }
        result = RobotArm_CheckTransitionSafety(
            x_axis->current_position, y_axis->current_position, z_axis->current_position,
            s_robot_arm.safe_move_target[ROBOT_AXIS_X], y_axis->current_position,
            z_axis->current_position);
        if (result == ROBOT_ARM_OK)
        {
            result = RobotArm_StartAxisMoveInternal(
                ROBOT_AXIS_X, s_robot_arm.safe_move_target[ROBOT_AXIS_X],
                s_robot_arm.safe_move_speed[ROBOT_AXIS_X]);
        }
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_X, result, 1u);
            break;
        }
        s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_X_WAIT;
        break;

    case ROBOT_SAFE_MOVE_X_WAIT:
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_X);
        if (progress > 0)
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_Y_START;
        }
        break;

    case ROBOT_SAFE_MOVE_Y_START:
        if (y_axis->current_position == s_robot_arm.safe_move_target[ROBOT_AXIS_Y])
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_FINAL_Z_START;
            break;
        }
        result = RobotArm_CheckTransitionSafety(
            x_axis->current_position, y_axis->current_position, z_axis->current_position,
            x_axis->current_position, s_robot_arm.safe_move_target[ROBOT_AXIS_Y],
            z_axis->current_position);
        if (result == ROBOT_ARM_OK)
        {
            result = RobotArm_StartAxisMoveInternal(
                ROBOT_AXIS_Y, s_robot_arm.safe_move_target[ROBOT_AXIS_Y],
                s_robot_arm.safe_move_speed[ROBOT_AXIS_Y]);
        }
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_Y, result, 1u);
            break;
        }
        s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_Y_WAIT;
        break;

    case ROBOT_SAFE_MOVE_Y_WAIT:
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_Y);
        if (progress > 0)
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_FINAL_Z_START;
        }
        break;

    case ROBOT_SAFE_MOVE_FINAL_Z_START:
        if (z_axis->current_position == s_robot_arm.safe_move_target[ROBOT_AXIS_Z])
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_DONE;
            break;
        }
        /* 最终 Z 动作必须基于已经真实到位的 X/Y 再次检查静态姿态安全。 */
        result = RobotArm_CheckPoseSafety(
            x_axis->current_position, y_axis->current_position,
            s_robot_arm.safe_move_target[ROBOT_AXIS_Z]);
        if (result == ROBOT_ARM_OK)
        {
            result = RobotArm_StartAxisMoveInternal(
                ROBOT_AXIS_Z, s_robot_arm.safe_move_target[ROBOT_AXIS_Z],
                s_robot_arm.safe_move_speed[ROBOT_AXIS_Z]);
        }
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_Z, result, 1u);
            break;
        }
        s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_FINAL_Z_WAIT;
        break;

    case ROBOT_SAFE_MOVE_FINAL_Z_WAIT:
        progress = RobotArm_ProcessAxisMove(ROBOT_AXIS_Z);
        if (progress > 0)
        {
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_DONE;
        }
        break;

    case ROBOT_SAFE_MOVE_DONE:
        if ((x_axis->current_position != s_robot_arm.safe_move_target[ROBOT_AXIS_X]) ||
            (y_axis->current_position != s_robot_arm.safe_move_target[ROBOT_AXIS_Y]) ||
            (z_axis->current_position != s_robot_arm.safe_move_target[ROBOT_AXIS_Z]) ||
            !x_axis->position_valid || !y_axis->position_valid || !z_axis->position_valid)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_Z,
                                           ROBOT_ARM_ERR_DRIVER, 1u);
        }
        else
        {
            s_robot_arm.operation = ROBOT_OP_NONE;
            s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_IDLE;
            s_robot_arm.state = ROBOT_ARM_IDLE;
            s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
        }
        break;

    default:
        break;
    }
}

/** 初始化 XYZ 机械臂管理层；不会启动任意电机。 */
void RobotArm_Init(void)
{
    uint8_t index;
    static const int32_t min_position[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_MIN_POSITION, ROBOT_ARM_Y_MIN_POSITION, ROBOT_ARM_Z_MIN_POSITION};
    static const int32_t max_position[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_MAX_POSITION, ROBOT_ARM_Y_MAX_POSITION, ROBOT_ARM_Z_MAX_POSITION};
    static const uint32_t speed[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_DEFAULT_SPEED, ROBOT_ARM_Y_DEFAULT_SPEED, ROBOT_ARM_Z_DEFAULT_SPEED};
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        s_robot_arm.axis[index].state = ROBOT_AXIS_IDLE;
        s_robot_arm.axis[index].current_position = 0;
        s_robot_arm.axis[index].target_position = 0;
        s_robot_arm.axis[index].min_position = min_position[index];
        s_robot_arm.axis[index].max_position = max_position[index];
        s_robot_arm.axis[index].default_speed = speed[index];
        /* 调试开关只提供上电后的零点坐标；运行时保护与状态机保持不变。 */
#if ROBOT_ARM_DEBUG_ASSUME_HOME
        s_robot_arm.axis[index].homed = 1u;
        s_robot_arm.axis[index].position_valid = 1u;
#else
        s_robot_arm.axis[index].homed = 0u;
        s_robot_arm.axis[index].position_valid = 0u;
#endif
        s_robot_arm.axis[index].active = 0u;
        s_robot_arm.axis[index].end_reason = ROBOT_MOVE_END_NONE;
    }
    s_robot_arm.state = ROBOT_ARM_IDLE;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_NONE;
    s_robot_arm.error_code = 0;
    RobotArm_ResetCombinedStates();
    RobotArm_ResetMoveDebug();
}

/** 在主循环中推进 Home、单轴和 MoveTo 状态机。 */
void RobotArm_Task(void)
{
    int8_t result;
    RobotAxisId_t axis;
    if ((s_robot_arm.operation == ROBOT_OP_HOME_X) ||
        (s_robot_arm.operation == ROBOT_OP_HOME_Y) ||
        (s_robot_arm.operation == ROBOT_OP_HOME_Z) ||
        (s_robot_arm.operation == ROBOT_OP_HOME_ALL))
    {
        RobotArm_TaskHome();
        return;
    }
    if (s_robot_arm.operation == ROBOT_OP_MOVE_TO)
    {
        RobotArm_TaskMoveTo();
        return;
    }
    if (s_robot_arm.operation == ROBOT_OP_MOVE_TO_SAFE)
    {
        RobotArm_TaskSafeMove();
        return;
    }
    if ((s_robot_arm.operation >= ROBOT_OP_MOVE_X) &&
        (s_robot_arm.operation <= ROBOT_OP_MOVE_Z))
    {
        axis = (RobotAxisId_t)(s_robot_arm.operation - ROBOT_OP_MOVE_X);
        result = RobotArm_ProcessAxisMove(axis);
        if (result > 0)
        {
            s_robot_arm.operation = ROBOT_OP_NONE;
            s_robot_arm.state = ROBOT_ARM_IDLE;
        }
    }
}

/** 启动 X 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveXRelative(int32_t delta, uint32_t speed)
{
    return RobotArm_StartSingleRelative(ROBOT_AXIS_X, delta, speed);
}

/** 启动 Y 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveYRelative(int32_t delta, uint32_t speed)
{
    return RobotArm_StartSingleRelative(ROBOT_AXIS_Y, delta, speed);
}

/** 启动 Z 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveZRelative(int32_t delta, uint32_t speed)
{
    return RobotArm_StartSingleRelative(ROBOT_AXIS_Z, delta, speed);
}

/** 启动 X 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveX(int32_t target, uint32_t speed)
{
    return RobotArm_StartSingleAbsolute(ROBOT_AXIS_X, target, speed);
}

/** 启动 Y 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveY(int32_t target, uint32_t speed)
{
    return RobotArm_StartSingleAbsolute(ROBOT_AXIS_Y, target, speed);
}

/** 启动 Z 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveZ(int32_t target, uint32_t speed)
{
    return RobotArm_StartSingleAbsolute(ROBOT_AXIS_Z, target, speed);
}

/**
 * 按 X、Y、Z 顺序启动一次非阻塞普通目标位置任务。
 *
 * speed 只保留在本次 MOVE_TO 运行态，绝不修改三轴 default_speed；每轴真正启动前
 * 都会再次应用各自最大安全速度和既有 DMA 加减速控制。
 *
 * @param x X 轴目标绝对逻辑坐标，单位为步数。
 * @param y Y 轴目标绝对逻辑坐标，单位为步数。
 * @param z Z 轴目标绝对逻辑坐标，单位为步数。
 * @param speed 本次三轴速度，单位为 steps/s；0 表示完全使用既有默认速度。
 * @return 命令被接受时返回 ROBOT_ARM_OK；坐标、传感器、安全检查或驱动前置条件失败时返回对应错误。
 */
RobotArmResult_t RobotArm_MoveToWithSpeed(int32_t x, int32_t y, int32_t z,
                                          uint16_t x_speed, uint16_t y_speed,
                                          uint16_t z_speed)
{
    RobotArmResult_t result;
    uint8_t index;
    int32_t targets[ROBOT_AXIS_COUNT];
    int32_t current;
    int64_t delta;
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if (!s_robot_arm.axis[ROBOT_AXIS_X].position_valid ||
        !s_robot_arm.axis[ROBOT_AXIS_Y].position_valid ||
        !s_robot_arm.axis[ROBOT_AXIS_Z].position_valid)
    {
        return ROBOT_ARM_ERR_POSITION_UNKNOWN;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    result = RobotArm_CheckAxisTarget(ROBOT_AXIS_X, x);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckAxisTarget(ROBOT_AXIS_Y, y);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckAxisTarget(ROBOT_AXIS_Z, z);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckTargetDirection(ROBOT_AXIS_X, x);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckTargetDirection(ROBOT_AXIS_Y, y);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckTargetDirection(ROBOT_AXIS_Z, z);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckPoseSafety(x, y, z);
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    targets[ROBOT_AXIS_X] = x;
    targets[ROBOT_AXIS_Y] = y;
    targets[ROBOT_AXIS_Z] = z;
    /* 三轴速度必须逐轴保留到实际启动 DMA 的时刻，不能重新合并为公共速度。 */
    s_robot_arm.move_to_speed[ROBOT_AXIS_X] = x_speed;
    s_robot_arm.move_to_speed[ROBOT_AXIS_Y] = y_speed;
    s_robot_arm.move_to_speed[ROBOT_AXIS_Z] = z_speed;
    RobotArm_ResetMoveDebug();
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        current = s_robot_arm.axis[index].current_position;
        delta = (int64_t)targets[index] - (int64_t)current;
        s_robot_arm.move_to_target[index] = targets[index];
        s_robot_arm.axis[index].target_position = targets[index];
        s_move_debug.received_current[index] = current;
        s_move_debug.received_target[index] = targets[index];
        s_move_debug.received_delta[index] = delta;
        s_robot_arm.move_axis_progress[index] =
            (delta == 0) ? ROBOT_MOVE_AXIS_NOT_REQUIRED :
                           ROBOT_MOVE_AXIS_WAIT_START;
        s_move_debug.axis_progress[index] =
            s_robot_arm.move_axis_progress[index];
    }
    if ((s_robot_arm.move_axis_progress[ROBOT_AXIS_X] ==
         ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
        (s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] ==
         ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
        (s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] ==
         ROBOT_MOVE_AXIS_NOT_REQUIRED))
    {
        s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
        s_robot_arm.error_code = 0;
        s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_COMPLETED;
        return ROBOT_ARM_OK;
    }
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_X_START;
    s_robot_arm.operation = ROBOT_OP_MOVE_TO;
    s_robot_arm.state = ROBOT_ARM_MOVING;
    s_robot_arm.error_code = 0;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_NONE;
    return ROBOT_ARM_OK;
}

/**
 * 按既有默认速度启动普通 MOVE_TO，保持旧调用和生产流程行为不变。
 *
 * @param x X 轴目标绝对逻辑坐标，单位为步数。
 * @param y Y 轴目标绝对逻辑坐标，单位为步数。
 * @param z Z 轴目标绝对逻辑坐标，单位为步数。
 * @return 命令被接受时返回 ROBOT_ARM_OK；失败时返回既有错误码。
 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z)
{
    return RobotArm_MoveToWithSpeed(x, y, z, 0u, 0u, 0u);
}

/** 按“必要时抬高 Z、移动 X/Y、最后移动 Z”的安全路径接受绝对位置任务。 */
/**
 * 以逐轴速度受理既有 Safe Move 状态机，不改变其抬 Z、X/Y、最终 Z 的动作顺序。
 *
 * @param x X 轴目标绝对坐标，单位为步数。
 * @param y Y 轴目标绝对坐标，单位为步数。
 * @param z Z 轴目标绝对坐标，单位为步数。
 * @param x_speed X 轴实际 DMA 目标速度，单位为 steps/s。
 * @param y_speed Y 轴实际 DMA 目标速度，单位为 steps/s。
 * @param z_speed Z 轴实际 DMA 目标速度，单位为 steps/s。
 * @return 命令受理时返回 ROBOT_ARM_OK；安全配置、行程或传感器检查失败时返回错误。
 */
RobotArmResult_t RobotArm_MoveToSafeWithSpeed(int32_t x, int32_t y, int32_t z,
                                              uint16_t x_speed,
                                              uint16_t y_speed,
                                              uint16_t z_speed)
{
    RobotArmResult_t result;
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if (!s_robot_arm.axis[ROBOT_AXIS_X].position_valid ||
        !s_robot_arm.axis[ROBOT_AXIS_Y].position_valid ||
        !s_robot_arm.axis[ROBOT_AXIS_Z].position_valid)
    {
        return ROBOT_ARM_ERR_POSITION_UNKNOWN;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    result = RobotArm_CheckAxisTarget(ROBOT_AXIS_X, x);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckAxisTarget(ROBOT_AXIS_Y, y);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckAxisTarget(ROBOT_AXIS_Z, z);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckPoseSafety(x, y, z);
    if (result == ROBOT_ARM_OK) result = RobotArm_ValidateSafeMoveConfig();
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    if ((RobotArm_GetX() == x) && (RobotArm_GetY() == y) && (RobotArm_GetZ() == z))
    {
        s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
        return ROBOT_ARM_OK;
    }

    /* 组合任务始终保存最终目标；各阶段只通过统一单轴 helper 改写活动轴命令。 */
    s_robot_arm.safe_move_target[ROBOT_AXIS_X] = x;
    s_robot_arm.safe_move_target[ROBOT_AXIS_Y] = y;
    s_robot_arm.safe_move_target[ROBOT_AXIS_Z] = z;
    s_robot_arm.safe_move_speed[ROBOT_AXIS_X] = x_speed;
    s_robot_arm.safe_move_speed[ROBOT_AXIS_Y] = y_speed;
    s_robot_arm.safe_move_speed[ROBOT_AXIS_Z] = z_speed;
    s_robot_arm.axis[ROBOT_AXIS_X].target_position = x;
    s_robot_arm.axis[ROBOT_AXIS_Y].target_position = y;
    s_robot_arm.axis[ROBOT_AXIS_Z].target_position = z;
    s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_PREPARE;
    s_robot_arm.operation = ROBOT_OP_MOVE_TO_SAFE;
    s_robot_arm.state = ROBOT_ARM_MOVING;
    s_robot_arm.error_code = 0;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_NONE;
    return ROBOT_ARM_OK;
}

/** 按各轴既有默认速度启动 Safe Move，保留非协议调用的兼容行为。 */
RobotArmResult_t RobotArm_MoveToSafe(int32_t x, int32_t y, int32_t z)
{
    return RobotArm_MoveToSafeWithSpeed(x, y, z, 0u, 0u, 0u);
}

/**
 * 启动指定轴的非阻塞双阶段 Home。
 *
 * 单轴 Home 复用既有 Home 状态机，并显式关闭 HomeAll 串行标志，避免当前轴
 * 完成后自动切换到其他机械轴。ACK/调用成功只表示状态机已受理；当前版本只在
 * 传感器初始 Active 或快速寻零命中 Active 后确认零点，不执行退让和慢速复找。
 *
 * @param axis 需要置零的实际机械轴，只允许 X、Y 或 Z。
 * @return 已受理时返回 ROBOT_ARM_OK；ERROR、忙碌、Home 配置或传感器不可用时
 *         返回对应错误码，且不会启动任何轴。
 */
RobotArmResult_t RobotArm_HomeAxis(RobotAxisId_t axis)
{
    RobotArmResult_t result;
    if (axis >= ROBOT_AXIS_COUNT)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    /* 单轴请求绝不能继承旧 HomeAll 的串行状态，完成当前轴后必须直接收尾。 */
    s_robot_arm.home_all_active = 0u;
    s_robot_arm.home_fast_speed = 0u;
    result = RobotArm_BeginHomeAxis(axis);
    if (result == ROBOT_ARM_OK)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_HOME_X + axis);
        s_robot_arm.error_code = 0;
    }
    return result;
}

/**
 * 以协议指定的快速速度启动单轴 Home，仅执行一次快速寻零。
 *
 * @param axis 需要置零的实际机械轴。
 * @param home_speed 第一次快速寻找 Home 传感器的速度，单位为 steps/s。
 * @return 已受理返回 ROBOT_ARM_OK；失败时返回当前配置、传感器或状态错误。
 */
RobotArmResult_t RobotArm_HomeAxisWithSpeed(RobotAxisId_t axis,
                                            uint16_t home_speed)
{
    RobotArmResult_t result;
    if (home_speed == 0u)
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    if (axis >= ROBOT_AXIS_COUNT)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    s_robot_arm.home_all_active = 0u;
    s_robot_arm.home_fast_speed = home_speed;
    result = RobotArm_BeginHomeAxis(axis);
    if (result == ROBOT_ARM_OK)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_HOME_X + axis);
        s_robot_arm.error_code = 0;
    }
    else
    {
        s_robot_arm.home_fast_speed = 0u;
    }
    return result;
}

/** 按 Z、Y、X 顺序启动非阻塞 HomeAll。 */
RobotArmResult_t RobotArm_Home(void)
{
    RobotArmResult_t result;
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    /* HomeAll 不带临时速度字段，三轴各自使用已标定的默认快速寻零速度。 */
    result = RobotArm_ValidateHomeConfig(
        ROBOT_AXIS_Z, RobotArm_GetConfiguredHomeFastSpeed(ROBOT_AXIS_Z));
    if (result == ROBOT_ARM_OK)
    {
        result = RobotArm_ValidateHomeConfig(
            ROBOT_AXIS_Y, RobotArm_GetConfiguredHomeFastSpeed(ROBOT_AXIS_Y));
    }
    if (result == ROBOT_ARM_OK)
    {
        result = RobotArm_ValidateHomeConfig(
            ROBOT_AXIS_X, RobotArm_GetConfiguredHomeFastSpeed(ROBOT_AXIS_X));
    }
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    s_robot_arm.home_all_active = 1u;
    /* HomeAll 的快速/脱离速度保持各轴 MCU 配置，不能继承 0x31 的临时字段。 */
    s_robot_arm.home_fast_speed = 0u;
    result = RobotArm_BeginHomeAxis(ROBOT_AXIS_Z);
    if (result == ROBOT_ARM_OK)
    {
        s_robot_arm.operation = ROBOT_OP_HOME_ALL;
        s_robot_arm.error_code = 0;
    }
    else
    {
        s_robot_arm.home_all_active = 0u;
    }
    return result;
}

/** 停止全部三轴并彻底结束当前组合操作。 */
void RobotArm_Stop(void)
{
    uint8_t index;
    uint8_t was_running;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        was_running = (uint8_t)(
            robot_axis->active ||
            RobotArmDriver_IsBusy((RobotAxisId_t)index) ||
            (s_robot_arm.move_axis_progress[index] ==
             ROBOT_MOVE_AXIS_RUNNING));

        /* STOP 对三轴无条件下发，清除与管理层标志不同步的底层 running。 */
        RobotArmDriver_Stop((RobotAxisId_t)index);
        if (was_running)
        {
            /* 已输出过部分脉冲时当前位置不可推断，绝不能提交 target。 */
            robot_axis->position_valid = 0u;
            if (robot_axis->state == ROBOT_AXIS_HOMING)
            {
                robot_axis->homed = 0u;
            }
        }
        robot_axis->active = 0u;
        robot_axis->state = ROBOT_AXIS_IDLE;
        robot_axis->moving_direction = 0;
        robot_axis->command_steps = 0u;
        robot_axis->move_start_ms = 0u;
        robot_axis->move_timeout_ms = 0u;
        robot_axis->end_reason = ROBOT_MOVE_END_STOPPED;
        s_robot_arm.move_axis_progress[index] =
            ROBOT_MOVE_AXIS_NOT_REQUIRED;
        s_move_debug.axis_progress[index] =
            ROBOT_MOVE_AXIS_NOT_REQUIRED;
    }
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_STOPPED;
    s_robot_arm.error_code = ROBOT_ARM_ERR_STOPPED;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.state = ROBOT_ARM_IDLE;
    RobotArm_ResetCombinedStates();
    s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_STOPPED;
}

/**
 * 在完整 HC165 快照更新后立即拦截正在压 S1/S2/S3 的负向运动。
 *
 * 传感器触发后允许正方向继续脱离；仅当当前逻辑方向为负时停止对应 DMA。普通
 * 运动会将当前坐标标记为未知；Home 快速寻零命中则由 Home 状态机确认零点。
 */
void RobotArm_OnSensorSnapshotUpdated(void)
{
    uint8_t index;
    RobotArm_UpdateDebugWatch();
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        if (RobotArmDriver_ShouldStopForNegativeLimit((RobotAxisId_t)index))
        {
            /* Home 快速寻零命中 S1/S2/S3 是成功条件，不得按普通越限置 ERROR；
             * 这里只做最快 DMA 停机，随后 Home 状态机在同一 Active 快照中置零。 */
            if (((s_robot_arm.operation == ROBOT_OP_HOME_X) ||
                 (s_robot_arm.operation == ROBOT_OP_HOME_Y) ||
                 (s_robot_arm.operation == ROBOT_OP_HOME_Z) ||
                 (s_robot_arm.operation == ROBOT_OP_HOME_ALL)) &&
                (s_robot_arm.home_axis == (RobotAxisId_t)index) &&
                (s_robot_arm.home_state == ROBOT_HOME_SEEK_FAST))
            {
                RobotArmDriver_Stop((RobotAxisId_t)index);
                s_robot_arm.axis[index].active = 0u;
                if (index == ROBOT_AXIS_X) dbg_x_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                if (index == ROBOT_AXIS_Y) dbg_y_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                if (index == ROBOT_AXIS_Z) dbg_z_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                RobotArm_UpdateDebugWatch();
                return;
            }
            if (index == ROBOT_AXIS_X) { dbg_x_limit_stop_count++; dbg_x_last_stop_reason = ROBOT_MOVE_END_LIMIT; }
            if (index == ROBOT_AXIS_Y) { dbg_y_limit_stop_count++; dbg_y_last_stop_reason = ROBOT_MOVE_END_LIMIT; }
            if (index == ROBOT_AXIS_Z) { dbg_z_limit_stop_count++; dbg_z_last_stop_reason = ROBOT_MOVE_END_LIMIT; }
            RobotArm_FailAxisMove((RobotAxisId_t)index, ROBOT_MOVE_END_LIMIT);
            RobotArm_UpdateDebugWatch();
            return;
        }
    }
}

/** 清除管理层 ERROR；不会恢复坐标有效性或回零状态。 */
RobotArmResult_t RobotArm_ClearError(void)
{
    uint8_t index;
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        if (s_robot_arm.axis[index].state == ROBOT_AXIS_ERROR)
        {
            s_robot_arm.axis[index].state = ROBOT_AXIS_IDLE;
        }
    }
    s_robot_arm.state = ROBOT_ARM_IDLE;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.error_code = 0;
    RobotArm_ResetCombinedStates();
    return ROBOT_ARM_OK;
}

/** 使旧入口已直接移动的轴坐标失效。 */
void RobotArm_InvalidatePosition(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis != 0)
    {
        if (robot_axis->active)
        {
            /* 旧直驱插入正式动作时立即终止管理层任务，防止继续后续轴。 */
            RobotArm_FailAxisMove(axis, ROBOT_MOVE_END_STOPPED);
            return;
        }
        /* 旧入口只破坏当前位置；曾经完成 Home 的历史不应被伪造或抹除。 */
        robot_axis->position_valid = 0u;
    }
}

/** 查询机械臂是否存在正在执行的独占动作。 */
uint8_t RobotArm_IsBusy(void)
{
    uint8_t index;
    if (s_robot_arm.operation != ROBOT_OP_NONE)
    {
        return 1u;
    }
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        if (RobotArmDriver_IsBusy((RobotAxisId_t)index))
        {
            return 1u;
        }
    }
    return 0u;
}

/** 查询机械臂整体状态。 */
RobotArmState_t RobotArm_GetState(void) { return s_robot_arm.state; }

/** 查询指定轴状态。 */
RobotAxisState_t RobotArm_GetAxisState(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    return (robot_axis == 0) ? ROBOT_AXIS_ERROR : robot_axis->state;
}

/** 查询 X 轴当前逻辑坐标。 */
int32_t RobotArm_GetX(void) { return s_robot_arm.axis[ROBOT_AXIS_X].current_position; }
/** 查询 Y 轴当前逻辑坐标。 */
int32_t RobotArm_GetY(void) { return s_robot_arm.axis[ROBOT_AXIS_Y].current_position; }
/** 查询 Z 轴当前逻辑坐标。 */
int32_t RobotArm_GetZ(void) { return s_robot_arm.axis[ROBOT_AXIS_Z].current_position; }

/** 查询指定轴坐标是否可信。 */
uint8_t RobotArm_IsPositionValid(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    return (robot_axis == 0) ? 0u : robot_axis->position_valid;
}

/** 查询指定轴是否在本次上电后成功建立过原点。 */
uint8_t RobotArm_IsHomed(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    return (robot_axis == 0) ? 0u : robot_axis->homed;
}

/** 获取机械臂坐标、状态、传感器和结束原因快照。 */
void RobotArm_GetStatus(RobotArmStatus_t *status)
{
    if (status == 0)
    {
        return;
    }
    status->x = RobotArm_GetX();
    status->y = RobotArm_GetY();
    status->z = RobotArm_GetZ();
    if (s_robot_arm.operation == ROBOT_OP_MOVE_TO_SAFE)
    {
        status->target_x = s_robot_arm.safe_move_target[ROBOT_AXIS_X];
        status->target_y = s_robot_arm.safe_move_target[ROBOT_AXIS_Y];
        status->target_z = s_robot_arm.safe_move_target[ROBOT_AXIS_Z];
    }
    else
    {
        status->target_x = s_robot_arm.axis[ROBOT_AXIS_X].target_position;
        status->target_y = s_robot_arm.axis[ROBOT_AXIS_Y].target_position;
        status->target_z = s_robot_arm.axis[ROBOT_AXIS_Z].target_position;
    }
    status->x_homed = RobotArm_IsHomed(ROBOT_AXIS_X);
    status->y_homed = RobotArm_IsHomed(ROBOT_AXIS_Y);
    status->z_homed = RobotArm_IsHomed(ROBOT_AXIS_Z);
    status->x_valid = RobotArm_IsPositionValid(ROBOT_AXIS_X);
    status->y_valid = RobotArm_IsPositionValid(ROBOT_AXIS_Y);
    status->z_valid = RobotArm_IsPositionValid(ROBOT_AXIS_Z);
    status->s1_x_home = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_X_HOME);
    status->s2_y_home = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Y_HOME);
    status->s3_z_home = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Z_HOME);
    status->x_state = RobotArm_GetAxisState(ROBOT_AXIS_X);
    status->y_state = RobotArm_GetAxisState(ROBOT_AXIS_Y);
    status->z_state = RobotArm_GetAxisState(ROBOT_AXIS_Z);
    status->arm_state = s_robot_arm.state;
    status->operation = s_robot_arm.operation;
    status->home_state = s_robot_arm.home_state;
    status->move_to_state = s_robot_arm.move_to_state;
    status->safe_move_state = s_robot_arm.safe_move_state;
    status->last_move_end_reason = s_robot_arm.last_move_end_reason;
    status->error_code = s_robot_arm.error_code;
}

/** 获取最近一次 MOVE_TO 的接收、启动和收尾诊断快照；不参与 V2 STATUS 打包。 */
void RobotArm_GetMoveDebug(RobotArmMoveDebug_t *debug)
{
    uint8_t index;
    if (debug == 0)
    {
        return;
    }
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        debug->received_current[index] =
            s_move_debug.received_current[index];
        debug->received_target[index] =
            s_move_debug.received_target[index];
        debug->received_delta[index] =
            s_move_debug.received_delta[index];
        debug->axis_progress[index] = s_move_debug.axis_progress[index];
        debug->start_called[index] = s_move_debug.start_called[index];
        debug->start_steps[index] = s_move_debug.start_steps[index];
        debug->start_result[index] = s_move_debug.start_result[index];
        debug->busy_before[index] = s_move_debug.busy_before[index];
        debug->busy_after[index] = s_move_debug.busy_after[index];
    }
    debug->finalize_reason = s_move_debug.finalize_reason;
}

/** 检查目标姿态安全性；当前无标定碰撞区时默认通过。 */
RobotArmResult_t RobotArm_CheckPoseSafety(int32_t x, int32_t y, int32_t z)
{
    (void)x;
    (void)y;
    (void)z;
#ifdef ROBOT_ARM_LOGIC_TEST
    extern uint8_t g_robot_arm_logic_test_pose_safety_blocked;
    if (g_robot_arm_logic_test_pose_safety_blocked)
    {
        return ROBOT_ARM_ERR_INTERLOCK;
    }
#endif
    return ROBOT_ARM_OK;
}

/** 检查两个姿态之间是否允许直接转换。 */
RobotArmResult_t RobotArm_CheckTransitionSafety(int32_t current_x,
                                                int32_t current_y,
                                                int32_t current_z,
                                                int32_t target_x,
                                                int32_t target_y,
                                                int32_t target_z)
{
    (void)target_z;
    if ((current_x != target_x) || (current_y != target_y))
    {
        if (!ROBOT_ARM_SAFE_MOVE_ENABLED)
        {
            return ROBOT_ARM_ERR_CONFIG;
        }
        /* Z=0 为最高点，因此 XY 改变时 Z 数值必须小于等于 Safe Z。 */
        if (current_z > ROBOT_ARM_SAFE_Z_POSITION)
        {
            return ROBOT_ARM_ERR_INTERLOCK;
        }
    }
    return ROBOT_ARM_OK;
}
