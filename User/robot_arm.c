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
    RobotMoveAxisProgress_t move_axis_progress[ROBOT_AXIS_COUNT];
    int32_t safe_move_target[ROBOT_AXIS_COUNT];
    RobotMoveEndReason_t last_move_end_reason;
    int32_t error_code;
} RobotArm_t;

static RobotArm_t s_robot_arm;
static RobotArmMoveDebug_t s_move_debug;

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

static uint8_t RobotArm_IsAxisLimitEnabled(RobotAxisId_t axis)
{
    static const uint8_t enabled[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_LIMIT_ENABLED,
        ROBOT_ARM_Y_LIMIT_ENABLED,
        ROBOT_ARM_Z_LIMIT_ENABLED};
    return enabled[axis];
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
    if (RobotArm_IsAxisLimitEnabled(axis) &&
        ((target < robot_axis->min_position) ||
         (target > robot_axis->max_position)))
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
    if ((axis == ROBOT_AXIS_Z) && (direction > 0) &&
        RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Z_LOWER_LIMIT))
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
    RobotArmResult_t result;
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
    if (speed == 0u)
    {
        speed = robot_axis->default_speed;
    }
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

static RobotArmResult_t RobotArm_ValidateHomeConfig(RobotAxisId_t axis)
{
    if (!RobotArm_IsHomeEnabled(axis) ||
        (RobotArm_GetHomeMaxSteps(axis) == 0u) ||
        (RobotArm_GetHomeTimeout(axis) == 0u) ||
        (ROBOT_ARM_HOME_FAST_SPEED == 0u) ||
        (ROBOT_ARM_HOME_SLOW_SPEED == 0u) ||
        (ROBOT_ARM_HOME_BACKOFF_STEPS == 0u))
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

static RobotArmResult_t RobotArm_BeginHomeAxis(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    RobotArmResult_t result = RobotArm_ValidateHomeConfig(axis);
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
    RobotArmResult_t result;

    if ((uint32_t)(millis() - s_robot_arm.home_start_ms) >=
        RobotArm_GetHomeTimeout(s_robot_arm.home_axis))
    {
        RobotArm_FailHome(ROBOT_ARM_ERR_HOME_TIMEOUT, ROBOT_MOVE_END_TIMEOUT);
        return;
    }
    /* Z 退出上限期间仍持续保护下限，避免错误参数驱动至 S4。 */
    if ((s_robot_arm.home_axis == ROBOT_AXIS_Z) && robot_axis->active &&
        (robot_axis->moving_direction > 0) &&
        RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Z_LOWER_LIMIT))
    {
        RobotArm_FailHome(ROBOT_ARM_ERR_LIMIT, ROBOT_MOVE_END_LIMIT);
        return;
    }

    switch (s_robot_arm.home_state)
    {
    case ROBOT_HOME_CHECK_SENSOR:
        if (RobotArmSensor_IsTriggered(sensor))
        {
            result = RobotArm_StartHomeDriver(s_robot_arm.home_axis,
                                              (int8_t)-home_direction,
                                              ROBOT_ARM_HOME_BACKOFF_STEPS,
                                              ROBOT_ARM_HOME_FAST_SPEED);
            s_robot_arm.home_state = ROBOT_HOME_BACKOFF_IF_ACTIVE;
        }
        else
        {
            result = RobotArm_StartHomeDriver(s_robot_arm.home_axis,
                                              home_direction,
                                              RobotArm_GetHomeMaxSteps(s_robot_arm.home_axis),
                                              ROBOT_ARM_HOME_FAST_SPEED);
            s_robot_arm.home_state = ROBOT_HOME_SEEK_FAST;
        }
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
        }
        break;

    case ROBOT_HOME_BACKOFF_IF_ACTIVE:
        if (!RobotArmSensor_IsTriggered(sensor))
        {
            RobotArmDriver_Stop(s_robot_arm.home_axis);
            robot_axis->active = 0u;
            result = RobotArm_StartHomeDriver(s_robot_arm.home_axis,
                                              home_direction,
                                              RobotArm_GetHomeMaxSteps(s_robot_arm.home_axis),
                                              ROBOT_ARM_HOME_FAST_SPEED);
            s_robot_arm.home_state = ROBOT_HOME_SEEK_FAST;
            if (result != ROBOT_ARM_OK)
            {
                RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
            }
        }
        else if (!RobotArmDriver_IsBusy(s_robot_arm.home_axis))
        {
            RobotArm_FailHome(ROBOT_ARM_ERR_SENSOR, ROBOT_MOVE_END_SENSOR);
        }
        break;

    case ROBOT_HOME_SEEK_FAST:
        if (RobotArmSensor_IsTriggered(sensor))
        {
            RobotArmDriver_Stop(s_robot_arm.home_axis);
            robot_axis->active = 0u;
            result = RobotArm_StartHomeDriver(s_robot_arm.home_axis,
                                              (int8_t)-home_direction,
                                              ROBOT_ARM_HOME_BACKOFF_STEPS,
                                              ROBOT_ARM_HOME_FAST_SPEED);
            s_robot_arm.home_state = ROBOT_HOME_BACKOFF_AFTER_TRIGGER;
            if (result != ROBOT_ARM_OK)
            {
                RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
            }
        }
        else if (!RobotArmDriver_IsBusy(s_robot_arm.home_axis))
        {
            RobotArm_FailHome(ROBOT_ARM_ERR_HOME_TIMEOUT, ROBOT_MOVE_END_TIMEOUT);
        }
        break;

    case ROBOT_HOME_BACKOFF_AFTER_TRIGGER:
        if (!RobotArmDriver_IsBusy(s_robot_arm.home_axis))
        {
            robot_axis->active = 0u;
            if (RobotArmDriver_GetRemainingSteps(s_robot_arm.home_axis) != 0u)
            {
                RobotArm_FailHome(ROBOT_ARM_ERR_DRIVER,
                                  ROBOT_MOVE_END_DRIVER_ERROR);
                return;
            }
            if (RobotArmSensor_IsTriggered(sensor))
            {
                RobotArm_FailHome(ROBOT_ARM_ERR_SENSOR, ROBOT_MOVE_END_SENSOR);
                return;
            }
            result = RobotArm_StartHomeDriver(s_robot_arm.home_axis,
                                              home_direction,
                                              RobotArm_GetHomeMaxSteps(s_robot_arm.home_axis),
                                              ROBOT_ARM_HOME_SLOW_SPEED);
            s_robot_arm.home_state = ROBOT_HOME_SEEK_SLOW;
            if (result != ROBOT_ARM_OK)
            {
                RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
            }
        }
        break;

    case ROBOT_HOME_SEEK_SLOW:
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
            ROBOT_AXIS_X, s_robot_arm.move_to_target[ROBOT_AXIS_X], 0u);
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
            ROBOT_AXIS_Y, s_robot_arm.move_to_target[ROBOT_AXIS_Y], 0u);
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
            ROBOT_AXIS_Z, s_robot_arm.move_to_target[ROBOT_AXIS_Z], 0u);
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
            ROBOT_AXIS_Z, ROBOT_ARM_SAFE_Z_POSITION, 0u);
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
                ROBOT_AXIS_X, s_robot_arm.safe_move_target[ROBOT_AXIS_X], 0u);
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
                ROBOT_AXIS_Y, s_robot_arm.safe_move_target[ROBOT_AXIS_Y], 0u);
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
                ROBOT_AXIS_Z, s_robot_arm.safe_move_target[ROBOT_AXIS_Z], 0u);
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

/** 按 X、Y、Z 顺序启动非阻塞目标位置任务。 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z)
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

/** 按“必要时抬高 Z、移动 X/Y、最后移动 Z”的安全路径接受绝对位置任务。 */
RobotArmResult_t RobotArm_MoveToSafe(int32_t x, int32_t y, int32_t z)
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

/** 启动指定轴的非阻塞双阶段 Home。 */
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
    result = RobotArm_BeginHomeAxis(axis);
    if (result == ROBOT_ARM_OK)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_HOME_X + axis);
        s_robot_arm.error_code = 0;
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
    result = RobotArm_ValidateHomeConfig(ROBOT_AXIS_Z);
    if (result == ROBOT_ARM_OK) result = RobotArm_ValidateHomeConfig(ROBOT_AXIS_Y);
    if (result == ROBOT_ARM_OK) result = RobotArm_ValidateHomeConfig(ROBOT_AXIS_X);
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    s_robot_arm.home_all_active = 1u;
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
    status->s4_z_lower_limit = RobotArmSensor_IsTriggered(ROBOT_ARM_SENSOR_Z_LOWER_LIMIT);
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
