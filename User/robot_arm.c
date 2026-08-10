#include "robot_arm.h"
#include "robot_arm_config.h"

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
    uint8_t active;
} RobotAxis_t;

typedef struct
{
    RobotAxis_t axis[ROBOT_AXIS_COUNT];
    RobotArmState_t state;
    RobotArmOperation_t operation;
    int32_t error_code;
} RobotArm_t;

static RobotArm_t s_robot_arm;

static RobotAxis_t *RobotArm_GetAxis(RobotAxisId_t axis)
{
    if (axis >= ROBOT_AXIS_COUNT)
    {
        return 0;
    }
    return &s_robot_arm.axis[axis];
}

static RobotArmOperation_t RobotArm_GetMoveOperation(RobotAxisId_t axis)
{
    return (RobotArmOperation_t)(ROBOT_OP_MOVE_X + axis);
}

static RobotArmResult_t RobotArm_StartRelative(RobotAxisId_t axis,
                                                int32_t delta,
                                                uint32_t speed)
{
    RobotAxis_t *robot_axis;
    uint32_t steps;
    int8_t direction;
    int64_t target;

    robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if (delta == 0)
    {
        return ROBOT_ARM_OK;
    }

    target = (int64_t)robot_axis->current_position + (int64_t)delta;
    if ((target < (int64_t)robot_axis->min_position) ||
        (target > (int64_t)robot_axis->max_position))
    {
        return ROBOT_ARM_ERR_LIMIT;
    }

    direction = (delta > 0) ? 1 : -1;
    steps = (delta > 0) ? (uint32_t)delta : (uint32_t)(-(int64_t)delta);
    if (!RobotArmDriver_Start(axis, direction, steps,
                              (speed == 0u) ? robot_axis->default_speed : speed))
    {
        return ROBOT_ARM_ERR_DRIVER;
    }

    /* 仅记录目标；真实 current_position 只能在底层 DMA 完成后提交。 */
    robot_axis->target_position = (int32_t)target;
    robot_axis->moving_direction = direction;
    robot_axis->state = ROBOT_AXIS_MOVING;
    robot_axis->active = 1u;
    s_robot_arm.state = ROBOT_ARM_MOVING;
    s_robot_arm.operation = RobotArm_GetMoveOperation(axis);
    return ROBOT_ARM_OK;
}

static RobotArmResult_t RobotArm_MoveAbsolute(RobotAxisId_t axis,
                                               int32_t target,
                                               uint32_t speed)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    int64_t delta;

    if (robot_axis == 0)
    {
        return ROBOT_ARM_ERR_DRIVER;
    }
    if (!robot_axis->position_valid)
    {
        return ROBOT_ARM_ERR_POSITION_UNKNOWN;
    }
    delta = (int64_t)target - (int64_t)robot_axis->current_position;
    if ((delta < INT32_MIN) || (delta > INT32_MAX))
    {
        return ROBOT_ARM_ERR_LIMIT;
    }
    return RobotArm_StartRelative(axis, (int32_t)delta, speed);
}

/** 初始化 XYZ 机械臂管理层；不会启动任意电机。 */
void RobotArm_Init(void)
{
    uint8_t index;
    static const int32_t min_position[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_MIN_POSITION, ROBOT_ARM_Y_MIN_POSITION, ROBOT_ARM_Z_MIN_POSITION};
    static const int32_t max_position[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_MAX_POSITION, ROBOT_ARM_Y_MAX_POSITION, ROBOT_ARM_Z_MAX_POSITION};
    static const uint32_t default_speed[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_DEFAULT_SPEED, ROBOT_ARM_Y_DEFAULT_SPEED, ROBOT_ARM_Z_DEFAULT_SPEED};

    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        s_robot_arm.axis[index].state = ROBOT_AXIS_IDLE;
        s_robot_arm.axis[index].current_position = 0;
        s_robot_arm.axis[index].target_position = 0;
        s_robot_arm.axis[index].min_position = min_position[index];
        s_robot_arm.axis[index].max_position = max_position[index];
        s_robot_arm.axis[index].default_speed = default_speed[index];
        /* 上电数值为零不代表机械位于零点，绝对坐标必须保持无效。 */
        s_robot_arm.axis[index].homed = 0u;
        s_robot_arm.axis[index].position_valid = 0u;
        s_robot_arm.axis[index].active = 0u;
    }
    s_robot_arm.state = ROBOT_ARM_IDLE;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.error_code = 0;
}

/** 在主循环中推进已启动动作的完成状态，不包含阻塞等待。 */
void RobotArm_Task(void)
{
    uint8_t index;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        if (robot_axis->active && !RobotArmDriver_IsBusy((RobotAxisId_t)index))
        {
            /* 驱动真实结束后再提交逻辑坐标，避免 DMA 启动即伪造到位。 */
            robot_axis->current_position = robot_axis->target_position;
            robot_axis->state = ROBOT_AXIS_IDLE;
            robot_axis->active = 0u;
            s_robot_arm.state = ROBOT_ARM_IDLE;
            s_robot_arm.operation = ROBOT_OP_NONE;
        }
    }
}

/** 启动 X 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveXRelative(int32_t delta, uint32_t speed)
{
    return RobotArm_StartRelative(ROBOT_AXIS_X, delta, speed);
}

/** 启动 Y 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveYRelative(int32_t delta, uint32_t speed)
{
    return RobotArm_StartRelative(ROBOT_AXIS_Y, delta, speed);
}

/** 启动 Z 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveZRelative(int32_t delta, uint32_t speed)
{
    return RobotArm_StartRelative(ROBOT_AXIS_Z, delta, speed);
}

/** 启动 X 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveX(int32_t target, uint32_t speed)
{
    return RobotArm_MoveAbsolute(ROBOT_AXIS_X, target, speed);
}

/** 启动 Y 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveY(int32_t target, uint32_t speed)
{
    return RobotArm_MoveAbsolute(ROBOT_AXIS_Y, target, speed);
}

/** 启动 Z 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveZ(int32_t target, uint32_t speed)
{
    return RobotArm_MoveAbsolute(ROBOT_AXIS_Z, target, speed);
}

/** 停止全部三轴；中途停止的活动轴坐标将标记为不可信。 */
void RobotArm_Stop(void)
{
    uint8_t index;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        if (robot_axis->active || RobotArmDriver_IsBusy((RobotAxisId_t)index))
        {
            /* 现有 X/Z 驱动没有可靠已执行步数接口，停止后绝不猜测坐标。 */
            robot_axis->position_valid = 0u;
            robot_axis->homed = 0u;
        }
        RobotArmDriver_Stop((RobotAxisId_t)index);
        robot_axis->active = 0u;
        robot_axis->state = ROBOT_AXIS_IDLE;
    }
    s_robot_arm.state = ROBOT_ARM_IDLE;
    s_robot_arm.operation = ROBOT_OP_NONE;
}

/** 使旧入口已直接移动的轴坐标失效。 */
void RobotArm_InvalidatePosition(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis != 0)
    {
        robot_axis->position_valid = 0u;
        robot_axis->homed = 0u;
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
    /* 过渡期仍保留旧串口直驱，发现底层在运行时禁止管理层插入新动作。 */
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
RobotArmState_t RobotArm_GetState(void)
{
    return s_robot_arm.state;
}

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

/** 查询指定轴是否已完成可靠回零。 */
uint8_t RobotArm_IsHomed(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    return (robot_axis == 0) ? 0u : robot_axis->homed;
}

/** 获取 XYZ 坐标、有效性及运动状态快照。 */
void RobotArm_GetStatus(RobotArmStatus_t *status)
{
    if (status == 0)
    {
        return;
    }
    status->x = RobotArm_GetX(); status->y = RobotArm_GetY(); status->z = RobotArm_GetZ();
    status->target_x = s_robot_arm.axis[ROBOT_AXIS_X].target_position;
    status->target_y = s_robot_arm.axis[ROBOT_AXIS_Y].target_position;
    status->target_z = s_robot_arm.axis[ROBOT_AXIS_Z].target_position;
    status->x_homed = RobotArm_IsHomed(ROBOT_AXIS_X); status->y_homed = RobotArm_IsHomed(ROBOT_AXIS_Y); status->z_homed = RobotArm_IsHomed(ROBOT_AXIS_Z);
    status->x_valid = RobotArm_IsPositionValid(ROBOT_AXIS_X); status->y_valid = RobotArm_IsPositionValid(ROBOT_AXIS_Y); status->z_valid = RobotArm_IsPositionValid(ROBOT_AXIS_Z);
    status->x_state = RobotArm_GetAxisState(ROBOT_AXIS_X); status->y_state = RobotArm_GetAxisState(ROBOT_AXIS_Y); status->z_state = RobotArm_GetAxisState(ROBOT_AXIS_Z);
    status->arm_state = s_robot_arm.state;
    status->operation = s_robot_arm.operation;
    status->error_code = s_robot_arm.error_code;
}

/** 预留 XYZ 串行 MoveTo 接口，当前不启用多轴状态机。 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z)
{
    (void)x; (void)y; (void)z;
    return ROBOT_ARM_ERR_NOT_SUPPORTED;
}

/** 预留 Home 接口，未确认原点传感器前不启动任意轴。 */
RobotArmResult_t RobotArm_HomeAxis(RobotAxisId_t axis)
{
    (void)axis;
    return ROBOT_ARM_ERR_NOT_SUPPORTED;
}
