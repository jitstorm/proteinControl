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

/** 同步移动的命令级阶段，保证所有普通轴完成后只检查一次补充找零。 */
typedef enum
{
    /** 非同步命令或已收尾，不应用补充找零语义。 */
    ROBOT_POST_HOME_NONE = 0,
    /** 等待 XYZ 原计算步数全部完成，目标零轴暂不按 Home 提前结束。 */
    ROBOT_POST_HOME_NORMAL_MOVE,
    /** 等待原目标零轴命中 S1/S2/S3；全部成功才收尾，失败则停止其他轴。 */
    ROBOT_POST_HOME_SEARCH
} RobotPostHomePhase_t;

/* 并发位移完成后每轮负向找零的最大步数；未命中可续段，总时限仍由轴 Home 配置控制。 */
#define ROBOT_ARM_POST_HOME_CHUNK_STEPS 5000u
/** 0x40 每轴在理论零点后最多补充搜索三段，避免传感器故障时持续朝负向运动。 */
#define ROBOT_ARM_DELAYED_HOME_MAX_CHUNKS 3u
/* 仅由明确参与当前同步命令的轴触发“目标为零后继续找零”语义。 */
#define ROBOT_ARM_AXIS_MASK(axis) ((uint8_t)(1u << (uint8_t)(axis)))
#define ROBOT_ARM_AXIS_MASK_ALL ((uint8_t)((1u << ROBOT_AXIS_COUNT) - 1u))

/** 机械臂命令运行态；原始目标保留至移动及补充找零全部结束。 */
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
    /** CMD=30 尚未完成寻零的轴；每个 bit 对应 X/Y/Z，三轴并发时独立清除。 */
    uint8_t home_all_pending_mask;
    /** CMD=30 三轴各自的传感器检查或快速寻零阶段，不能再复用单一 home_state。 */
    RobotHomeState_t home_all_axis_state[ROBOT_AXIS_COUNT];
    uint32_t home_start_ms;
    /** 当前同步命令的普通移动/补充找零阶段，STOP 时清除。 */
    RobotPostHomePhase_t post_home_phase;
    /** 本次同步命令受理时刻，单位毫秒；零目标轴总超时覆盖两个阶段。 */
    uint32_t post_home_start_ms;
    /** 当前同步命令中允许零目标补充找零的轴掩码；未参与 Phase 的轴必须为 0。 */
    uint8_t post_home_axis_mask;
    /** 原请求每轴速度，单位 steps/s；补充找零不使用同步降速后的速度。 */
    uint16_t post_home_speed[ROBOT_AXIS_COUNT];
    int32_t move_to_target[ROBOT_AXIS_COUNT];
    uint16_t move_to_speed[ROBOT_AXIS_COUNT];
    /** 0x40 各轴相对同一受理时刻的启动延时，单位为 100ms。 */
    uint8_t delayed_start_100ms[ROBOT_AXIS_COUNT];
    /** 0x40 已受理时刻；延时到期只由主循环比较，绝不阻塞等待。 */
    uint32_t delayed_start_base_ms;
    /** 0x40 正在执行理论零点后的补充找零轴；每个 bit 对应 X/Y/Z。 */
    uint8_t delayed_home_search_mask;
    /** 0x40 每轴已经启动的 5000 脉冲补充搜索段数。 */
    uint8_t delayed_home_search_count[ROBOT_AXIS_COUNT];
    /** 当前 MOVE_TO 是否由 Phase 驱动 XY 终止速度曲线；普通 MOVE_TO 始终为 0。 */
    uint8_t phase_xy_profile;
    /** X/Y 各自的 Phase 起始速度和终止速度，单位 steps/s。 */
    uint32_t phase_start_frequency[ROBOT_AXIS_COUNT];
    uint32_t phase_terminal_frequency[ROBOT_AXIS_COUNT];
    /** X/Y 从起始速度变化到终止速度的 Phase 加速时间，单位毫秒。 */
    uint32_t phase_acceleration_time_ms[ROBOT_AXIS_COUNT];
    /* 0 表示 HomeAll 使用 MCU 已标定快速速度；非零仅由 0x31 单轴 Home 设置。 */
    uint16_t home_fast_speed;
    /* 0 表示单轴 Home 使用对应轴默认加速度；非零来自 0x31 的本次请求。 */
    uint16_t home_acceleration;
    /** 单轴 Home 开始时锁存的可信到理论零点距离；0 表示只能低速未知搜索。 */
    uint32_t home_known_distance;
    RobotMoveAxisProgress_t move_axis_progress[ROBOT_AXIS_COUNT];
    int32_t safe_move_target[ROBOT_AXIS_COUNT];
    uint16_t safe_move_speed[ROBOT_AXIS_COUNT];
    RobotMoveEndReason_t last_move_end_reason;
    int32_t error_code;
} RobotArm_t;

static RobotArm_t s_robot_arm;
static RobotArmMoveDebug_t s_move_debug;

static uint8_t RobotArm_StartDelayedHomeSearch(RobotAxisId_t axis);
static int8_t RobotArm_TaskDelayedHomeSearch(RobotAxisId_t axis);

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
/* RAM Watch：XYZ_SYNC 只镜像上层完成收集状态，不能参与运动控制或协议输出。 */
volatile RobotMoveToState_t dbg_xyz_state;
volatile uint8_t dbg_xyz_x_required;
volatile uint8_t dbg_xyz_y_required;
volatile uint8_t dbg_xyz_z_required;
volatile uint8_t dbg_xyz_x_done;
volatile uint8_t dbg_xyz_y_done;
volatile uint8_t dbg_xyz_z_done;

/**
 * 刷新 XYZ_SYNC 的完成收集快照，供 Keil Watch 确认较早完成的轴是否保持锁存。
 *
 * 距离为零的轴不会启动，在此按“无需等待即已完成”显示为 done=1，避免把正常的
 * XY 同步误判为缺少 Z 完成事件。调试变量只读取正式状态机，绝不作为控制输入。
 */
static void RobotArm_UpdateXyzSyncDebug(void)
{
    dbg_xyz_state = s_robot_arm.move_to_state;
    dbg_xyz_x_required = (s_robot_arm.move_axis_progress[ROBOT_AXIS_X] !=
                          ROBOT_MOVE_AXIS_NOT_REQUIRED) ? 1u : 0u;
    dbg_xyz_y_required = (s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] !=
                          ROBOT_MOVE_AXIS_NOT_REQUIRED) ? 1u : 0u;
    dbg_xyz_z_required = (s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] !=
                          ROBOT_MOVE_AXIS_NOT_REQUIRED) ? 1u : 0u;
    dbg_xyz_x_done = (s_robot_arm.move_axis_progress[ROBOT_AXIS_X] ==
                      ROBOT_MOVE_AXIS_NOT_REQUIRED ||
                      s_robot_arm.move_axis_progress[ROBOT_AXIS_X] ==
                      ROBOT_MOVE_AXIS_COMPLETED) ? 1u : 0u;
    dbg_xyz_y_done = (s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] ==
                      ROBOT_MOVE_AXIS_NOT_REQUIRED ||
                      s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] ==
                      ROBOT_MOVE_AXIS_COMPLETED) ? 1u : 0u;
    dbg_xyz_z_done = (s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] ==
                      ROBOT_MOVE_AXIS_NOT_REQUIRED ||
                      s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] ==
                      ROBOT_MOVE_AXIS_COMPLETED) ? 1u : 0u;
}

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
    if (s_robot_arm.operation == ROBOT_OP_HOME_ALL)
    {
        dbg_x_home_stage = s_robot_arm.home_all_axis_state[ROBOT_AXIS_X];
        dbg_y_home_stage = s_robot_arm.home_all_axis_state[ROBOT_AXIS_Y];
        dbg_z_home_stage = s_robot_arm.home_all_axis_state[ROBOT_AXIS_Z];
    }
    else
    {
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

/**
 * 返回单轴 Home 在未指定加速度时使用的轴默认值。
 *
 * @param axis 当前正在置零的 X、Y 或 Z 机械轴。
 * @return 对应轴默认加速度，单位 steps/s^2；无效轴返回 0。
 */
static uint32_t RobotArm_GetConfiguredHomeAcceleration(RobotAxisId_t axis)
{
    static const uint32_t acceleration[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_DEFAULT_ACCELERATION,
        ROBOT_ARM_Y_HOME_DEFAULT_ACCELERATION,
        ROBOT_ARM_Z_HOME_DEFAULT_ACCELERATION};
    return (axis < ROBOT_AXIS_COUNT) ? acceleration[axis] : 0u;
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

/** 清除组合动作及补充找零阶段，确保 STOP 或清错后不会续跑旧命令。 */
static void RobotArm_ResetCombinedStates(void)
{
    uint8_t index;
    s_robot_arm.post_home_phase = ROBOT_POST_HOME_NONE;
    s_robot_arm.post_home_axis_mask = 0u;
    s_robot_arm.home_state = ROBOT_HOME_IDLE;
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_IDLE;
    s_robot_arm.safe_move_state = ROBOT_SAFE_MOVE_IDLE;
    s_robot_arm.home_all_active = 0u;
    s_robot_arm.home_all_pending_mask = 0u;
    s_robot_arm.home_known_distance = 0u;
    s_robot_arm.delayed_home_search_mask = 0u;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_NOT_REQUIRED;
        s_robot_arm.home_all_axis_state[index] = ROBOT_HOME_IDLE;
        s_robot_arm.delayed_home_search_count[index] = 0u;
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

/**
 * 将已确认压到对应 Home 传感器的轴建立为物理零点。
 *
 * S1/S2/S3 分别是 X/Y/Z 的唯一零点传感器。传感器 Active 已经是实际位置为 0
 * 的硬件证据，因此不能保留被中断前的估算步数，也不能把坐标标记为未知。
 *
 * @param axis 已被对应 Home 传感器确认处于物理零点的实际机械轴。
 */
static void RobotArm_SetAxisPositionToHome(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return;
    }
    robot_axis->current_position = 0;
    robot_axis->homed = 1u;
    robot_axis->position_valid = 1u;
}

/**
 * 以已确认的 Home 零点结束当前负向轴动作。
 *
 * 该路径用于普通零目标运动及同步移动后的补充找零。必须先停实际 STEP/DMA/TIM，再提交零点，
 * 防止 DMA 在主循环处理前续装下一段脉冲。完成后由单轴、MOVE_TO 或 SafeMove
 * 状态机在下一轮把该轴作为正常完成继续收尾或启动后续轴。
 *
 * @param axis 已在负向运动中触发对应 Home 传感器的实际机械轴。
 */
static void RobotArm_CompleteAxisMoveAtHome(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return;
    }
    RobotArmDriver_Stop(axis);
    RobotArm_SetAxisPositionToHome(axis);
    robot_axis->target_position = 0;
    robot_axis->active = 0u;
    robot_axis->command_steps = 0u;
    robot_axis->state = ROBOT_AXIS_IDLE;
    robot_axis->end_reason = ROBOT_MOVE_END_COMPLETED;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
}

/**
 * 将负向 Home 命中但目标不是零点的动作以传感器错误结束。
 *
 * 传感器已经证明轴处于零点，故仍保留 current_position=0、homed=1 和
 * position_valid=1；错误仅表示本次非零目标不可能准确完成，不能误报为 LIMIT
 * 并丢失已确认的坐标基准。
 *
 * @param axis 已确认到达 Home 零点但本次目标不为 0 的实际机械轴。
 */
static void RobotArm_FailAxisMoveAtUnexpectedHome(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    uint8_t was_move_to = (s_robot_arm.operation == ROBOT_OP_MOVE_TO) ? 1u : 0u;
    uint8_t was_safe_move =
        (s_robot_arm.operation == ROBOT_OP_MOVE_TO_SAFE) ? 1u : 0u;
    if (robot_axis == 0)
    {
        return;
    }
    RobotArm_CompleteAxisMoveAtHome(axis);
    robot_axis->state = ROBOT_AXIS_ERROR;
    robot_axis->end_reason = ROBOT_MOVE_END_SENSOR;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_SENSOR;
    s_robot_arm.error_code = ROBOT_ARM_ERR_SENSOR;
    s_robot_arm.state = ROBOT_ARM_ERROR;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.move_to_state = was_move_to ?
                                    ROBOT_MOVE_TO_ERROR : ROBOT_MOVE_TO_IDLE;
    s_robot_arm.safe_move_state = was_safe_move ?
                                      ROBOT_SAFE_MOVE_ERROR : ROBOT_SAFE_MOVE_IDLE;
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

/**
 * 判断本轴是否应在同步普通移动结束后再处理 Home。
 * 仅原目标为零的 0x01 第一阶段适用；供上层与 DMA 续段共用，防止提前截断计算步数。
 * @param axis X/Y/Z 实际机械轴。
 * @return 当前命令需延后处理该轴 Home 时返回 1；其他动作和搜索阶段返回 0。
 */
uint8_t RobotArm_ShouldDeferHome(RobotAxisId_t axis)
{
    return axis < ROBOT_AXIS_COUNT &&
           s_robot_arm.operation == ROBOT_OP_MOVE_TO &&
           s_robot_arm.post_home_phase == ROBOT_POST_HOME_NORMAL_MOVE &&
           (s_robot_arm.post_home_axis_mask & ROBOT_ARM_AXIS_MASK(axis)) != 0u &&
           s_robot_arm.move_to_target[axis] == 0;
}

/**
 * 判断当前运动方向是否被 Home 阻挡；同步零目标第一阶段仅按完整步数结束。
 * @param axis 需要检查的实际机械轴。
 * @param direction 逻辑运动方向，负数表示朝 Home 方向。
 * @return 除限定的同步第一阶段外，负向且 Home 已触发时返回 1。
 */
static uint8_t RobotArm_IsDirectionBlocked(RobotAxisId_t axis,
                                           int8_t direction)
{
    if (!RobotArm_ShouldDeferHome(axis) && direction < 0 &&
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
    if (s_robot_arm.phase_xy_profile &&
        ((axis == ROBOT_AXIS_X) || (axis == ROBOT_AXIS_Y)))
    {
        driver_result = RobotArmDriver_StartPhase(
            axis, direction, steps, s_robot_arm.phase_start_frequency[axis],
            s_robot_arm.phase_terminal_frequency[axis],
            s_robot_arm.phase_acceleration_time_ms[axis]);
    }
    else
    {
        driver_result = RobotArmDriver_Start(axis, direction, steps, speed);
    }
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

/**
 * 收集普通轴运动结果；同步零目标轴必须以底层完整步数结束，不提前按 Home 置零。
 * @param axis 当前收集完成状态的实际机械轴。
 * @return 完成返回 1，等待返回 0，限位、超时或驱动异常返回 -1 并终止命令。
 */
static int8_t RobotArm_ProcessAxisMove(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return 0;
    }

    /* Home 命中在传感器快照回调中已同步停机并提交零点；此处把它交给调用状态机
     * 当作正常完成，才能让 MOVE_TO/SafeMove 继续后续实际轴。 */
    if (!robot_axis->active)
    {
        return (robot_axis->end_reason == ROBOT_MOVE_END_COMPLETED) &&
               (robot_axis->current_position == robot_axis->target_position);
    }

    /* 快照回调遗漏前的兜底：目标为 0 时，Home Active 仍是准确到位而非 LIMIT。 */
    if (RobotArm_IsDirectionBlocked(axis, robot_axis->moving_direction))
    {
        if (robot_axis->target_position == 0)
        {
            RobotArm_CompleteAxisMoveAtHome(axis);
            return 1;
        }
        RobotArm_FailAxisMoveAtUnexpectedHome(axis);
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
 * @param acceleration 本次快速寻零加速度，单位为 steps/s^2。
 * @return 实际配置完整且速度、加速度均在协议可表示范围内时返回 ROBOT_ARM_OK，否则返回
 *         ROBOT_ARM_ERR_CONFIG。
 */
static RobotArmResult_t RobotArm_ValidateHomeConfig(RobotAxisId_t axis,
                                                     uint32_t fast_speed,
                                                     uint32_t acceleration)
{
    if (!RobotArm_IsHomeEnabled(axis) ||
        (RobotArm_GetHomeMaxSteps(axis) == 0u) ||
        (RobotArm_GetHomeTimeout(axis) == 0u) ||
        (fast_speed == 0u) || (fast_speed > UINT16_MAX) ||
        (acceleration == 0u) || (acceleration > UINT16_MAX))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    return ROBOT_ARM_OK;
}

/**
 * 启动一次朝物理 Home 方向的 DMA 寻零运动。
 *
 * 仅在传感器尚未触发后调用；传入的速度和加速度都必须已完成协议或默认值解析，
 * 这样 X/Y/Z 才能使用同一轮 Home 的真实运动参数。驱动未进入运行态时不能把
 * 状态机置为 SEEK_FAST，以免错误等待不会产生的传感器命中。
 *
 * @param axis 需要寻零的实际机械轴。
 * @param direction 指向该轴 Home 传感器的逻辑负方向。
 * @param steps 本轮允许输出的最大 STEP 上升沿数量。
 * @param speed 快速寻零目标速度，单位 steps/s。
 * @param acceleration 快速寻零加速度，单位 steps/s^2。
 * @return 驱动真实启动时返回 ROBOT_ARM_OK，否则返回 ROBOT_ARM_ERR_DRIVER。
 */
static RobotArmResult_t RobotArm_StartHomeDriver(RobotAxisId_t axis,
                                                  int8_t direction,
                                                  uint32_t steps,
                                                  uint32_t speed,
                                                  uint32_t acceleration)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (!RobotArmDriver_StartWithAcceleration(
            axis, direction, steps, speed, acceleration))
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

/**
 * 返回当前单轴 Home 唯一快速寻零阶段实际使用的加速度。
 *
 * @param axis 当前正在置零的机械轴。
 * @return 本次请求加速度或对应轴默认加速度，单位 steps/s^2。
 */
static uint32_t RobotArm_GetActiveHomeAcceleration(RobotAxisId_t axis)
{
    return (s_robot_arm.home_acceleration != 0u) ?
               s_robot_arm.home_acceleration :
               RobotArm_GetConfiguredHomeAcceleration(axis);
}

/** 返回指定机械轴接近 S1/S2/S3 时的末段低速区长度，单位为 STEP。 */
static uint32_t RobotArm_GetHomeSlowZoneSteps(RobotAxisId_t axis)
{
    static const uint32_t slow_zone[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_SLOW_ZONE_STEPS, ROBOT_ARM_Y_HOME_SLOW_ZONE_STEPS,
        ROBOT_ARM_Z_HOME_SLOW_ZONE_STEPS};
    return (axis < ROBOT_AXIS_COUNT) ? slow_zone[axis] : 0u;
}

/** 返回指定机械轴接近 S1/S2/S3 时的最大速度，单位为 steps/s。 */
static uint32_t RobotArm_GetHomeSlowSpeed(RobotAxisId_t axis)
{
    static const uint32_t slow_speed[ROBOT_AXIS_COUNT] = {
        ROBOT_ARM_X_HOME_SLOW_SPEED, ROBOT_ARM_Y_HOME_SLOW_SPEED,
        ROBOT_ARM_Z_HOME_SLOW_SPEED};
    return (axis < ROBOT_AXIS_COUNT) ? slow_speed[axis] : 0u;
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
    s_robot_arm.home_known_distance = 0u;
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
        axis, RobotArm_GetActiveHomeFastSpeed(axis),
        RobotArm_GetActiveHomeAcceleration(axis));
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    /* 仅可信且非零的软件坐标可用于规划到理论零点的连续减速。 */
    s_robot_arm.home_known_distance =
        (robot_axis->position_valid && robot_axis->current_position > 0) ?
            (uint32_t)robot_axis->current_position : 0u;
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

    RobotArm_SetAxisPositionToHome(completed_axis);
    robot_axis->target_position = 0;
    robot_axis->active = 0u;
    robot_axis->state = ROBOT_AXIS_IDLE;
    robot_axis->end_reason = ROBOT_MOVE_END_COMPLETED;
    s_robot_arm.home_known_distance = 0u;
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
        /* S1/S2/S3 已触发时不允许输出任何 STEP，真实传感器优先于软件坐标。 */
        if (RobotArmSensor_IsTriggered(sensor))
        {
            RobotArm_CompleteHomeAxis();
        }
        else
        {
            if (s_robot_arm.home_known_distance != 0u)
            {
                /* 已知距离在一条 DMA 轮廓内完成高速、平滑减速和低速保持。 */
                if (!RobotArmDriver_StartHomeApproach(
                        s_robot_arm.home_axis, home_direction,
                        s_robot_arm.home_known_distance,
                        RobotArm_GetActiveHomeFastSpeed(s_robot_arm.home_axis),
                        RobotArm_GetHomeSlowSpeed(s_robot_arm.home_axis),
                        RobotArm_GetHomeSlowZoneSteps(s_robot_arm.home_axis),
                        RobotArm_GetActiveHomeAcceleration(s_robot_arm.home_axis)))
                {
                    result = ROBOT_ARM_ERR_DRIVER;
                }
                else
                {
                    robot_axis->active = 1u;
                    robot_axis->moving_direction = home_direction;
                    robot_axis->command_steps = s_robot_arm.home_known_distance;
                    robot_axis->end_reason = ROBOT_MOVE_END_NONE;
                }
                s_robot_arm.home_state = ROBOT_HOME_SEEK_KNOWN;
            }
            else
            {
                /* current=0 但未命中传感器没有可信距离，只允许低速有限搜索。 */
                result = RobotArm_StartHomeDriver(
                    s_robot_arm.home_axis, home_direction,
                    ROBOT_ARM_HOME_EXTRA_SEARCH_STEPS,
                    RobotArm_GetHomeSlowSpeed(s_robot_arm.home_axis),
                    RobotArm_GetActiveHomeAcceleration(s_robot_arm.home_axis));
                s_robot_arm.home_state = ROBOT_HOME_UNKNOWN_SEARCH;
            }
        }
        if (!RobotArmSensor_IsTriggered(sensor) && (result != ROBOT_ARM_OK))
        {
            RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
        }
        break;

    case ROBOT_HOME_SEEK_KNOWN:
        if (RobotArmSensor_IsTriggered(sensor))
        {
            RobotArmDriver_Stop(s_robot_arm.home_axis);
            robot_axis->active = 0u;
            RobotArm_CompleteHomeAxis();
        }
        else if (!RobotArmDriver_IsBusy(s_robot_arm.home_axis))
        {
            /* 理论零点不是完成条件；未命中 S 时继续低速搜索且有固定上限。 */
            result = RobotArm_StartHomeDriver(
                s_robot_arm.home_axis, home_direction,
                ROBOT_ARM_HOME_EXTRA_SEARCH_STEPS,
                RobotArm_GetHomeSlowSpeed(s_robot_arm.home_axis),
                RobotArm_GetActiveHomeAcceleration(s_robot_arm.home_axis));
            if (result == ROBOT_ARM_OK)
            {
                s_robot_arm.home_state = ROBOT_HOME_EXTRA_SEARCH;
            }
            else
            {
                RobotArm_FailHome(result, ROBOT_MOVE_END_DRIVER_ERROR);
            }
        }
        break;

    case ROBOT_HOME_EXTRA_SEARCH:
    case ROBOT_HOME_UNKNOWN_SEARCH:
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

/**
 * 将 CMD=30 中已经命中自身原点传感器的一轴标记为完成。
 *
 * 三轴并发 Home 时，S1/S2/S3 各自独立构成对应轴完成条件。先停止该轴实际 DMA，
 * 再写入可信零点；其他仍在寻找原点的轴不得因为一轴完成而停止。
 *
 * @param axis 已命中自身 Home 传感器的实际机械轴。
 */
static void RobotArm_CompleteHomeAllAxis(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if ((robot_axis == 0) ||
        ((s_robot_arm.home_all_pending_mask & ROBOT_ARM_AXIS_MASK(axis)) == 0u))
    {
        return;
    }
    RobotArmDriver_Stop(axis);
    RobotArm_SetAxisPositionToHome(axis);
    robot_axis->target_position = 0;
    robot_axis->active = 0u;
    robot_axis->command_steps = 0u;
    robot_axis->state = ROBOT_AXIS_IDLE;
    robot_axis->end_reason = ROBOT_MOVE_END_COMPLETED;
    s_robot_arm.home_all_axis_state[axis] = ROBOT_HOME_DONE;
    s_robot_arm.home_all_pending_mask &= (uint8_t)~ROBOT_ARM_AXIS_MASK(axis);
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;

    if (s_robot_arm.home_all_pending_mask == 0u)
    {
        s_robot_arm.home_all_active = 0u;
        s_robot_arm.home_state = ROBOT_HOME_DONE;
        s_robot_arm.operation = ROBOT_OP_NONE;
        s_robot_arm.state = ROBOT_ARM_IDLE;
    }
}

/**
 * 以安全停机方式终止 CMD=30 的并发 Home。
 *
 * 任一尚未完成轴超时或驱动启动失败时，其他仍在向原点运行的轴必须立刻停止，避免
 * 上位机已经收到失败而机械轴仍继续压向限位。已由传感器确认的轴保留可信零点；
 * 被中断的轴清除坐标有效性，要求后续重新回零。
 *
 * @param failed_axis 首个失败的实际机械轴。
 * @param error 要上报给 CMD=30 终态的失败结果。
 * @param reason 失败对应的机械结束原因。
 */
static void RobotArm_FailHomeAll(RobotAxisId_t failed_axis,
                                 RobotArmResult_t error,
                                 RobotMoveEndReason_t reason)
{
    uint8_t index;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        if ((s_robot_arm.home_all_pending_mask & ROBOT_ARM_AXIS_MASK(index)) == 0u)
        {
            continue;
        }
        RobotArmDriver_Stop((RobotAxisId_t)index);
        robot_axis->active = 0u;
        robot_axis->homed = 0u;
        robot_axis->position_valid = 0u;
        robot_axis->state = ROBOT_AXIS_ERROR;
        robot_axis->end_reason = ((RobotAxisId_t)index == failed_axis) ?
                                     reason : ROBOT_MOVE_END_STOPPED;
        s_robot_arm.home_all_axis_state[index] = ROBOT_HOME_ERROR;
    }
    s_robot_arm.home_all_pending_mask = 0u;
    s_robot_arm.home_all_active = 0u;
    s_robot_arm.last_move_end_reason = reason;
    s_robot_arm.error_code = (int32_t)error;
    s_robot_arm.home_state = ROBOT_HOME_ERROR;
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.state = ROBOT_ARM_ERROR;
}

/**
 * 并发推进 CMD=30 的 X/Y/Z 三轴快速寻零。
 *
 * 每轴都先读取自身 S1/S2/S3；初始已触发时直接置零，否则以该轴已标定快速速度
 * 启动 DMA。三轴共享同一次命令起始时刻，但分别应用各自 Home 超时，完成条件是
 * 三个 pending bit 均被对应传感器清除。
 */
static void RobotArm_TaskHomeAll(void)
{
    uint8_t index;
    RobotArmResult_t result;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxisId_t axis = (RobotAxisId_t)index;
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        RobotArmSensorId_t sensor;
        if ((s_robot_arm.home_all_pending_mask & ROBOT_ARM_AXIS_MASK(axis)) == 0u)
        {
            continue;
        }
        if ((uint32_t)(millis() - s_robot_arm.home_start_ms) >=
            RobotArm_GetHomeTimeout(axis))
        {
            RobotArm_FailHomeAll(axis, ROBOT_ARM_ERR_HOME_TIMEOUT,
                                 ROBOT_MOVE_END_TIMEOUT);
            return;
        }
        sensor = RobotArm_GetHomeSensor(axis);
        if (s_robot_arm.home_all_axis_state[axis] == ROBOT_HOME_CHECK_SENSOR)
        {
            if (RobotArmSensor_IsTriggered(sensor))
            {
                RobotArm_CompleteHomeAllAxis(axis);
                if (s_robot_arm.operation != ROBOT_OP_HOME_ALL) return;
                continue;
            }
            result = RobotArm_StartHomeDriver(
                axis, RobotArm_GetHomeDirection(axis), RobotArm_GetHomeMaxSteps(axis),
                RobotArm_GetConfiguredHomeFastSpeed(axis),
                RobotArm_GetConfiguredHomeAcceleration(axis));
            if (result != ROBOT_ARM_OK)
            {
                RobotArm_FailHomeAll(axis, result, ROBOT_MOVE_END_DRIVER_ERROR);
                return;
            }
            s_robot_arm.home_all_axis_state[axis] = ROBOT_HOME_SEEK_FAST;
        }
        else if (s_robot_arm.home_all_axis_state[axis] == ROBOT_HOME_SEEK_FAST)
        {
            if (RobotArmSensor_IsTriggered(sensor))
            {
                RobotArm_CompleteHomeAllAxis(axis);
                if (s_robot_arm.operation != ROBOT_OP_HOME_ALL) return;
            }
            else if (!RobotArmDriver_IsBusy(axis))
            {
                robot_axis->active = 0u;
                RobotArm_FailHomeAll(axis, ROBOT_ARM_ERR_HOME_TIMEOUT,
                                     ROBOT_MOVE_END_TIMEOUT);
                return;
            }
        }
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

/**
 * 为 XYZ_SYNC 预先核对每个需要运动的轴，避免启动第一轴后才发现其他轴不能启动。
 *
 * 复用现有目标和传感器检查；同步零目标第一阶段延后 Home 处理，实际启动仍由既有 DMA 入口负责。
 *
 * @return 全部需要运动的轴均可启动时返回 ROBOT_ARM_OK，否则返回首个错误。
 */
static RobotArmResult_t RobotArm_ValidateSyncMoveStart(void)
{
    uint8_t index;
    RobotArmResult_t result;

    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        if (s_robot_arm.move_axis_progress[index] ==
            ROBOT_MOVE_AXIS_NOT_REQUIRED)
        {
            continue;
        }
        result = RobotArm_CheckAxisTarget((RobotAxisId_t)index,
                                          s_robot_arm.move_to_target[index]);
        if (result != ROBOT_ARM_OK)
        {
            return result;
        }
        result = RobotArm_CheckTargetDirection((RobotAxisId_t)index,
                                                s_robot_arm.move_to_target[index]);
        if (result != ROBOT_ARM_OK)
        {
            return result;
        }
    }
    return ROBOT_ARM_OK;
}

/**
 * 按距离/最大速度比例计算 Phase 中 X/Y 两轴的实际速度。
 *
 * 该换算只由 0x39 Phase 调用，不能用于普通 0x34；0x34 的每根轴必须严格使用请求速度。
 * X/Y 使用限制轴作为时间基准，Z 保持请求的原始速度，因此允许先于或晚于 X/Y 到达。
 * 所有乘法通过 uint64_t 完成，避免 int24 距离与 uint16 速度相乘时发生 32 位溢出。
 * X/Y 距离为零时不参与计算也不会被启动。
 *
 * @return 速度可用时返回 ROBOT_ARM_OK；需要运动的轴没有有效速度时返回 CONFIG。
 */
static RobotArmResult_t RobotArm_CalculateSyncMoveSpeed(void)
{
    uint8_t index;
    uint8_t limit_axis = ROBOT_AXIS_COUNT;
    uint32_t distance[ROBOT_AXIS_COUNT];
    uint32_t maximum_speed[ROBOT_AXIS_COUNT];

    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        int64_t delta = (int64_t)s_robot_arm.move_to_target[index] -
                        (int64_t)s_robot_arm.axis[index].current_position;
        distance[index] = (uint32_t)((delta >= 0) ? delta : -delta);
        if (distance[index] == 0u)
        {
            s_robot_arm.move_to_speed[index] = 0u;
            continue;
        }
        maximum_speed[index] = RobotArm_ResolveMoveSpeed(
            (RobotAxisId_t)index, s_robot_arm.move_to_speed[index]);
        if (maximum_speed[index] == 0u)
        {
            return ROBOT_ARM_ERR_CONFIG;
        }
        if ((limit_axis == ROBOT_AXIS_COUNT) ||
            ((uint64_t)distance[index] * maximum_speed[limit_axis] >
             (uint64_t)distance[limit_axis] * maximum_speed[index]))
        {
            limit_axis = index;
        }
    }
    if (limit_axis == ROBOT_AXIS_COUNT)
    {
        return ROBOT_ARM_OK;
    }
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        uint64_t calculated;
        if (distance[index] == 0u)
        {
            continue;
        }
        if (index == limit_axis)
        {
            s_robot_arm.move_to_speed[index] = (uint16_t)maximum_speed[index];
            continue;
        }
        calculated = ((uint64_t)distance[index] * maximum_speed[limit_axis]) /
                     distance[limit_axis];
        if (calculated == 0u)
        {
            calculated = 1u;
        }
        if (calculated > maximum_speed[index])
        {
            calculated = maximum_speed[index];
        }
        s_robot_arm.move_to_speed[index] = (uint16_t)calculated;
    }
    return ROBOT_ARM_OK;
}

/**
 * 任一同步轴异常时立即停止其余仍在运行的实际轴。
 *
 * 不能让其他轴继续完成，否则停止后的坐标不再可信且会形成危险的非预期姿态。
 *
 * @param failed_axis 已经由既有单轴处理标记失败的轴。
 */
static void RobotArm_StopOtherSyncAxes(RobotAxisId_t failed_axis)
{
    uint8_t index;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        if ((index == failed_axis) || !robot_axis->active)
        {
            continue;
        }
        RobotArmDriver_Stop((RobotAxisId_t)index);
        robot_axis->active = 0u;
        robot_axis->position_valid = 0u;
        robot_axis->state = ROBOT_AXIS_ERROR;
        robot_axis->end_reason = s_robot_arm.last_move_end_reason;
        s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_NOT_REQUIRED;
        s_move_debug.axis_progress[index] = ROBOT_MOVE_AXIS_NOT_REQUIRED;
    }
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_ERROR;
}

/**
 * 结束补充找零失败并停止其余同步轴，避免单轴失控后其他轴继续运行。
 * @param axis 失败的 X/Y/Z 轴。
 * @param reason 超时或驱动异常原因。
 */
static void RobotArm_FailPostMoveHome(RobotAxisId_t axis, RobotMoveEndReason_t reason)
{
    RobotArm_FailAxisMove(axis, reason);
    s_robot_arm.axis[axis].homed = 0u;
    RobotArm_StopOtherSyncAxes(axis);
    if (reason == ROBOT_MOVE_END_TIMEOUT)
    {
        s_robot_arm.error_code = ROBOT_ARM_ERR_HOME_TIMEOUT;
    }
    s_robot_arm.post_home_phase = ROBOT_POST_HOME_NONE;
}

/**
 * 复用 Home 驱动启动最多 5000 步负向搜索，保留原坐标目标，不累计搜索路程。
 * @param axis 原目标为零且尚未触发 S1/S2/S3 的机械轴。
 * @return 实际进入 Busy 返回 1；启动失败会停止组合动作并返回 0。
 */
static uint8_t RobotArm_StartPostMoveHomeChunk(RobotAxisId_t axis)
{
    if (RobotArm_StartHomeDriver(axis, RobotArm_GetHomeDirection(axis),
            ROBOT_ARM_POST_HOME_CHUNK_STEPS,
            RobotArm_ResolveMoveSpeed(axis, s_robot_arm.post_home_speed[axis]),
            RobotArm_GetConfiguredHomeAcceleration(axis)) != ROBOT_ARM_OK ||
        !RobotArmDriver_IsBusy(axis))
    {
        RobotArm_FailPostMoveHome(axis, ROBOT_MOVE_END_DRIVER_ERROR);
        return 0u;
    }
    s_robot_arm.axis[axis].state = ROBOT_AXIS_HOMING;
    s_robot_arm.axis[axis].position_valid = 0u;
    s_robot_arm.move_axis_progress[axis] = ROBOT_MOVE_AXIS_RUNNING;
    return 1u;
}

/**
 * 普通 XYZ 全部结束后一次性检查原目标零轴，同时启动未命中 Home 的各轴搜索。
 * 已触发 S1/S2/S3 的轴直接建立零点，绝不追加补偿运动。
 * @return 存在搜索或启动失败时返回 1，阻止本轮成功收尾；无需搜索返回 0。
 */
static uint8_t RobotArm_StartPostMoveHomeIfNeeded(void)
{
    uint8_t index;
    uint8_t started = 0u;
    s_robot_arm.post_home_phase = ROBOT_POST_HOME_SEARCH;
    /* 普通位移结束并不代表整条请求完成；复用现有等待状态，避免 page0 在补零时报告 DONE。
     * 搜索由 post_home_phase 分支推进，只有全部轴结束后统一清为 IDLE；失败仍进入 ERROR。 */
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_XYZ_WAIT;
    for (index = 0u; index < ROBOT_AXIS_Z; index++)
    {
        if ((s_robot_arm.move_to_target[index] != 0) ||
            ((s_robot_arm.post_home_axis_mask &
              ROBOT_ARM_AXIS_MASK((RobotAxisId_t)index)) == 0u)) continue;
        if (RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor((RobotAxisId_t)index)))
        {
            RobotArm_CompleteAxisMoveAtHome((RobotAxisId_t)index);
        }
        else
        {
            if (!RobotArm_StartPostMoveHomeChunk((RobotAxisId_t)index)) return 1u;
            started = 1u;
        }
    }
    return started;
}

/**
 * 推进所有补充找零轴：传感器命中即停轴置零，完整走完一段但未命中则续段。
 * 各轴独立锁存完成；超时由命令级检查处理，驱动短走则整体失败。
 */
static void RobotArm_TaskPostMoveHome(void)
{
    uint8_t index;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxisId_t axis = (RobotAxisId_t)index;
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        if (s_robot_arm.move_to_target[index] != 0 ||
            ((s_robot_arm.post_home_axis_mask & ROBOT_ARM_AXIS_MASK(axis)) == 0u) ||
            s_robot_arm.move_axis_progress[index] != ROBOT_MOVE_AXIS_RUNNING) continue;
        /* 快照回调已经停轴时同样锁存完成，不要求传感器持续保持触发。 */
        if ((!robot_axis->active && robot_axis->end_reason == ROBOT_MOVE_END_COMPLETED) ||
            RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor(axis)))
        {
            RobotArm_CompleteAxisMoveAtHome(axis);
            s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_COMPLETED;
        }
        else if (!RobotArmDriver_IsBusy(axis))
        {
            if (RobotArmDriver_GetRemainingSteps(axis) != 0u ||
                RobotArmDriver_GetCompletedSteps(axis) < robot_axis->command_steps)
            {
                RobotArm_FailPostMoveHome(axis, ROBOT_MOVE_END_DRIVER_ERROR);
                return;
            }
            if (!RobotArm_StartPostMoveHomeChunk(axis)) return;
        }
    }
}

/** 所有轴真实完成且坐标一致后收尾；同步命令先完成一次补充找零检查。 */
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

    /* 只有普通三轴全部结束才进入第二阶段；SEARCH 标志防止重复启动。 */
    if (s_robot_arm.post_home_phase == ROBOT_POST_HOME_NORMAL_MOVE &&
        RobotArm_StartPostMoveHomeIfNeeded()) return;
    s_robot_arm.post_home_phase = ROBOT_POST_HOME_NONE;

    /* 最终轴确认完成的同一轮立即清理，避免 STATUS 读到短暂的伪 Busy。 */
    s_robot_arm.operation = ROBOT_OP_NONE;
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_IDLE;
    s_robot_arm.state = ROBOT_ARM_IDLE;
    s_robot_arm.error_code = 0;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
    s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_COMPLETED;
}

/** 推进 MOVE_TO；同步零目标轴在原请求内完成普通移动和搜索，失败或 STOP 后不续跑。 */
static void RobotArm_TaskMoveTo(void)
{
    RobotArmResult_t result;
    int8_t progress;
    uint8_t index;
    uint8_t all_completed;
    /* 零目标轴总时限从受理开始，不能在每轮 5000 步搜索时重置。 */
    if (s_robot_arm.post_home_phase != ROBOT_POST_HOME_NONE)
    {
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if (s_robot_arm.move_to_target[index] == 0 &&
                (s_robot_arm.post_home_phase == ROBOT_POST_HOME_NORMAL_MOVE ||
                 (s_robot_arm.move_axis_progress[index] == ROBOT_MOVE_AXIS_RUNNING &&
                  s_robot_arm.axis[index].active)) &&
                (uint32_t)(millis() - s_robot_arm.post_home_start_ms) >=
                    RobotArm_GetHomeTimeout((RobotAxisId_t)index))
            {
                RobotArm_FailPostMoveHome((RobotAxisId_t)index, ROBOT_MOVE_END_TIMEOUT);
                return;
            }
        }
    }
    if (s_robot_arm.post_home_phase == ROBOT_POST_HOME_SEARCH)
    {
        RobotArm_TaskPostMoveHome();
        if (s_robot_arm.operation == ROBOT_OP_MOVE_TO) RobotArm_TryCompleteMoveTo();
        RobotArm_UpdateXyzSyncDebug();
        return;
    }
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

    case ROBOT_MOVE_TO_XYZ_START:
        /* 先完成所有轴的共同安全检查，再连续启动，避免第二轴被拒绝时第一轴独自运动。 */
        result = RobotArm_ValidateSyncMoveStart();
        if (result != ROBOT_ARM_OK)
        {
            RobotArm_FailCombinedMoveStart(ROBOT_AXIS_X, result, 0u);
            break;
        }
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if (s_robot_arm.move_axis_progress[index] ==
                ROBOT_MOVE_AXIS_NOT_REQUIRED)
            {
                continue;
            }
            result = RobotArm_StartAxisMoveInternal(
                (RobotAxisId_t)index, s_robot_arm.move_to_target[index],
                s_robot_arm.move_to_speed[index]);
            if (result != ROBOT_ARM_OK)
            {
                /* 已连续启动的轴必须立刻停止，不能留下部分同步动作继续执行。 */
                RobotArm_StopOtherSyncAxes((RobotAxisId_t)index);
                RobotArm_FailCombinedMoveStart((RobotAxisId_t)index, result, 0u);
                break;
            }
            s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_RUNNING;
            s_move_debug.axis_progress[index] = ROBOT_MOVE_AXIS_RUNNING;
        }
        if (s_robot_arm.operation == ROBOT_OP_MOVE_TO)
        {
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_XYZ_WAIT;
        }
        RobotArm_UpdateXyzSyncDebug();
        break;

    case ROBOT_MOVE_TO_XYZ_WAIT:
        /* 每轮持续收集所有未完成轴；已锁存 COMPLETED 的轴绝不再次处理或清零。 */
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if ((s_robot_arm.move_axis_progress[index] ==
                 ROBOT_MOVE_AXIS_NOT_REQUIRED) ||
                (s_robot_arm.move_axis_progress[index] ==
                 ROBOT_MOVE_AXIS_COMPLETED))
            {
                continue;
            }
            progress = RobotArm_ProcessAxisMove((RobotAxisId_t)index);
            if (progress < 0)
            {
                RobotArm_StopOtherSyncAxes((RobotAxisId_t)index);
                break;
            }
            if (progress > 0)
            {
                s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_COMPLETED;
                s_move_debug.axis_progress[index] = ROBOT_MOVE_AXIS_COMPLETED;
            }
        }
        all_completed = 1u;
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if ((s_robot_arm.move_axis_progress[index] !=
                 ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
                (s_robot_arm.move_axis_progress[index] !=
                 ROBOT_MOVE_AXIS_COMPLETED))
            {
                all_completed = 0u;
                break;
            }
        }
        RobotArm_UpdateXyzSyncDebug();
        if ((s_robot_arm.operation == ROBOT_OP_MOVE_TO) && all_completed)
        {
            s_robot_arm.move_to_state = ROBOT_MOVE_TO_DONE;
            RobotArm_TryCompleteMoveTo();
            RobotArm_UpdateXyzSyncDebug();
        }
        break;

    case ROBOT_MOVE_TO_DELAYED_WAIT:
        /* 0x40 的每轴延时共用受理时刻；到期轴立即启动，不等待其他轴完成。 */
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if (s_robot_arm.move_axis_progress[index] != ROBOT_MOVE_AXIS_WAIT_START ||
                (uint32_t)(millis() - s_robot_arm.delayed_start_base_ms) <
                    (uint32_t)s_robot_arm.delayed_start_100ms[index] * 100u)
            {
                continue;
            }
            if ((s_robot_arm.move_to_target[index] == 0) &&
                (s_robot_arm.axis[index].current_position == 0))
            {
                /* 理论坐标已是零时仍必须检查真实 S1/S2/S3，未命中则直接进入有限搜索。 */
                if (!RobotArm_StartDelayedHomeSearch((RobotAxisId_t)index))
                {
                    return;
                }
                continue;
            }
            result = RobotArm_StartAxisMoveInternal(
                (RobotAxisId_t)index, s_robot_arm.move_to_target[index],
                s_robot_arm.move_to_speed[index]);
            if (result != ROBOT_ARM_OK)
            {
                /* 某轴到期却无法启动时，已运行轴必须停止，未到期轴绝不能继续启动。 */
                RobotArm_StopOtherSyncAxes((RobotAxisId_t)index);
                RobotArm_FailCombinedMoveStart((RobotAxisId_t)index, result, 0u);
                return;
            }
            s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_RUNNING;
            s_move_debug.axis_progress[index] = ROBOT_MOVE_AXIS_RUNNING;
        }
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if (s_robot_arm.move_axis_progress[index] != ROBOT_MOVE_AXIS_RUNNING)
            {
                continue;
            }
            if ((s_robot_arm.delayed_home_search_mask &
                 ROBOT_ARM_AXIS_MASK((RobotAxisId_t)index)) != 0u)
            {
                progress = RobotArm_TaskDelayedHomeSearch((RobotAxisId_t)index);
            }
            else
            {
                progress = RobotArm_ProcessAxisMove((RobotAxisId_t)index);
            }
            if (progress < 0)
            {
                RobotArm_StopOtherSyncAxes((RobotAxisId_t)index);
                return;
            }
            if (progress > 0)
            {
                if ((s_robot_arm.move_to_target[index] == 0) &&
                    !RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor((RobotAxisId_t)index)))
                {
                    /* 理论零点的 DMA 已完整结束，但物理零点尚未确认，立即补充负向找零。 */
                    if (!RobotArm_StartDelayedHomeSearch((RobotAxisId_t)index))
                    {
                        return;
                    }
                }
                else
                {
                    s_robot_arm.move_axis_progress[index] = ROBOT_MOVE_AXIS_COMPLETED;
                    s_move_debug.axis_progress[index] = ROBOT_MOVE_AXIS_COMPLETED;
                }
            }
        }
        all_completed = 1u;
        for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
        {
            if ((s_robot_arm.move_axis_progress[index] != ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
                (s_robot_arm.move_axis_progress[index] != ROBOT_MOVE_AXIS_COMPLETED))
            {
                all_completed = 0u;
                break;
            }
        }
        if (all_completed)
        {
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
    if (s_robot_arm.operation == ROBOT_OP_HOME_ALL)
    {
        RobotArm_TaskHomeAll();
        return;
    }
    if ((s_robot_arm.operation == ROBOT_OP_HOME_X) ||
        (s_robot_arm.operation == ROBOT_OP_HOME_Y) ||
        (s_robot_arm.operation == ROBOT_OP_HOME_Z))
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
 * 按指定运动方式启动一次非阻塞目标位置任务，并限定零目标补充找零的参与轴。
 *
 * 对外 MOVE_TO 的三轴零目标语义保持不变；Phase 仅让目标坐标实际变化的轴运动。
 * 进入补充找零，防止未参与的 Z 轴因当前坐标为 0 被误判为需要 Home。
 *
 * @param x X 轴目标绝对逻辑坐标，单位为步数。
 * @param y Y 轴目标绝对逻辑坐标，单位为步数。
 * @param z Z 轴目标绝对逻辑坐标，单位为步数。
 * @param x_speed X 轴最大速度，单位为 steps/s；0 表示使用既有默认速度。
 * @param y_speed Y 轴最大速度，单位为 steps/s；0 表示使用既有默认速度。
 * @param z_speed Z 轴最大速度，单位为 steps/s；0 表示使用既有默认速度。
 * @param motion_mode 顺序模式保持原行为；同步模式完整到位后对零目标轴按需找零，并允许零目标从未知坐标直接搜索，总超时或搜索失败会终止请求。
 * @param post_home_axis_mask 并发零目标需要补充找零的 X/Y/Z 轴掩码；顺序模式忽略。
 * @param apply_xy_speed_sync 保留的旧同步配速开关；当前 0x34 和 0x39 均传 0，保留请求逐轴速度。
 * @return 命令被接受时返回 ROBOT_ARM_OK；坐标、传感器、安全检查或驱动前置条件失败时返回对应错误。
 */
static RobotArmResult_t RobotArm_StartMoveToWithSpeedAndMode(
    int32_t x, int32_t y, int32_t z, uint16_t x_speed, uint16_t y_speed,
    uint16_t z_speed, RobotMoveMotionMode_t motion_mode,
    uint8_t post_home_axis_mask, uint8_t apply_xy_speed_sync)
{
    RobotArmResult_t result;
    uint8_t index;
    int32_t targets[ROBOT_AXIS_COUNT];
    int32_t current;
    int64_t delta;
    s_robot_arm.phase_xy_profile = 0u;
    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if ((motion_mode != ROBOT_MOVE_MOTION_SEQUENTIAL) &&
        (motion_mode != ROBOT_MOVE_MOTION_XYZ_SYNC))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    post_home_axis_mask &= ROBOT_ARM_AXIS_MASK_ALL;
    /* 0x01 的零目标轴允许从未知坐标开始搜索；非零目标仍必须有可信坐标，
     * 避免把绝对位置运动误当作 Home 而越过软件行程保护。 */
    if ((!s_robot_arm.axis[ROBOT_AXIS_X].position_valid &&
         !(motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC && x == 0)) ||
        (!s_robot_arm.axis[ROBOT_AXIS_Y].position_valid &&
         !(motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC && y == 0)) ||
        (!s_robot_arm.axis[ROBOT_AXIS_Z].position_valid &&
         !(motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC && z == 0)))
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
    /* 仅同步零目标允许先跑完计算步数；非零目标及顺序模式保留负向限位检查。 */
    if (result == ROBOT_ARM_OK &&
        !(motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC && x == 0 &&
          (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_X)) != 0u))
        result = RobotArm_CheckTargetDirection(ROBOT_AXIS_X, x);
    if (result == ROBOT_ARM_OK &&
        !(motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC && y == 0 &&
          (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Y)) != 0u))
        result = RobotArm_CheckTargetDirection(ROBOT_AXIS_Y, y);
    if (result == ROBOT_ARM_OK &&
        !(motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC && z == 0 &&
          (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Z)) != 0u))
        result = RobotArm_CheckTargetDirection(ROBOT_AXIS_Z, z);
    if (result == ROBOT_ARM_OK) result = RobotArm_CheckPoseSafety(x, y, z);
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    /* 提前校验搜索配置；原请求速度需另存，避免后续 Phase 的边界速度或零距离清零污染搜索速度。 */
    if (motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC)
    {
        if (x == 0 && (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_X)) != 0u) result = RobotArm_ValidateHomeConfig(ROBOT_AXIS_X,
            RobotArm_ResolveMoveSpeed(ROBOT_AXIS_X, x_speed),
            RobotArm_GetConfiguredHomeAcceleration(ROBOT_AXIS_X));
        if (result == ROBOT_ARM_OK && y == 0 && (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Y)) != 0u) result = RobotArm_ValidateHomeConfig(ROBOT_AXIS_Y,
            RobotArm_ResolveMoveSpeed(ROBOT_AXIS_Y, y_speed),
            RobotArm_GetConfiguredHomeAcceleration(ROBOT_AXIS_Y));
        if (result == ROBOT_ARM_OK && z == 0 && (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Z)) != 0u) result = RobotArm_ValidateHomeConfig(ROBOT_AXIS_Z,
            RobotArm_ResolveMoveSpeed(ROBOT_AXIS_Z, z_speed),
            RobotArm_GetConfiguredHomeAcceleration(ROBOT_AXIS_Z));
        if (result != ROBOT_ARM_OK) return result;
    }
    s_robot_arm.post_home_speed[ROBOT_AXIS_X] = x_speed;
    s_robot_arm.post_home_speed[ROBOT_AXIS_Y] = y_speed;
    s_robot_arm.post_home_speed[ROBOT_AXIS_Z] = z_speed;
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
    if ((motion_mode != ROBOT_MOVE_MOTION_XYZ_SYNC ||
         ((x != 0 || (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_X)) == 0u) &&
          (y != 0 || (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Y)) == 0u) &&
          (z != 0 || (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Z)) == 0u))) &&
        (s_robot_arm.move_axis_progress[ROBOT_AXIS_X] ==
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
    if (motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC)
    {
        if (apply_xy_speed_sync)
        {
            result = RobotArm_CalculateSyncMoveSpeed();
            if (result != ROBOT_ARM_OK)
            {
                return result;
            }
        }
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_XYZ_START;
        RobotArm_UpdateXyzSyncDebug();
    }
    else
    {
        s_robot_arm.move_to_state = ROBOT_MOVE_TO_X_START;
    }
    s_robot_arm.post_home_axis_mask = post_home_axis_mask;
    s_robot_arm.post_home_phase = (motion_mode == ROBOT_MOVE_MOTION_XYZ_SYNC &&
        ((x == 0 && (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_X)) != 0u) ||
         (y == 0 && (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Y)) != 0u) ||
         (z == 0 && (post_home_axis_mask & ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Z)) != 0u))) ?
        ROBOT_POST_HOME_NORMAL_MOVE : ROBOT_POST_HOME_NONE;
    s_robot_arm.post_home_start_ms = millis();
    s_robot_arm.operation = ROBOT_OP_MOVE_TO;
    s_robot_arm.state = ROBOT_ARM_MOVING;
    s_robot_arm.error_code = 0;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_NONE;
    return ROBOT_ARM_OK;
}

/**
 * 按 X/Y/Z 顺序启动带临时速度的普通 MOVE_TO，保留旧调用路径。
 *
 * @return 已受理时返回 ROBOT_ARM_OK；安全、传感器或驱动失败时返回对应错误码。
 */
RobotArmResult_t RobotArm_MoveToWithSpeed(int32_t x, int32_t y, int32_t z,
                                          uint16_t x_speed, uint16_t y_speed,
                                          uint16_t z_speed)
{
    return RobotArm_MoveToWithSpeedAndMode(
        x, y, z, x_speed, y_speed, z_speed, ROBOT_MOVE_MOTION_SEQUENTIAL);
}

/**
 * 为 0x40 已到理论零点但尚未命中传感器的单轴启动有限补充找零。
 *
 * 每段固定最多 5000 个 STEP，最多三段。传感器快照命中时会立即在
 * RobotArm_OnSensorSnapshotUpdated 中停止 DMA 并提交物理零点；三段均未命中则
 * 以 Home 超时失败，禁止继续向负方向搜索。
 *
 * @param axis 已完成理论零点普通移动、需要寻找实际 Home 传感器的机械轴。
 * @return 已启动搜索或已经由传感器确认零点时返回 1；启动失败或搜索次数耗尽返回 0。
 */
static uint8_t RobotArm_StartDelayedHomeSearch(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return 0u;
    }
    if (RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor(axis)))
    {
        RobotArm_CompleteAxisMoveAtHome(axis);
        s_robot_arm.move_axis_progress[axis] = ROBOT_MOVE_AXIS_COMPLETED;
        return 1u;
    }
    if (s_robot_arm.delayed_home_search_count[axis] >= ROBOT_ARM_DELAYED_HOME_MAX_CHUNKS)
    {
        RobotArm_FailPostMoveHome(axis, ROBOT_MOVE_END_TIMEOUT);
        return 0u;
    }
    if (RobotArm_StartHomeDriver(axis, RobotArm_GetHomeDirection(axis),
            ROBOT_ARM_POST_HOME_CHUNK_STEPS,
            RobotArm_ResolveMoveSpeed(axis, s_robot_arm.move_to_speed[axis]),
            RobotArm_GetConfiguredHomeAcceleration(axis)) != ROBOT_ARM_OK ||
        !RobotArmDriver_IsBusy(axis))
    {
        RobotArm_FailPostMoveHome(axis, ROBOT_MOVE_END_DRIVER_ERROR);
        return 0u;
    }
    s_robot_arm.delayed_home_search_count[axis]++;
    s_robot_arm.delayed_home_search_mask |= ROBOT_ARM_AXIS_MASK(axis);
    robot_axis->state = ROBOT_AXIS_HOMING;
    robot_axis->position_valid = 0u;
    robot_axis->active = 1u;
    robot_axis->command_steps = ROBOT_ARM_POST_HOME_CHUNK_STEPS;
    robot_axis->target_position = 0;
    robot_axis->end_reason = ROBOT_MOVE_END_NONE;
    s_robot_arm.move_axis_progress[axis] = ROBOT_MOVE_AXIS_RUNNING;
    return 1u;
}

/**
 * 推进 0x40 单轴有限补充找零，并在每段完整结束后决定续段或失败。
 *
 * @param axis 正在补充寻找对应 S1/S2/S3 的机械轴。
 * @return 已完成返回 1，仍在搜索返回 0，搜索失败返回 -1。
 */
static int8_t RobotArm_TaskDelayedHomeSearch(RobotAxisId_t axis)
{
    RobotAxis_t *robot_axis = RobotArm_GetAxis(axis);
    if (robot_axis == 0)
    {
        return -1;
    }
    if ((!robot_axis->active && robot_axis->end_reason == ROBOT_MOVE_END_COMPLETED) ||
        RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor(axis)))
    {
        RobotArm_CompleteAxisMoveAtHome(axis);
        s_robot_arm.delayed_home_search_mask &= (uint8_t)~ROBOT_ARM_AXIS_MASK(axis);
        return 1;
    }
    if (RobotArmDriver_IsBusy(axis))
    {
        return 0;
    }
    if ((RobotArmDriver_GetRemainingSteps(axis) != 0u) ||
        (RobotArmDriver_GetCompletedSteps(axis) < robot_axis->command_steps))
    {
        RobotArm_FailPostMoveHome(axis, ROBOT_MOVE_END_DRIVER_ERROR);
        return -1;
    }
    robot_axis->active = 0u;
    if (!RobotArm_StartDelayedHomeSearch(axis))
    {
        return -1;
    }
    return (s_robot_arm.move_axis_progress[axis] == ROBOT_MOVE_AXIS_COMPLETED) ? 1 : 0;
}

/**
 * 将一条 Phase 的绝对目标坐标转换为既有 MOVE_TO 状态机可执行的动作。
 *
 * X/Y 继续沿用现有独立 DMA、限位、完成步数与坐标提交链路，只在真正启动 DMA 时
 * 注入批次层已继承的起始速度、请求终止速度和加速时间；Z 不参与该频率规划，保持独立 z_speed。
 * 全轴静止、XY 缺少终止速度或 Z 运动而 z_speed 为零时拒绝，避免没有 DMA 终态或非法速度。
 *
 * @param phase 单条绝对目标 Phase，速度单位为 steps/s。
 * @return 已启动时返回 ROBOT_ARM_OK；配置无效或机械臂忙时返回对应错误。
 */
RobotArmResult_t RobotArm_StartPhase(const RobotArmPhase_t *phase)
{
    int64_t target_x;
    int64_t target_y;
    int64_t target_z;
    uint32_t x_steps;
    uint32_t y_steps;
    uint8_t z_moves;
    uint32_t x_start;
    uint32_t y_start;
    uint32_t x_terminal;
    uint32_t y_terminal;
    uint16_t x_speed;
    uint16_t y_speed;
    uint8_t post_home_axis_mask = 0u;
    RobotArmResult_t result;
    if (phase == 0)
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    target_x = phase->target_x;
    target_y = phase->target_y;
    target_z = phase->target_z;
    if ((target_x > 2147483647L) || (target_x < (-2147483647L - 1L)) ||
        (target_y > 2147483647L) || (target_y < (-2147483647L - 1L)) ||
        (target_z > 2147483647L) || (target_z < (-2147483647L - 1L)))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    x_steps = (uint32_t)((target_x >= s_robot_arm.axis[ROBOT_AXIS_X].current_position) ?
              (target_x - s_robot_arm.axis[ROBOT_AXIS_X].current_position) :
              (s_robot_arm.axis[ROBOT_AXIS_X].current_position - target_x));
    y_steps = (uint32_t)((target_y >= s_robot_arm.axis[ROBOT_AXIS_Y].current_position) ?
              (target_y - s_robot_arm.axis[ROBOT_AXIS_Y].current_position) :
              (s_robot_arm.axis[ROBOT_AXIS_Y].current_position - target_y));
    z_moves = (target_z != s_robot_arm.axis[ROBOT_AXIS_Z].current_position) ? 1u : 0u;
    if (((x_steps == 0u) && (y_steps == 0u) && !z_moves) ||
        (z_moves && (phase->z_speed == 0u)))
    {
        return ROBOT_ARM_ERR_CONFIG;
    }
    x_start = 0u;
    y_start = 0u;
    x_terminal = 0u;
    y_terminal = 0u;
    if ((x_steps != 0u) || (y_steps != 0u))
    {
        /* PB10/PB11 的独立 DMA 驱动均限定 50000 steps/s，超限由协议拒绝而非静默截断。 */
        if ((phase->x_terminal_speed > 50000u) ||
            (phase->y_terminal_speed > 50000u) ||
            ((x_steps != 0u) && (phase->x_terminal_speed == 0u)) ||
            ((y_steps != 0u) && (phase->y_terminal_speed == 0u)))
        {
            return ROBOT_ARM_ERR_CONFIG;
        }
        /*
         * 首次参与 Batch 的轴以安全频率起步；其余条目使用协议层继承的同轴终止速度。
         * 终止速度和加速时间均原样进入 PB10/PB11，不能再按 XY 位移比例改写。
         */
        x_start = (phase->x_start_speed == 0u) ?
            ROBOT_ARM_PHASE_BOUNDARY_FREQUENCY : phase->x_start_speed;
        x_terminal = phase->x_terminal_speed;
        y_start = (phase->y_start_speed == 0u) ?
            ROBOT_ARM_PHASE_BOUNDARY_FREQUENCY : phase->y_start_speed;
        y_terminal = phase->y_terminal_speed;
    }
    x_speed = (x_terminal > 65535u) ? 65535u : (uint16_t)((x_terminal == 0u) ? 1u : x_terminal);
    y_speed = (y_terminal > 65535u) ? 65535u : (uint16_t)((y_terminal == 0u) ? 1u : y_terminal);
    /* 只有本条实际运动的轴，目标零点才触发既有补充找零语义。 */
    if (x_steps != 0u)
    {
        post_home_axis_mask |= ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_X);
    }
    if (y_steps != 0u)
    {
        post_home_axis_mask |= ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Y);
    }
    if (z_moves)
    {
        post_home_axis_mask |= ROBOT_ARM_AXIS_MASK(ROBOT_AXIS_Z);
    }
    result = RobotArm_StartMoveToWithSpeedAndMode(
        (int32_t)target_x, (int32_t)target_y, (int32_t)target_z,
        x_speed, y_speed,
        z_moves ? phase->z_speed : 1u,
        ROBOT_MOVE_MOTION_XYZ_SYNC, post_home_axis_mask, 0u);
    if (result != ROBOT_ARM_OK)
    {
        return result;
    }
    s_robot_arm.phase_xy_profile = ((x_steps != 0u) || (y_steps != 0u)) ? 1u : 0u;
    s_robot_arm.phase_start_frequency[ROBOT_AXIS_X] = x_start;
    s_robot_arm.phase_start_frequency[ROBOT_AXIS_Y] = y_start;
    s_robot_arm.phase_terminal_frequency[ROBOT_AXIS_X] = x_terminal;
    s_robot_arm.phase_terminal_frequency[ROBOT_AXIS_Y] = y_terminal;
    s_robot_arm.phase_acceleration_time_ms[ROBOT_AXIS_X] = phase->x_acceleration_time_ms;
    s_robot_arm.phase_acceleration_time_ms[ROBOT_AXIS_Y] = phase->y_acceleration_time_ms;
    /* Z 轴运动时保持配置速度；静止时维持安全占位值，不向定时器写入零速。 */
    s_robot_arm.move_to_speed[ROBOT_AXIS_Z] = z_moves ? phase->z_speed : 1u;
    return ROBOT_ARM_OK;
}

/**
 * 按指定运动方式启动一次非阻塞普通目标位置任务。
 *
 * 对外公开的并发 MOVE_TO 保持三轴零目标均可补充找零的既有语义，并严格保留各轴请求速度；
 * Phase 才通过内部入口启用 X/Y 配速，避免改变 0x34 的协议行为。
 *
 * @param x X 轴目标绝对逻辑坐标，单位为步数。
 * @param y Y 轴目标绝对逻辑坐标，单位为步数。
 * @param z Z 轴目标绝对逻辑坐标，单位为步数。
 * @param x_speed X 轴最大速度，单位为 steps/s；0 表示使用既有默认速度。
 * @param y_speed Y 轴最大速度，单位为 steps/s；0 表示使用既有默认速度。
 * @param z_speed Z 轴最大速度，单位为 steps/s；0 表示使用既有默认速度。
 * @param motion_mode SEQUENTIAL 顺序移动；XYZ_SYNC 并发启动各轴且保留各自速度，对零目标允许从未知坐标直接搜索，均以传感器命中为完成条件。
 * @return 已受理返回 ROBOT_ARM_OK；坐标、传感器、安全检查或驱动前置条件失败时返回对应错误码。
 */
RobotArmResult_t RobotArm_MoveToWithSpeedAndMode(
    int32_t x, int32_t y, int32_t z, uint16_t x_speed, uint16_t y_speed,
    uint16_t z_speed, RobotMoveMotionMode_t motion_mode)
{
    return RobotArm_StartMoveToWithSpeedAndMode(
        x, y, z, x_speed, y_speed, z_speed, motion_mode,
        ROBOT_ARM_AXIS_MASK_ALL, 0u);
}

/**
 * 启动 0x40 的独立错峰绝对移动，保留既有单轴绝对坐标和完成步数确认链路。
 *
 * 本函数只建立延时计划，实际 DMA 只能由 RobotArm_Task 在各轴到期后启动；因此
 * ACK 不能被解释为任何一个电机已开始或已完成运动。
 *
 * @param x X 轴目标绝对逻辑坐标，单位为步数。
 * @param y Y 轴目标绝对逻辑坐标，单位为步数。
 * @param z Z 轴目标绝对逻辑坐标，单位为步数。
 * @param x_speed X 轴速度，单位为 steps/s，必须非 0。
 * @param y_speed Y 轴速度，单位为 steps/s，必须非 0。
 * @param z_speed Z 轴速度，单位为 steps/s，必须非 0。
 * @param x_delay_100ms X 轴启动延时，单位为 100ms。
 * @param y_delay_100ms Y 轴启动延时，单位为 100ms。
 * @param z_delay_100ms Z 轴启动延时，单位为 100ms。
 * @return 延时计划已受理返回 ROBOT_ARM_OK；前置条件不满足时返回错误码。
 */
RobotArmResult_t RobotArm_MoveToWithDelayedStart(
    int32_t x, int32_t y, int32_t z, uint16_t x_speed, uint16_t y_speed,
    uint16_t z_speed, uint8_t x_delay_100ms, uint8_t y_delay_100ms,
    uint8_t z_delay_100ms)
{
    int32_t targets[ROBOT_AXIS_COUNT];
    uint16_t speeds[ROBOT_AXIS_COUNT];
    uint8_t delays[ROBOT_AXIS_COUNT];
    uint8_t index;
    int64_t delta;
    RobotArmResult_t result;

    if (s_robot_arm.state == ROBOT_ARM_ERROR)
    {
        return (RobotArmResult_t)s_robot_arm.error_code;
    }
    if (RobotArm_IsBusy())
    {
        return ROBOT_ARM_ERR_BUSY;
    }
    if (!RobotArmSensor_IsReady())
    {
        return ROBOT_ARM_ERR_SENSOR;
    }
    targets[ROBOT_AXIS_X] = x; targets[ROBOT_AXIS_Y] = y; targets[ROBOT_AXIS_Z] = z;
    speeds[ROBOT_AXIS_X] = x_speed; speeds[ROBOT_AXIS_Y] = y_speed; speeds[ROBOT_AXIS_Z] = z_speed;
    delays[ROBOT_AXIS_X] = x_delay_100ms; delays[ROBOT_AXIS_Y] = y_delay_100ms; delays[ROBOT_AXIS_Z] = z_delay_100ms;
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        if (!s_robot_arm.axis[index].position_valid)
        {
            return ROBOT_ARM_ERR_POSITION_UNKNOWN;
        }
        result = RobotArm_CheckAxisTarget((RobotAxisId_t)index, targets[index]);
        if (result != ROBOT_ARM_OK) return result;
        /* 零目标允许理论到位后继续以有限段数寻找真实 Home，不能在预检查时当作负向越限拒绝。 */
        if (targets[index] != 0)
        {
            result = RobotArm_CheckTargetDirection((RobotAxisId_t)index, targets[index]);
            if (result != ROBOT_ARM_OK) return result;
        }
    }
    result = RobotArm_CheckPoseSafety(x, y, z);
    if (result != ROBOT_ARM_OK) return result;

    s_robot_arm.phase_xy_profile = 0u;
    s_robot_arm.post_home_phase = ROBOT_POST_HOME_NONE;
    s_robot_arm.post_home_axis_mask = 0u;
    RobotArm_ResetMoveDebug();
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        delta = (int64_t)targets[index] - s_robot_arm.axis[index].current_position;
        s_robot_arm.move_to_target[index] = targets[index];
        s_robot_arm.move_to_speed[index] = speeds[index];
        s_robot_arm.delayed_start_100ms[index] = delays[index];
        s_robot_arm.axis[index].target_position = targets[index];
        /* 零目标即使理论坐标已为零，也必须在本轴延时到期后确认传感器或有限补充找零。 */
        s_robot_arm.move_axis_progress[index] = (delta == 0 && targets[index] != 0) ?
            ROBOT_MOVE_AXIS_NOT_REQUIRED : ROBOT_MOVE_AXIS_WAIT_START;
        s_move_debug.received_current[index] = s_robot_arm.axis[index].current_position;
        s_move_debug.received_target[index] = targets[index];
        s_move_debug.received_delta[index] = delta;
        s_move_debug.axis_progress[index] = s_robot_arm.move_axis_progress[index];
    }
    if ((s_robot_arm.move_axis_progress[ROBOT_AXIS_X] == ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
        (s_robot_arm.move_axis_progress[ROBOT_AXIS_Y] == ROBOT_MOVE_AXIS_NOT_REQUIRED) &&
        (s_robot_arm.move_axis_progress[ROBOT_AXIS_Z] == ROBOT_MOVE_AXIS_NOT_REQUIRED))
    {
        s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_COMPLETED;
        s_move_debug.finalize_reason = ROBOT_MOVE_FINALIZE_COMPLETED;
        return ROBOT_ARM_OK;
    }
    s_robot_arm.delayed_start_base_ms = millis();
    s_robot_arm.move_to_state = ROBOT_MOVE_TO_DELAYED_WAIT;
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
    s_robot_arm.home_acceleration = 0u;
    result = RobotArm_BeginHomeAxis(axis);
    if (result == ROBOT_ARM_OK)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_HOME_X + axis);
        s_robot_arm.error_code = 0;
    }
    return result;
}

/**
 * 以协议指定的快速速度启动单轴 Home，并使用该轴默认加速度。
 *
 * @param axis 需要置零的实际机械轴。
 * @param home_speed 第一次快速寻找 Home 传感器的速度，单位为 steps/s。
 * @return 已受理返回 ROBOT_ARM_OK；失败时返回当前配置、传感器或状态错误。
 */
RobotArmResult_t RobotArm_HomeAxisWithSpeed(RobotAxisId_t axis,
                                            uint16_t home_speed)
{
    return RobotArm_HomeAxisWithSpeedAndAcceleration(axis, home_speed, 0u);
}

/**
 * 以协议指定的速度和加速度启动单轴 Home，仅执行一次快速寻零。
 *
 * 加速度传 0 时仅在这里回退为对应轴默认值，确保后续启动底层 DMA 时始终得到
 * 非零物理参数；不允许以默认值掩盖空的速度字段。
 *
 * @param axis 需要置零的实际机械轴。
 * @param home_speed 第一次快速寻找 Home 传感器的速度，单位为 steps/s。
 * @param home_acceleration 第一次快速寻找 Home 传感器的加速度，单位为 steps/s^2。
 * @return 已受理返回 ROBOT_ARM_OK；失败时返回当前配置、传感器或状态错误。
 */
RobotArmResult_t RobotArm_HomeAxisWithSpeedAndAcceleration(
    RobotAxisId_t axis, uint16_t home_speed, uint16_t home_acceleration)
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
    s_robot_arm.home_acceleration = home_acceleration;
    result = RobotArm_BeginHomeAxis(axis);
    if (result == ROBOT_ARM_OK)
    {
        s_robot_arm.operation = (RobotArmOperation_t)(ROBOT_OP_HOME_X + axis);
        s_robot_arm.error_code = 0;
    }
    else
    {
        s_robot_arm.home_fast_speed = 0u;
        s_robot_arm.home_acceleration = 0u;
    }
    return result;
}

/**
 * 同时启动 X、Y、Z 的非阻塞 HomeAll。
 *
 * CMD=30 的三轴寻零没有机械顺序依赖：每轴以自身标定快速速度朝 S1/S2/S3 寻零，
 * 并仅在自身传感器命中后停止。任一未完成轴超时或驱动失败会安全停止其他仍在运行的
 * Home 轴；只有三轴均确认零点才正常完成。
 *
 * @return 三轴 Home 配置和传感器均可用时返回 ROBOT_ARM_OK；否则不启动任意轴并返回错误。
 */
RobotArmResult_t RobotArm_Home(void)
{
    RobotArmResult_t result;
    uint8_t index;
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
        ROBOT_AXIS_Z, RobotArm_GetConfiguredHomeFastSpeed(ROBOT_AXIS_Z),
        RobotArm_GetConfiguredHomeAcceleration(ROBOT_AXIS_Z));
    if (result == ROBOT_ARM_OK)
    {
        result = RobotArm_ValidateHomeConfig(
            ROBOT_AXIS_Y, RobotArm_GetConfiguredHomeFastSpeed(ROBOT_AXIS_Y),
            RobotArm_GetConfiguredHomeAcceleration(ROBOT_AXIS_Y));
    }
    if (result == ROBOT_ARM_OK)
    {
        result = RobotArm_ValidateHomeConfig(
            ROBOT_AXIS_X, RobotArm_GetConfiguredHomeFastSpeed(ROBOT_AXIS_X),
            RobotArm_GetConfiguredHomeAcceleration(ROBOT_AXIS_X));
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
    s_robot_arm.home_all_pending_mask = ROBOT_ARM_AXIS_MASK_ALL;
    /* HomeAll 的快速/脱离速度保持各轴 MCU 配置，不能继承 0x31 的临时字段。 */
    s_robot_arm.home_fast_speed = 0u;
    s_robot_arm.home_acceleration = 0u;
    s_robot_arm.home_start_ms = millis();
    s_robot_arm.home_state = ROBOT_HOME_CHECK_SENSOR;
    s_robot_arm.operation = ROBOT_OP_HOME_ALL;
    s_robot_arm.state = ROBOT_ARM_HOMING;
    s_robot_arm.error_code = 0;
    s_robot_arm.last_move_end_reason = ROBOT_MOVE_END_NONE;
    for (index = 0u; index < ROBOT_AXIS_Z; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        robot_axis->homed = 0u;
        robot_axis->position_valid = 0u;
        robot_axis->target_position = 0;
        robot_axis->state = ROBOT_AXIS_HOMING;
        robot_axis->active = 0u;
        robot_axis->end_reason = ROBOT_MOVE_END_NONE;
        s_robot_arm.home_all_axis_state[index] = ROBOT_HOME_CHECK_SENSOR;
    }
    return ROBOT_ARM_OK;
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
 * 在完整 HC165 快照更新后立即按 S1/S2/S3 的物理零点语义处理运动轴。
 *
 * S1=X、S2=Y、S3=Z 的 Active 均表示对应轴实际位置为 0。负向运行首次命中时
 * 除同步零目标的普通移动阶段外，必须立刻停止 STEP/DMA/TIM；目标为 0 的普通运动
 * 与 Home 都正常完成并建立可信零点。只有已在 Active 零点还请求继续负向时，才由
 * 启动前方向检查拒绝为 LIMIT。
 */
void RobotArm_OnSensorSnapshotUpdated(void)
{
    uint8_t index;
    RobotArm_UpdateDebugWatch();
    for (index = 0u; index < ROBOT_AXIS_COUNT; index++)
    {
        RobotAxis_t *robot_axis = &s_robot_arm.axis[index];
        RobotAxisId_t axis = (RobotAxisId_t)index;
        /* CMD=30 三轴并发寻零由独立 pending bit 管理，不能落入单轴 Home 的 home_axis 分支。 */
        if ((s_robot_arm.operation == ROBOT_OP_HOME_ALL) &&
            ((s_robot_arm.home_all_pending_mask & ROBOT_ARM_AXIS_MASK(axis)) != 0u))
        {
            if (RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor(axis)) &&
                (s_robot_arm.home_all_axis_state[axis] == ROBOT_HOME_SEEK_FAST))
            {
                RobotArm_CompleteHomeAllAxis(axis);
            }
            RobotArm_UpdateDebugWatch();
            continue;
        }
        /* 同步零目标第一阶段保持原坐标和完整脉冲，等待全轴结束后统一检查 Home。 */
        if (RobotArm_ShouldDeferHome(axis)) continue;
        if (RobotArmSensor_IsTriggered(RobotArm_GetHomeSensor(axis)))
        {
            /* 即使当前没有任务，Active 也是真实零点证据，不能让旧坐标继续存在。 */
            RobotArm_SetAxisPositionToHome(axis);
        }
        if (RobotArmDriver_ShouldStopForNegativeLimit((RobotAxisId_t)index))
        {
            /* Home 快速寻零命中 S1/S2/S3 是成功条件，不得按普通越限置 ERROR。 */
            if (((s_robot_arm.operation == ROBOT_OP_HOME_X) ||
                 (s_robot_arm.operation == ROBOT_OP_HOME_Y) ||
                 (s_robot_arm.operation == ROBOT_OP_HOME_Z) ||
                 (s_robot_arm.operation == ROBOT_OP_HOME_ALL)) &&
                (s_robot_arm.home_axis == (RobotAxisId_t)index) &&
                ((s_robot_arm.home_state == ROBOT_HOME_SEEK_FAST) ||
                 (s_robot_arm.home_state == ROBOT_HOME_SEEK_KNOWN) ||
                 (s_robot_arm.home_state == ROBOT_HOME_EXTRA_SEARCH) ||
                 (s_robot_arm.home_state == ROBOT_HOME_UNKNOWN_SEARCH)))
            {
                RobotArmDriver_Stop((RobotAxisId_t)index);
                s_robot_arm.axis[index].active = 0u;
                if (index == ROBOT_AXIS_X) dbg_x_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                if (index == ROBOT_AXIS_Y) dbg_y_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                if (index == ROBOT_AXIS_Z) dbg_z_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                RobotArm_CompleteHomeAxis();
                RobotArm_UpdateDebugWatch();
                continue;
            }
            if (robot_axis->target_position == 0)
            {
                RobotArm_CompleteAxisMoveAtHome(axis);
                if (index == ROBOT_AXIS_X) dbg_x_last_stop_reason = ROBOT_MOVE_END_COMPLETED;
                if (index == ROBOT_AXIS_Y) dbg_y_last_stop_reason = ROBOT_MOVE_END_COMPLETED;
                if (index == ROBOT_AXIS_Z) dbg_z_last_stop_reason = ROBOT_MOVE_END_COMPLETED;
            }
            else
            {
                RobotArm_FailAxisMoveAtUnexpectedHome(axis);
                if (index == ROBOT_AXIS_X) dbg_x_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                if (index == ROBOT_AXIS_Y) dbg_y_last_stop_reason = ROBOT_MOVE_END_SENSOR;
                if (index == ROBOT_AXIS_Z) dbg_z_last_stop_reason = ROBOT_MOVE_END_SENSOR;
            }
            RobotArm_UpdateDebugWatch();
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
