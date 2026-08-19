#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__

#include <stdint.h>
#include "robot_arm_driver.h"

typedef enum
{
    ROBOT_AXIS_IDLE = 0,
    ROBOT_AXIS_MOVING,
    ROBOT_AXIS_HOMING,
    ROBOT_AXIS_ERROR
} RobotAxisState_t;

typedef enum
{
    ROBOT_ARM_IDLE = 0,
    ROBOT_ARM_MOVING,
    ROBOT_ARM_HOMING,
    ROBOT_ARM_ERROR
} RobotArmState_t;

typedef enum
{
    ROBOT_OP_NONE = 0,
    ROBOT_OP_MOVE_X,
    ROBOT_OP_MOVE_Y,
    ROBOT_OP_MOVE_Z,
    ROBOT_OP_MOVE_TO,
    ROBOT_OP_MOVE_TO_SAFE,
    ROBOT_OP_HOME_X,
    ROBOT_OP_HOME_Y,
    ROBOT_OP_HOME_Z,
    ROBOT_OP_HOME_ALL
} RobotArmOperation_t;

typedef enum
{
    ROBOT_MOVE_END_NONE = 0,
    ROBOT_MOVE_END_COMPLETED,
    ROBOT_MOVE_END_STOPPED,
    ROBOT_MOVE_END_SENSOR,
    ROBOT_MOVE_END_LIMIT,
    ROBOT_MOVE_END_INTERLOCK,
    ROBOT_MOVE_END_TIMEOUT,
    ROBOT_MOVE_END_DRIVER_ERROR
} RobotMoveEndReason_t;

typedef enum
{
    ROBOT_HOME_IDLE = 0,
    ROBOT_HOME_CHECK_SENSOR,
    ROBOT_HOME_BACKOFF_IF_ACTIVE,
    ROBOT_HOME_SEEK_FAST,
    ROBOT_HOME_BACKOFF_AFTER_TRIGGER,
    ROBOT_HOME_SEEK_SLOW,
    ROBOT_HOME_DONE,
    ROBOT_HOME_ERROR
} RobotHomeState_t;

typedef enum
{
    ROBOT_MOVE_TO_IDLE = 0,
    ROBOT_MOVE_TO_X_START,
    ROBOT_MOVE_TO_X_WAIT,
    ROBOT_MOVE_TO_Y_START,
    ROBOT_MOVE_TO_Y_WAIT,
    ROBOT_MOVE_TO_Z_START,
    ROBOT_MOVE_TO_Z_WAIT,
    ROBOT_MOVE_TO_DONE,
    ROBOT_MOVE_TO_ERROR
} RobotMoveToState_t;

typedef enum
{
    ROBOT_MOVE_AXIS_NOT_REQUIRED = 0,
    ROBOT_MOVE_AXIS_WAIT_START,
    ROBOT_MOVE_AXIS_RUNNING,
    ROBOT_MOVE_AXIS_COMPLETED
} RobotMoveAxisProgress_t;

typedef enum
{
    ROBOT_MOVE_FINALIZE_NONE = 0,
    ROBOT_MOVE_FINALIZE_WAIT_AXIS,
    ROBOT_MOVE_FINALIZE_DRIVER_BUSY,
    ROBOT_MOVE_FINALIZE_POSITION_MISMATCH,
    ROBOT_MOVE_FINALIZE_COMPLETED,
    ROBOT_MOVE_FINALIZE_START_FAILED,
    ROBOT_MOVE_FINALIZE_STOPPED
} RobotMoveFinalizeReason_t;

typedef enum
{
    ROBOT_SAFE_MOVE_IDLE = 0,
    ROBOT_SAFE_MOVE_PREPARE,
    ROBOT_SAFE_MOVE_RAISE_Z_START,
    ROBOT_SAFE_MOVE_RAISE_Z_WAIT,
    ROBOT_SAFE_MOVE_X_START,
    ROBOT_SAFE_MOVE_X_WAIT,
    ROBOT_SAFE_MOVE_Y_START,
    ROBOT_SAFE_MOVE_Y_WAIT,
    ROBOT_SAFE_MOVE_FINAL_Z_START,
    ROBOT_SAFE_MOVE_FINAL_Z_WAIT,
    ROBOT_SAFE_MOVE_DONE,
    ROBOT_SAFE_MOVE_ERROR
} RobotSafeMoveState_t;

typedef enum
{
    ROBOT_ARM_OK = 0,
    ROBOT_ARM_ERR_BUSY,
    ROBOT_ARM_ERR_POSITION_UNKNOWN,
    ROBOT_ARM_ERR_NOT_HOMED,
    ROBOT_ARM_ERR_LIMIT,
    ROBOT_ARM_ERR_INTERLOCK,
    ROBOT_ARM_ERR_DRIVER,
    ROBOT_ARM_ERR_SENSOR,
    ROBOT_ARM_ERR_HOME_TIMEOUT,
    ROBOT_ARM_ERR_MOVE_TIMEOUT,
    ROBOT_ARM_ERR_STOPPED,
    ROBOT_ARM_ERR_NOT_SUPPORTED,
    ROBOT_ARM_ERR_CONFIG
} RobotArmResult_t;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t target_x;
    int32_t target_y;
    int32_t target_z;
    uint8_t x_homed;
    uint8_t y_homed;
    uint8_t z_homed;
    uint8_t x_valid;
    uint8_t y_valid;
    uint8_t z_valid;
    uint8_t s1_x_home;
    uint8_t s2_y_home;
    uint8_t s3_z_home;
    uint8_t s4_z_lower_limit;
    RobotAxisState_t x_state;
    RobotAxisState_t y_state;
    RobotAxisState_t z_state;
    RobotArmState_t arm_state;
    RobotArmOperation_t operation;
    RobotHomeState_t home_state;
    RobotMoveToState_t move_to_state;
    RobotSafeMoveState_t safe_move_state;
    RobotMoveEndReason_t last_move_end_reason;
    int32_t error_code;
} RobotArmStatus_t;

typedef struct
{
    int32_t received_current[ROBOT_AXIS_COUNT];
    int32_t received_target[ROBOT_AXIS_COUNT];
    int64_t received_delta[ROBOT_AXIS_COUNT];
    RobotMoveAxisProgress_t axis_progress[ROBOT_AXIS_COUNT];
    uint8_t start_called[ROBOT_AXIS_COUNT];
    uint32_t start_steps[ROBOT_AXIS_COUNT];
    uint8_t start_result[ROBOT_AXIS_COUNT];
    uint8_t busy_before[ROBOT_AXIS_COUNT];
    uint8_t busy_after[ROBOT_AXIS_COUNT];
    RobotMoveFinalizeReason_t finalize_reason;
} RobotArmMoveDebug_t;

/** 初始化 XYZ 机械臂管理层；不会启动任意电机。 */
void RobotArm_Init(void);
/** 在主循环中推进 Home、单轴和 MoveTo 状态机。 */
void RobotArm_Task(void);
/** 启动 X 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveXRelative(int32_t delta, uint32_t speed);
/** 启动 Y 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveYRelative(int32_t delta, uint32_t speed);
/** 启动 Z 轴相对 STEP 运动。 */
RobotArmResult_t RobotArm_MoveZRelative(int32_t delta, uint32_t speed);
/** 启动 X 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveX(int32_t target, uint32_t speed);
/** 启动 Y 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveY(int32_t target, uint32_t speed);
/** 启动 Z 轴绝对 STEP 运动，要求当前坐标可信。 */
RobotArmResult_t RobotArm_MoveZ(int32_t target, uint32_t speed);
/** 按 X、Y、Z 顺序启动非阻塞目标位置任务。 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z);
/** 按“必要时抬高 Z、移动 X/Y、最后移动 Z”的安全路径接受绝对位置任务。 */
RobotArmResult_t RobotArm_MoveToSafe(int32_t x, int32_t y, int32_t z);
/** 启动指定轴的非阻塞双阶段 Home。 */
RobotArmResult_t RobotArm_HomeAxis(RobotAxisId_t axis);
/** 按 Z、Y、X 顺序启动非阻塞 HomeAll。 */
RobotArmResult_t RobotArm_Home(void);
/** 停止全部三轴并彻底结束当前组合操作。 */
void RobotArm_Stop(void);
/** 清除管理层 ERROR；不会恢复坐标有效性或回零状态。 */
RobotArmResult_t RobotArm_ClearError(void);
/** 使旧入口已直接移动的轴坐标失效。 */
void RobotArm_InvalidatePosition(RobotAxisId_t axis);
/** 查询机械臂是否存在正在执行的独占动作。 */
uint8_t RobotArm_IsBusy(void);
/** 查询机械臂整体状态。 */
RobotArmState_t RobotArm_GetState(void);
/** 查询指定轴状态。 */
RobotAxisState_t RobotArm_GetAxisState(RobotAxisId_t axis);
/** 查询 X 轴当前逻辑坐标。 */
int32_t RobotArm_GetX(void);
/** 查询 Y 轴当前逻辑坐标。 */
int32_t RobotArm_GetY(void);
/** 查询 Z 轴当前逻辑坐标。 */
int32_t RobotArm_GetZ(void);
/** 查询指定轴坐标是否可信。 */
uint8_t RobotArm_IsPositionValid(RobotAxisId_t axis);
/** 查询指定轴是否在本次上电后成功建立过原点。 */
uint8_t RobotArm_IsHomed(RobotAxisId_t axis);
/** 获取机械臂坐标、状态、传感器和结束原因快照。 */
void RobotArm_GetStatus(RobotArmStatus_t *status);
/** 获取最近一次 MOVE_TO 的接收、启动和收尾诊断快照；不参与 V2 STATUS 打包。 */
void RobotArm_GetMoveDebug(RobotArmMoveDebug_t *debug);
/** 检查目标姿态安全性；当前无标定碰撞区时默认通过。 */
RobotArmResult_t RobotArm_CheckPoseSafety(int32_t x, int32_t y, int32_t z);
/** 检查两个姿态之间是否允许直接转换。 */
RobotArmResult_t RobotArm_CheckTransitionSafety(int32_t current_x,
                                                int32_t current_y,
                                                int32_t current_z,
                                                int32_t target_x,
                                                int32_t target_y,
                                                int32_t target_z);

#endif
