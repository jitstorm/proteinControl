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
    ROBOT_OP_HOME_X,
    ROBOT_OP_HOME_Y,
    ROBOT_OP_HOME_Z,
    ROBOT_OP_HOME_ALL
} RobotArmOperation_t;

typedef enum
{
    ROBOT_ARM_OK = 0,
    ROBOT_ARM_ERR_BUSY,
    ROBOT_ARM_ERR_POSITION_UNKNOWN,
    ROBOT_ARM_ERR_LIMIT,
    ROBOT_ARM_ERR_DRIVER,
    ROBOT_ARM_ERR_NOT_SUPPORTED
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
    RobotAxisState_t x_state;
    RobotAxisState_t y_state;
    RobotAxisState_t z_state;
    RobotArmState_t arm_state;
    RobotArmOperation_t operation;
    int32_t error_code;
} RobotArmStatus_t;

/** 初始化 XYZ 机械臂管理层；不会启动任意电机。 */
void RobotArm_Init(void);
/** 在主循环中推进已启动动作的完成状态，不包含阻塞等待。 */
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
/** 停止全部三轴；中途停止的活动轴坐标将标记为不可信。 */
void RobotArm_Stop(void);
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
/** 查询指定轴是否已完成可靠回零。 */
uint8_t RobotArm_IsHomed(RobotAxisId_t axis);
/** 获取 XYZ 坐标、有效性及运动状态快照。 */
void RobotArm_GetStatus(RobotArmStatus_t *status);
/** 预留 XYZ 串行 MoveTo 接口，当前不启用多轴状态机。 */
RobotArmResult_t RobotArm_MoveTo(int32_t x, int32_t y, int32_t z);
/** 预留 Home 接口，未确认原点传感器前不启动任意轴。 */
RobotArmResult_t RobotArm_HomeAxis(RobotAxisId_t axis);

#endif
