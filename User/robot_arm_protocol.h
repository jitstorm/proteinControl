#ifndef __ROBOT_ARM_PROTOCOL_H
#define __ROBOT_ARM_PROTOCOL_H

#include <stdint.h>
#include "protocol_v2.h"

typedef enum
{
    ROBOT_ARM_CMD_HOME = 0x30,
    ROBOT_ARM_CMD_HOME_AXIS = 0x31,
    ROBOT_ARM_CMD_MOVE_AXIS_ABS = 0x32,
    ROBOT_ARM_CMD_MOVE_AXIS_REL = 0x33,
    ROBOT_ARM_CMD_MOVE_TO = 0x34,
    ROBOT_ARM_CMD_MOVE_TO_SAFE = 0x35,
    ROBOT_ARM_CMD_STOP = 0x36,
    ROBOT_ARM_CMD_CLEAR_ERROR = 0x37,
    ROBOT_ARM_CMD_STATUS = 0x38,
    ROBOT_ARM_CMD_ACK = 0x70,
    ROBOT_ARM_CMD_EVENT = 0x71,
    ROBOT_ARM_CMD_STATUS_RSP = 0x72
} RobotArmProtocolCommand_t;

typedef enum
{
    ROBOT_ARM_ACK_ACCEPTED = 0,
    ROBOT_ARM_ACK_REJECTED = 1
} RobotArmProtocolAck_t;

typedef enum
{
    ROBOT_ARM_EVENT_COMPLETED = 0,
    ROBOT_ARM_EVENT_STOPPED = 1,
    ROBOT_ARM_EVENT_LIMIT = 2,
    ROBOT_ARM_EVENT_TIMEOUT = 3,
    ROBOT_ARM_EVENT_INTERLOCK = 4,
    ROBOT_ARM_EVENT_SENSOR_ERROR = 5,
    ROBOT_ARM_EVENT_DRIVER_ERROR = 6,
    ROBOT_ARM_EVENT_HOME_COMPLETED = 7,
    ROBOT_ARM_EVENT_HOME_FAILED = 8
} RobotArmProtocolEventType_t;

typedef enum
{
    ROBOT_PROTOCOL_OK = 0,
    ROBOT_PROTOCOL_ERR_BAD_AXIS = 0x80,
    ROBOT_PROTOCOL_ERR_BAD_CMD = 0x81,
    ROBOT_PROTOCOL_ERR_BAD_PAGE = 0x82
} RobotArmProtocolError_t;

typedef uint8_t (*RobotArmProtocolTx_t)(const uint8_t *frame, uint8_t length);

typedef struct
{
    uint32_t tx_queued_count;
    uint32_t tx_consumed_count;
    uint32_t tx_overflow_count;
    uint32_t ack_overflow_count;
    uint32_t status_overflow_count;
    /* STATUS 请求、构帧、提交及物理发送完成计数，仅供现场调试器读取。 */
    uint32_t status_request_count;
    uint32_t status_build_count;
    uint32_t status_tx_submit_count;
    uint32_t status_tx_done_count;
    uint32_t status_tx_reject_count;
    uint32_t event_produced_count;
    uint32_t event_queued_count;
    uint32_t event_consumed_count;
    uint32_t event_retry_count;
} RobotArmProtocolStats_t;

/** 初始化 RobotArm V2 命令层及发送回调。 */
void RobotArmProtocol_Init(RobotArmProtocolTx_t tx_callback);
/** 分发一帧已经通过 CRC 校验的 RobotArm V2 请求。 */
void RobotArmProtocol_HandleFrame(const ProtocolV2Frame_t *request);
/** 轮询异步 RobotArm 操作并发送真正的完成或失败事件。 */
void RobotArmProtocol_Task(void);
/** 查询发送队列是否至少能可靠保存一个命令可能产生的 ACK 和 EVENT。 */
uint8_t RobotArmProtocol_CanAcceptRequest(void);
/** 获取发送队列和异步事件生命周期统计。 */
void RobotArmProtocol_GetStats(RobotArmProtocolStats_t *stats);

#endif
