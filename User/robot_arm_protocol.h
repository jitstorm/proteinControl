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
    /* 已生成并保存到 RAM 的异步 0x71 终态结果数量。 */
    uint32_t event_produced_count;
    /* 已成为 pending 结果、等待 Android 后续查询的终态数量。 */
    uint32_t event_queued_count;
    /* 保留的历史统计字段；本版本不主动发送 0x71，因此始终为 0。 */
    uint32_t event_consumed_count;
    /* 保留的历史统计字段；本版本不再进行 0x71 发送重试，因此始终为 0。 */
    uint32_t event_retry_count;
} RobotArmProtocolStats_t;

/**
 * 初始化 RobotArm V2 命令层及从机回复发送回调。
 *
 * ACK 和 STATUS 只在 Android 请求后回复；机械臂异步终态以 pending 0x71 结果
 * 保存在 MCU RAM，避免 MCU 在两线制半双工 RS485 上主动发送产生总线冲突。
 *
 * @param tx_callback Android 请求对应回复使用的底层阻塞发送回调。
 */
void RobotArmProtocol_Init(RobotArmProtocolTx_t tx_callback);
/**
 * 分发一帧已经通过 CRC 校验的 RobotArm V2 请求。
 *
 * 对 0x38 的 page0～page3 只发送对应查询的 0x72；异步动作终态仍只保存于 RAM，
 * 不会在没有 Android 请求时主动发送 0x71。动作请求的 ACK 仅表示接受结果。
 *
 * @param request 已被 V2 解析器完成 CRC 校验的固定帧请求。
 */
void RobotArmProtocol_HandleFrame(const ProtocolV2Frame_t *request);
/**
 * 轮询异步 RobotArm 操作并保存最终 0x71 结果，不主动发送串口。
 *
 * 当动作正常完成、STOP、限位或故障结束时，结果保留原始 CMD/SEQ，供后续
 * Android 主动查询机制读取；该版本尚未增加查询命令。
 */
void RobotArmProtocol_Task(void);
/**
 * 查询发送队列是否至少能可靠保存一个命令可能产生的 ACK。
 *
 * pending 终态保存在独立 RAM 缓存，不占 UART 发送队列；返回 0 时调用方应保留
 * 当前接收帧到后续主循环，避免 ACK 或 STATUS 回复被覆盖。
 *
 * @return 1 表示可接收下一帧 V2 请求；0 表示仍有待回复数据，暂不可接收。
 */
uint8_t RobotArmProtocol_CanAcceptRequest(void);
/** 获取发送队列和异步事件生命周期统计。 */
void RobotArmProtocol_GetStats(RobotArmProtocolStats_t *stats);

#endif
