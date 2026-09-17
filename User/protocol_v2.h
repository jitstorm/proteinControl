#ifndef __PROTOCOL_V2_H
#define __PROTOCOL_V2_H

#include <stdint.h>

#define PROTOCOL_V1_FRAME_SIZE 10u
#define PROTOCOL_V2_FRAME_SIZE 24u
#define PROTOCOL_V2_DATA_SIZE 16u
#define PROTOCOL_V2_PHASE_BATCH_CMD 0x39u
#define PROTOCOL_V2_PHASE_BATCH_MAX_PHASES 16u
#define PROTOCOL_V2_PHASE_SIZE 16u
#define PROTOCOL_V2_PHASE_BATCH_HEADER_SIZE 4u
#define PROTOCOL_V2_PHASE_BATCH_MAX_PAYLOAD \
    (PROTOCOL_V2_PHASE_BATCH_HEADER_SIZE + \
     PROTOCOL_V2_PHASE_BATCH_MAX_PHASES * PROTOCOL_V2_PHASE_SIZE)
#define PROTOCOL_V2_PHASE_BATCH_MAX_FRAME_SIZE \
    (PROTOCOL_V2_PHASE_BATCH_MAX_PAYLOAD + 10u)
#define PROTOCOL_V2_HEAD 0xAAu
#define PROTOCOL_V2_MARK 0xFEu
#define PROTOCOL_V2_TAIL 0x55u

typedef struct
{
    uint8_t cmd;
    uint16_t seq;
    uint8_t data[PROTOCOL_V2_DATA_SIZE];
} ProtocolV2Frame_t;

/**
 * 0x39 批量相位请求的已校验原始载荷。
 *
 * 旧 V2 保持固定 24B；仅 0x39 使用 LEN 指定的变长载荷。payload 仅在 CRC、尾字节
 * 和长度均通过后才进入该结构，调用方仍须继续校验批头和每条相位字段。
 */
typedef struct
{
    uint16_t seq;
    uint16_t length;
    uint8_t payload[PROTOCOL_V2_PHASE_BATCH_MAX_PAYLOAD];
} ProtocolV2PhaseBatchFrame_t;

typedef struct
{
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t frame_error_count;
    uint32_t queue_overflow_count;
    uint32_t v1_queue_overflow_count;
    uint32_t v2_queue_overflow_count;
} ProtocolV2Stats_t;

/** 初始化固定 24B 协议解析器及 V1/V2 帧队列。 */
void ProtocolV2_Init(void);
/** 向非阻塞流解析器输入一个 USART 接收字节。 */
void ProtocolV2_InputByte(uint8_t byte);
/** 取出一帧保持原格式的 V1 固定 10B 帧。 */
uint8_t ProtocolV2_TakeV1Frame(uint8_t *frame);
/** 取出一帧已校验并解码的 V2 固定 24B 帧。 */
uint8_t ProtocolV2_TakeFrame(ProtocolV2Frame_t *frame);
/**
 * 取出一帧已完成长度、尾字节和 CRC 校验的 0x39 批量相位请求。
 *
 * 批量帧使用独立单槽队列，避免将变长数据误解释为旧 24B V2；调用方取得后仍需
 * 校验业务字段。队列为空或 frame 无效时不改变队列状态。
 *
 * @param frame 接收已校验 SEQ、LEN 与原始 payload 的输出结构。
 * @return 成功取出一帧返回 1；无帧或输出指针为空返回 0。
 */
uint8_t ProtocolV2_TakePhaseBatchFrame(ProtocolV2PhaseBatchFrame_t *frame);
/** 将 V2 逻辑帧编码为固定 24B 线格式。 */
void ProtocolV2_Encode(const ProtocolV2Frame_t *frame, uint8_t *raw_frame);
/** 校验并解码固定 24B 线格式。 */
uint8_t ProtocolV2_Decode(const uint8_t *raw_frame, ProtocolV2Frame_t *frame);
/** 按 CCITT-FALSE 参数计算指定字节范围的 CRC16。 */
uint16_t ProtocolV2_CalculateCrc(const uint8_t *data, uint16_t length);
/** 从小端字节流读取 uint16。 */
uint16_t ProtocolV2_ReadU16LE(const uint8_t *data);
/**
 * 从小端字节流读取有符号 int24。
 *
 * int24 仅用于压缩 24B V2 帧中的运动请求；机械臂内部位置、目标和步数运算仍
 * 使用 int32_t，避免三字节算术造成溢出或符号错误。
 *
 * @param data 指向低字节在前的三个协议字节。
 * @return 经 bit23 符号扩展后的 int32_t 值，范围为 -8388608～8388607。
 */
int32_t ProtocolV2_ReadI24LE(const uint8_t *data);
/** 从小端字节流读取 uint32。 */
uint32_t ProtocolV2_ReadU32LE(const uint8_t *data);
/** 从小端字节流读取 int32。 */
int32_t ProtocolV2_ReadI32LE(const uint8_t *data);
/** 将 uint16 写入小端字节流。 */
void ProtocolV2_WriteU16LE(uint8_t *data, uint16_t value);
/** 将 uint32 写入小端字节流。 */
void ProtocolV2_WriteU32LE(uint8_t *data, uint32_t value);
/** 将 int32 写入小端字节流。 */
void ProtocolV2_WriteI32LE(uint8_t *data, int32_t value);
/** 返回下一个 16 位 SEQ，65535 后自然回绕为 0。 */
uint16_t ProtocolV2_NextSeq(uint16_t seq);
/** 获取 V2 解析统计快照。 */
void ProtocolV2_GetStats(ProtocolV2Stats_t *stats);

#endif
