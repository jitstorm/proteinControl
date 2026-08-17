#ifndef __PROTOCOL_V2_H
#define __PROTOCOL_V2_H

#include <stdint.h>

#define PROTOCOL_V1_FRAME_SIZE 10u
#define PROTOCOL_V2_FRAME_SIZE 24u
#define PROTOCOL_V2_DATA_SIZE 16u
#define PROTOCOL_V2_HEAD 0xAAu
#define PROTOCOL_V2_MARK 0xFEu
#define PROTOCOL_V2_TAIL 0x55u

typedef struct
{
    uint8_t cmd;
    uint16_t seq;
    uint8_t data[PROTOCOL_V2_DATA_SIZE];
} ProtocolV2Frame_t;

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
/** 将 V2 逻辑帧编码为固定 24B 线格式。 */
void ProtocolV2_Encode(const ProtocolV2Frame_t *frame, uint8_t *raw_frame);
/** 校验并解码固定 24B 线格式。 */
uint8_t ProtocolV2_Decode(const uint8_t *raw_frame, ProtocolV2Frame_t *frame);
/** 按 CCITT-FALSE 参数计算指定字节范围的 CRC16。 */
uint16_t ProtocolV2_CalculateCrc(const uint8_t *data, uint16_t length);
/** 从小端字节流读取 uint16。 */
uint16_t ProtocolV2_ReadU16LE(const uint8_t *data);
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
