#include "protocol_v2.h"

#define PROTOCOL_V2_QUEUE_SIZE 4u

typedef struct
{
    uint8_t candidate[PROTOCOL_V2_FRAME_SIZE];
    uint8_t index;
    uint8_t expected_size;
} ProtocolV2Parser_t;

static ProtocolV2Parser_t s_parser;
static uint8_t s_v1_queue[PROTOCOL_V2_QUEUE_SIZE][PROTOCOL_V1_FRAME_SIZE];
static uint8_t s_v1_head;
static uint8_t s_v1_tail;
static uint8_t s_v1_count;
static ProtocolV2Frame_t s_v2_queue[PROTOCOL_V2_QUEUE_SIZE];
static uint8_t s_v2_head;
static uint8_t s_v2_tail;
static uint8_t s_v2_count;
static ProtocolV2Stats_t s_stats;

static void ProtocolV2_CopyBytes(uint8_t *destination,
                                 const uint8_t *source,
                                 uint8_t length)
{
    uint8_t index;
    for (index = 0u; index < length; index++)
    {
        destination[index] = source[index];
    }
}

static void ProtocolV2_ClearBytes(uint8_t *data, uint16_t length)
{
    uint16_t index;
    for (index = 0u; index < length; index++)
    {
        data[index] = 0u;
    }
}

static uint8_t ProtocolV2_IsV1Valid(const uint8_t *frame)
{
    uint8_t checksum = 0u;
    uint8_t index;
    if ((frame[0] != PROTOCOL_V2_HEAD) ||
        (frame[PROTOCOL_V1_FRAME_SIZE - 2u] != PROTOCOL_V2_TAIL))
    {
        return 0u;
    }
    for (index = 0u; index < (PROTOCOL_V1_FRAME_SIZE - 2u); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return (checksum == frame[PROTOCOL_V1_FRAME_SIZE - 1u]) ? 1u : 0u;
}

static void ProtocolV2_QueueV1(const uint8_t *frame)
{
    if (s_v1_count >= PROTOCOL_V2_QUEUE_SIZE)
    {
        s_stats.queue_overflow_count++;
        s_stats.v1_queue_overflow_count++;
        return;
    }
    ProtocolV2_CopyBytes(s_v1_queue[s_v1_tail], frame,
                         PROTOCOL_V1_FRAME_SIZE);
    s_v1_tail = (uint8_t)((s_v1_tail + 1u) % PROTOCOL_V2_QUEUE_SIZE);
    s_v1_count++;
}

static void ProtocolV2_QueueV2(const ProtocolV2Frame_t *frame)
{
    if (s_v2_count >= PROTOCOL_V2_QUEUE_SIZE)
    {
        s_stats.queue_overflow_count++;
        s_stats.v2_queue_overflow_count++;
        return;
    }
    s_v2_queue[s_v2_tail] = *frame;
    s_v2_tail = (uint8_t)((s_v2_tail + 1u) % PROTOCOL_V2_QUEUE_SIZE);
    s_v2_count++;
}

static void ProtocolV2_ResyncCandidate(void)
{
    int16_t source;
    uint8_t length;
    uint8_t index;

    /* 从最近的 AA 重新开始，避免一帧损坏后长期错位。 */
    for (source = (int16_t)s_parser.index - 1; source > 0; source--)
    {
        if (s_parser.candidate[source] == PROTOCOL_V2_HEAD)
        {
            length = (uint8_t)(s_parser.index - (uint8_t)source);
            for (index = 0u; index < length; index++)
            {
                s_parser.candidate[index] =
                    s_parser.candidate[(uint8_t)source + index];
            }
            s_parser.index = length;
            s_parser.expected_size = (length >= 2u &&
                                      s_parser.candidate[1] == PROTOCOL_V2_MARK) ?
                                         PROTOCOL_V2_FRAME_SIZE :
                                         ((length >= 2u) ? PROTOCOL_V1_FRAME_SIZE : 0u);
            return;
        }
    }
    s_parser.index = 0u;
    s_parser.expected_size = 0u;
}

static void ProtocolV2_ProcessCandidate(void)
{
    ProtocolV2Frame_t frame;
    if (s_parser.expected_size == PROTOCOL_V2_FRAME_SIZE)
    {
        if (s_parser.candidate[21] != PROTOCOL_V2_TAIL)
        {
            s_stats.frame_error_count++;
            ProtocolV2_ResyncCandidate();
            return;
        }
        if (!ProtocolV2_Decode(s_parser.candidate, &frame))
        {
            s_stats.crc_error_count++;
            ProtocolV2_ResyncCandidate();
            return;
        }
        ProtocolV2_QueueV2(&frame);
        s_stats.valid_frame_count++;
        s_parser.index = 0u;
        s_parser.expected_size = 0u;
        return;
    }
    else if (ProtocolV2_IsV1Valid(s_parser.candidate))
    {
        ProtocolV2_QueueV1(s_parser.candidate);
        s_parser.index = 0u;
        s_parser.expected_size = 0u;
        return;
    }
    ProtocolV2_ResyncCandidate();
}

/** 初始化固定 24B 协议解析器及 V1/V2 帧队列。 */
void ProtocolV2_Init(void)
{
    ProtocolV2_ClearBytes((uint8_t *)&s_parser, (uint16_t)sizeof(s_parser));
    s_v1_head = 0u;
    s_v1_tail = 0u;
    s_v1_count = 0u;
    s_v2_head = 0u;
    s_v2_tail = 0u;
    s_v2_count = 0u;
    ProtocolV2_ClearBytes((uint8_t *)&s_stats, (uint16_t)sizeof(s_stats));
}

/** 向非阻塞流解析器输入一个 USART 接收字节。 */
void ProtocolV2_InputByte(uint8_t byte)
{
    if (s_parser.index == 0u)
    {
        if (byte == PROTOCOL_V2_HEAD)
        {
            s_parser.candidate[0] = byte;
            s_parser.index = 1u;
        }
        return;
    }

    if (s_parser.index >= PROTOCOL_V2_FRAME_SIZE)
    {
        s_stats.frame_error_count++;
        s_parser.index = 0u;
        s_parser.expected_size = 0u;
        return;
    }
    s_parser.candidate[s_parser.index++] = byte;
    if (s_parser.index == 2u)
    {
        s_parser.expected_size = (byte == PROTOCOL_V2_MARK) ?
                                     PROTOCOL_V2_FRAME_SIZE :
                                     PROTOCOL_V1_FRAME_SIZE;
    }
    if ((s_parser.expected_size != 0u) &&
        (s_parser.index >= s_parser.expected_size))
    {
        ProtocolV2_ProcessCandidate();
    }
}

/** 取出一帧保持原格式的 V1 固定 10B 帧。 */
uint8_t ProtocolV2_TakeV1Frame(uint8_t *frame)
{
    if ((frame == 0) || (s_v1_count == 0u))
    {
        return 0u;
    }
    ProtocolV2_CopyBytes(frame, s_v1_queue[s_v1_head],
                         PROTOCOL_V1_FRAME_SIZE);
    s_v1_head = (uint8_t)((s_v1_head + 1u) % PROTOCOL_V2_QUEUE_SIZE);
    s_v1_count--;
    return 1u;
}

/** 取出一帧已校验并解码的 V2 固定 24B 帧。 */
uint8_t ProtocolV2_TakeFrame(ProtocolV2Frame_t *frame)
{
    if ((frame == 0) || (s_v2_count == 0u))
    {
        return 0u;
    }
    *frame = s_v2_queue[s_v2_head];
    s_v2_head = (uint8_t)((s_v2_head + 1u) % PROTOCOL_V2_QUEUE_SIZE);
    s_v2_count--;
    return 1u;
}

/** 按 CCITT-FALSE 参数计算指定字节范围的 CRC16。 */
uint16_t ProtocolV2_CalculateCrc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t index;
    uint8_t bit;
    for (index = 0u; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) ?
                      (uint16_t)((crc << 1) ^ 0x1021u) :
                      (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/** 将 V2 逻辑帧编码为固定 24B 线格式。 */
void ProtocolV2_Encode(const ProtocolV2Frame_t *frame, uint8_t *raw_frame)
{
    uint16_t crc;
    uint8_t index;
    raw_frame[0] = PROTOCOL_V2_HEAD;
    raw_frame[1] = PROTOCOL_V2_MARK;
    raw_frame[2] = frame->cmd;
    ProtocolV2_WriteU16LE(&raw_frame[3], frame->seq);
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++)
    {
        raw_frame[5u + index] = frame->data[index];
    }
    raw_frame[21] = PROTOCOL_V2_TAIL;
    crc = ProtocolV2_CalculateCrc(raw_frame, 22u);
    ProtocolV2_WriteU16LE(&raw_frame[22], crc);
}

/** 校验并解码固定 24B 线格式。 */
uint8_t ProtocolV2_Decode(const uint8_t *raw_frame, ProtocolV2Frame_t *frame)
{
    uint16_t expected_crc;
    uint16_t actual_crc;
    uint8_t index;
    if ((raw_frame == 0) || (frame == 0) ||
        (raw_frame[0] != PROTOCOL_V2_HEAD) ||
        (raw_frame[1] != PROTOCOL_V2_MARK) ||
        (raw_frame[21] != PROTOCOL_V2_TAIL))
    {
        return 0u;
    }
    expected_crc = ProtocolV2_ReadU16LE(&raw_frame[22]);
    actual_crc = ProtocolV2_CalculateCrc(raw_frame, 22u);
    if (expected_crc != actual_crc)
    {
        return 0u;
    }
    frame->cmd = raw_frame[2];
    frame->seq = ProtocolV2_ReadU16LE(&raw_frame[3]);
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++)
    {
        frame->data[index] = raw_frame[5u + index];
    }
    return 1u;
}

/** 从小端字节流读取 uint16。 */
uint16_t ProtocolV2_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/** 从小端字节流读取 uint32。 */
uint32_t ProtocolV2_ReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/** 从小端字节流读取 int32。 */
int32_t ProtocolV2_ReadI32LE(const uint8_t *data)
{
    return (int32_t)ProtocolV2_ReadU32LE(data);
}

/** 将 uint16 写入小端字节流。 */
void ProtocolV2_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

/** 将 uint32 写入小端字节流。 */
void ProtocolV2_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

/** 将 int32 写入小端字节流。 */
void ProtocolV2_WriteI32LE(uint8_t *data, int32_t value)
{
    ProtocolV2_WriteU32LE(data, (uint32_t)value);
}

/** 返回下一个 16 位 SEQ，65535 后自然回绕为 0。 */
uint16_t ProtocolV2_NextSeq(uint16_t seq)
{
    return (uint16_t)(seq + 1u);
}

/** 获取 V2 解析统计快照。 */
void ProtocolV2_GetStats(ProtocolV2Stats_t *stats)
{
    if (stats != 0)
    {
        *stats = s_stats;
    }
}
