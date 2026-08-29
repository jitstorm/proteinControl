#include <stdint.h>
#include "protocol_v2.h"

#define TEST_CHECK(condition) do { if (!(condition) && (s_failure == 0)) s_failure = __LINE__; } while (0)

static int s_failure;

static void TestFeed(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    for (index = 0u; index < length; index++)
    {
        ProtocolV2_InputByte(data[index]);
    }
}

static void TestBuildFrame(uint8_t *raw, uint8_t cmd, uint16_t seq)
{
    ProtocolV2Frame_t frame;
    uint8_t index;
    frame.cmd = cmd;
    frame.seq = seq;
    for (index = 0u; index < PROTOCOL_V2_DATA_SIZE; index++)
    {
        frame.data[index] = index;
    }
    ProtocolV2_Encode(&frame, raw);
}

static void TestBuildV1(uint8_t *raw, uint8_t cmd)
{
    uint8_t index;
    uint8_t sum = 0u;
    raw[0] = 0xAAu;
    raw[1] = cmd;
    for (index = 0u; index < 6u; index++)
    {
        raw[2u + index] = index;
    }
    raw[8] = 0x55u;
    for (index = 0u; index < 8u; index++)
    {
        sum = (uint8_t)(sum + raw[index]);
    }
    raw[9] = sum;
}

static void TestExpectV2(uint8_t cmd, uint16_t seq)
{
    ProtocolV2Frame_t frame;
    TEST_CHECK(ProtocolV2_TakeFrame(&frame) == 1u);
    TEST_CHECK(frame.cmd == cmd);
    TEST_CHECK(frame.seq == seq);
}

/** 运行固定 24B V2、失步恢复及 V1/V2 共存逻辑测试。 */
int main(void)
{
    uint8_t raw[PROTOCOL_V2_FRAME_SIZE];
    uint8_t raw2[PROTOCOL_V2_FRAME_SIZE];
    uint8_t v1[PROTOCOL_V1_FRAME_SIZE];
    uint8_t v1_out[PROTOCOL_V1_FRAME_SIZE];
    uint8_t stream[PROTOCOL_V2_FRAME_SIZE + 1u];
    uint8_t crc_vector[9] = {'1','2','3','4','5','6','7','8','9'};
    ProtocolV2Frame_t frame;
    ProtocolV2Stats_t stats;
    uint8_t index;

    TEST_CHECK(ProtocolV2_CalculateCrc(crc_vector, 9u) == 0x29B1u);
    TestBuildFrame(raw, 0x30u, 105u);

    /* 1：正常固定 24B 帧。 */
    ProtocolV2_Init();
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 105u);

    /* 2：连续两帧。 */
    ProtocolV2_Init();
    TestBuildFrame(raw2, 0x31u, 106u);
    TestFeed(raw, sizeof(raw));
    TestFeed(raw2, sizeof(raw2));
    TestExpectV2(0x30u, 105u);
    TestExpectV2(0x31u, 106u);

    /* 3、4：半帧分批和逐字节输入均不依赖阻塞等待。 */
    ProtocolV2_Init();
    TestFeed(raw, 12u);
    TEST_CHECK(ProtocolV2_TakeFrame(&frame) == 0u);
    TestFeed(&raw[12], 12u);
    TestExpectV2(0x30u, 105u);
    ProtocolV2_Init();
    for (index = 0u; index < sizeof(raw); index++)
    {
        ProtocolV2_InputByte(raw[index]);
    }
    TestExpectV2(0x30u, 105u);

    /* 5：CRC 错误不得产生有效命令。 */
    ProtocolV2_Init();
    raw[22] ^= 0x01u;
    TestFeed(raw, sizeof(raw));
    TEST_CHECK(ProtocolV2_TakeFrame(&frame) == 0u);
    ProtocolV2_GetStats(&stats);
    TEST_CHECK(stats.crc_error_count == 1u);
    raw[22] ^= 0x01u;

    /* 6、7、8：HEAD、MARK、TAIL 损坏后必须恢复到下一正常帧。 */
    ProtocolV2_Init();
    raw[0] = 0xABu;
    TestFeed(raw, sizeof(raw));
    raw[0] = PROTOCOL_V2_HEAD;
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 105u);
    ProtocolV2_Init();
    raw[1] = 0xFDu;
    TestFeed(raw, sizeof(raw));
    raw[1] = PROTOCOL_V2_MARK;
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 105u);
    ProtocolV2_Init();
    raw[21] = 0x54u;
    TestFeed(raw, sizeof(raw));
    raw[21] = PROTOCOL_V2_TAIL;
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 105u);

    /* 9：中间丢一个字节，利用下一帧 AA FE 重新同步。 */
    ProtocolV2_Init();
    for (index = 0u; index < sizeof(raw); index++)
    {
        if (index != 10u) ProtocolV2_InputByte(raw[index]);
    }
    TestFeed(raw2, sizeof(raw2));
    TestExpectV2(0x31u, 106u);

    /* 10：中间插入一个字节，错误 candidate 不得造成永久错位。 */
    ProtocolV2_Init();
    for (index = 0u; index < 10u; index++) stream[index] = raw[index];
    stream[10] = 0x77u;
    for (index = 10u; index < sizeof(raw); index++) stream[index + 1u] = raw[index];
    TestFeed(stream, sizeof(stream));
    TestFeed(raw2, sizeof(raw2));
    TestExpectV2(0x31u, 106u);

    /* 11：垃圾数据不会阻止后续正常 V2。 */
    ProtocolV2_Init();
    stream[0] = 0x12u;
    stream[1] = 0x55u;
    stream[2] = 0xFEu;
    TestFeed(stream, 3u);
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 105u);

    /* 12～14：V1/V2 任意连续顺序均保持各自固定格式。 */
    TestBuildV1(v1, 0x05u);
    ProtocolV2_Init();
    TestFeed(v1, sizeof(v1));
    TestFeed(raw, sizeof(raw));
    TEST_CHECK(ProtocolV2_TakeV1Frame(v1_out) == 1u);
    TestExpectV2(0x30u, 105u);
    ProtocolV2_Init();
    TestFeed(raw, sizeof(raw));
    TestFeed(v1, sizeof(v1));
    TestExpectV2(0x30u, 105u);
    TEST_CHECK(ProtocolV2_TakeV1Frame(v1_out) == 1u);
    ProtocolV2_Init();
    TestFeed(v1, sizeof(v1));
    TestFeed(raw, sizeof(raw));
    TestFeed(v1, sizeof(v1));
    TEST_CHECK(ProtocolV2_TakeV1Frame(v1_out) == 1u);
    TestExpectV2(0x30u, 105u);
    TEST_CHECK(ProtocolV2_TakeV1Frame(v1_out) == 1u);

    /* 15～17：DATA 内 AA、55、AA FE 仅作为普通数据。 */
    ProtocolV2_Decode(raw, &frame);
    frame.data[0] = 0xAAu;
    frame.data[1] = 0x55u;
    frame.data[2] = 0xAAu;
    frame.data[3] = 0xFEu;
    ProtocolV2_Encode(&frame, raw);
    ProtocolV2_Init();
    TestFeed(raw, sizeof(raw));
    TEST_CHECK(ProtocolV2_TakeFrame(&frame) == 1u);
    TEST_CHECK(frame.data[0] == 0xAAu && frame.data[1] == 0x55u);
    TEST_CHECK(frame.data[2] == 0xAAu && frame.data[3] == 0xFEu);

    /* 18～20：SEQ 0、65535 和自然回绕均保持 uint16 语义。 */
    TestBuildFrame(raw, 0x30u, 0u);
    ProtocolV2_Init();
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 0u);
    TestBuildFrame(raw, 0x30u, 65535u);
    ProtocolV2_Init();
    TestFeed(raw, sizeof(raw));
    TestExpectV2(0x30u, 65535u);
    TEST_CHECK(ProtocolV2_NextSeq(65535u) == 0u);

    /* 两个固定四帧接收队列满时保留前四帧，并分别累计 overflow。 */
    ProtocolV2_Init();
    for (index = 0u; index < 5u; index++) TestFeed(v1, sizeof(v1));
    ProtocolV2_GetStats(&stats);
    TEST_CHECK(stats.v1_queue_overflow_count == 1u);
    TEST_CHECK(stats.v2_queue_overflow_count == 0u);
    for (index = 0u; index < 4u; index++)
        TEST_CHECK(ProtocolV2_TakeV1Frame(v1_out) == 1u);
    TEST_CHECK(ProtocolV2_TakeV1Frame(v1_out) == 0u);

    ProtocolV2_Init();
    for (index = 0u; index < 5u; index++) TestFeed(raw, sizeof(raw));
    ProtocolV2_GetStats(&stats);
    TEST_CHECK(stats.v1_queue_overflow_count == 0u);
    TEST_CHECK(stats.v2_queue_overflow_count == 1u);
    for (index = 0u; index < 4u; index++)
        TEST_CHECK(ProtocolV2_TakeFrame(&frame) == 1u);
    TEST_CHECK(ProtocolV2_TakeFrame(&frame) == 0u);

    TEST_CHECK(ProtocolV2_ReadI32LE((uint8_t[]){0x78u,0x56u,0x34u,0x12u}) ==
               (int32_t)0x12345678);
    /* int24 只压缩运动请求线格式，必须正确扩展 bit23 的符号。 */
    TEST_CHECK(ProtocolV2_ReadI24LE((uint8_t[]){0x00u,0x00u,0x00u}) == 0);
    TEST_CHECK(ProtocolV2_ReadI24LE((uint8_t[]){0x01u,0x00u,0x00u}) == 1);
    TEST_CHECK(ProtocolV2_ReadI24LE((uint8_t[]){0xFFu,0xFFu,0xFFu}) == -1);
    TEST_CHECK(ProtocolV2_ReadI24LE((uint8_t[]){0xFFu,0xFFu,0x7Fu}) == 8388607);
    TEST_CHECK(ProtocolV2_ReadI24LE((uint8_t[]){0x00u,0x00u,0x80u}) == -8388608);
    return s_failure;
}
