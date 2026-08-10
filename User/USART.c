
#include <USART.h>
#include "protocol.h"
#include "DELAY.h"
#include "time.h"
#include "RingBuffer.h"

#define FRAME_HEADER 0xA5
#define FRAME_END 0x55
#define RX_BUFFER_SIZE 10
#define USART1_FRAME_QUEUE_SIZE 4
#define USART2_TX_WAIT_TIMEOUT_MS 5u
#define USART_PROTOCOL_HEADER 0xAAu
#define USART_PROTOCOL_TAIL 0x55u
#define USART_PROTOCOL_FRAME_SIZE 10u

typedef struct
{
    uint8_t frame[USART_PROTOCOL_FRAME_SIZE];
    uint8_t index;
} USART_FrameParser;

static uint8_t usart1_rx_storage[USART1_RX_BUFFER_SIZE];
static uint8_t usart3_rx_storage[USART3_RX_BUFFER_SIZE];
static RingBuffer usart1_rx_ring;
static RingBuffer usart3_rx_ring;
static USART_FrameParser usart1_frame_parser;
static USART_FrameParser usart3_frame_parser;
USART_RxStats usart1_rx_stats;
USART_RxStats usart3_rx_stats;

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t receiving = 0;
volatile uint8_t frame_ready = 0;
static volatile uint8_t usart1_frame_queue[USART1_FRAME_QUEUE_SIZE][RX_BUFFER_SIZE];
static volatile uint8_t usart1_frame_head = 0;
static volatile uint8_t usart1_frame_tail = 0;
static volatile uint8_t usart1_frame_count = 0;
#define USART1_TEMP_FRAME_SIZE 5
#define USART1_TEMP_HEADER 0xA9
#define USART1_TEMP_TAIL 0x55
#define USART1_TEMP_TIMEOUT_MS 4000
#define USART1_TEMP_RX_GAP_MS 50
static volatile uint8_t usart1_temp_index = 0;
static volatile uint8_t usart1_temp_receiving = 0;
static volatile uint8_t usart1_temp_buffer[USART1_TEMP_FRAME_SIZE];
static volatile uint16_t usart1_remote_temperature = 0;
static volatile uint32_t usart1_remote_temperature_tick = 0;
static volatile uint32_t usart1_temp_rx_tick = 0;

#define RS485_EN1_PORT GPIOA
#define RS485_EN1_PIN GPIO_Pin_8
#define RS485_EN3_PORT GPIOB
#define RS485_EN3_PIN GPIO_Pin_15

static void RS485_SetReceive(USART_TypeDef *USARTx)
{
    if (USARTx == USART1)
    {
        GPIO_ResetBits(RS485_EN1_PORT, RS485_EN1_PIN);
    }
    else if (USARTx == USART3)
    {
        GPIO_ResetBits(RS485_EN3_PORT, RS485_EN3_PIN);
    }
}

static void RS485_SetTransmit(USART_TypeDef *USARTx)
{
    if (USARTx == USART1)
    {
        GPIO_SetBits(RS485_EN1_PORT, RS485_EN1_PIN);
    }
    else if (USARTx == USART3)
    {
        GPIO_SetBits(RS485_EN3_PORT, RS485_EN3_PIN);
    }
}

static void USART_ResetRxStats(USART_RxStats *stats)
{
    stats->rx_count = 0u;
    stats->rx_overflow_count = 0u;
    stats->frame_ok_count = 0u;
    stats->frame_error_count = 0u;
}

static uint8_t USART_IsProtocolFrameValid(const uint8_t *frame)
{
    uint8_t checksum;
    uint8_t index;

    if (frame[0] != USART_PROTOCOL_HEADER ||
        frame[USART_PROTOCOL_FRAME_SIZE - 2u] != USART_PROTOCOL_TAIL)
    {
        return 0u;
    }

    checksum = 0u;
    /* 固定协议把帧尾放在第 9 字节，校验和为最后一个字节。 */
    for (index = 0u; index < (USART_PROTOCOL_FRAME_SIZE - 2u); index++)
    {
        checksum += frame[index];
    }

    return (checksum == frame[USART_PROTOCOL_FRAME_SIZE - 1u]) ? 1u : 0u;
}

static void USART_ProcessProtocolByte(USART_FrameParser *parser,
                                      uint8_t byte,
                                      USART_RxStats *stats)
{
    uint8_t source_index;
    uint8_t destination_index;

    if (parser->index == 0u)
    {
        if (byte == USART_PROTOCOL_HEADER)
        {
            parser->frame[0] = byte;
            parser->index = 1u;
        }
        return;
    }

    parser->frame[parser->index] = byte;
    parser->index++;
    if (parser->index < USART_PROTOCOL_FRAME_SIZE)
    {
        return;
    }

    if (USART_IsProtocolFrameValid(parser->frame))
    {
        /* 协议解析只在主循环执行，绝不占用串口中断时间。 */
        parse_frame(parser->frame);
        stats->frame_ok_count++;
        parser->index = 0u;
        return;
    }

    stats->frame_error_count++;

    /* 失败后保留帧内后续可能出现的 AA，以尽快与连续数据重新对齐。 */
    parser->index = 0u;
    for (source_index = 1u; source_index < USART_PROTOCOL_FRAME_SIZE; source_index++)
    {
        if (parser->frame[source_index] == USART_PROTOCOL_HEADER)
        {
            destination_index = 0u;
            while (source_index < USART_PROTOCOL_FRAME_SIZE)
            {
                parser->frame[destination_index] = parser->frame[source_index];
                destination_index++;
                source_index++;
            }
            parser->index = destination_index;
            break;
        }
    }
}

static void USART1_ProcessTemperatureByte(uint8_t byte)
{
    uint8_t checksum;
    uint16_t temp_value;
    uint32_t now_tick;

    now_tick = millis();
    if (usart1_temp_receiving &&
        (now_tick - usart1_temp_rx_tick) > USART1_TEMP_RX_GAP_MS)
    {
        usart1_temp_index = 0u;
        usart1_temp_receiving = 0u;
    }

    if (!usart1_temp_receiving)
    {
        if (byte == USART1_TEMP_HEADER)
        {
            usart1_temp_buffer[0] = byte;
            usart1_temp_index = 1u;
            usart1_temp_receiving = 1u;
            usart1_temp_rx_tick = now_tick;
        }
        return;
    }

    if (byte == USART1_TEMP_HEADER)
    {
        usart1_temp_buffer[0] = byte;
        usart1_temp_index = 1u;
        usart1_temp_rx_tick = now_tick;
        return;
    }

    if (usart1_temp_index < USART1_TEMP_FRAME_SIZE)
    {
        usart1_temp_buffer[usart1_temp_index] = byte;
        usart1_temp_index++;
        usart1_temp_rx_tick = now_tick;
    }

    if (usart1_temp_index < USART1_TEMP_FRAME_SIZE)
    {
        return;
    }

    usart1_temp_receiving = 0u;
    checksum = usart1_temp_buffer[0] + usart1_temp_buffer[1] +
               usart1_temp_buffer[2] + usart1_temp_buffer[3];
    if (usart1_temp_buffer[3] == USART1_TEMP_TAIL &&
        checksum == usart1_temp_buffer[4])
    {
        temp_value = ((uint16_t)usart1_temp_buffer[1] << 8) |
                     usart1_temp_buffer[2];
        usart1_remote_temperature = temp_value;
        usart1_remote_temperature_tick = now_tick;
    }
    usart1_temp_index = 0u;
}

static void USART_RxIrqHandler(USART_TypeDef *USARTx,
                               RingBuffer *ring_buffer,
                               USART_RxStats *stats)
{
    uint16_t status;
    uint8_t byte;

    status = USARTx->SR;
    if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        /* STM32F1 通过先读 SR 再读 DR 清除错误标志，防止接收中断被持续占用。 */
        byte = (uint8_t)USARTx->DR;
        (void)byte;
        return;
    }

    if (status & USART_SR_RXNE)
    {
        byte = (uint8_t)USARTx->DR;
        stats->rx_count++;
        if (!RingBuffer_WriteByte(ring_buffer, byte))
        {
            stats->rx_overflow_count++;
        }
    }
}


void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 两个串口各自拥有缓冲区，避免高速接收时相互抢占数据。 */
    RingBuffer_Init(&usart1_rx_ring, usart1_rx_storage, USART1_RX_BUFFER_SIZE);
    RingBuffer_Init(&usart3_rx_ring, usart3_rx_storage, USART3_RX_BUFFER_SIZE);
    usart1_frame_parser.index = 0u;
    usart3_frame_parser.index = 0u;
    USART_ResetRxStats(&usart1_rx_stats);
    USART_ResetRxStats(&usart3_rx_stats);

    /* 串口 3 使用部分重映射至 PC10/PC11，需额外开启 GPIOC、AFIO 和 USART3 时钟。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // 1. 开启时�?
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1 �?APB2
    // 2. GPIO 配置
    // USART1 TX (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART1 RX (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* 两路 485 默认进入接收状态，避免上电后误驱动总线。 */
    GPIO_InitStructure.GPIO_Pin = RS485_EN1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RS485_EN1_PORT, &GPIO_InitStructure);
    RS485_SetReceive(USART1);

    GPIO_InitStructure.GPIO_Pin = RS485_EN3_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    RS485_SetReceive(USART3);

    /*
     * PB10/PB11 已被步进 DMA 占用，因此串口 3 部分重映射到 PC10/PC11。
     * PC10 为 TX，PC11 为 RX，PB15 仅用于控制 485 收发方向。
     */
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 3. USART 配置（通用�?
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // 4. 初始�?USART1
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
    USART_ITConfig(USART1, USART_IT_PE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    /* 串口 3 使用 PB15 控制 485 收发方向。 */
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_ERR, ENABLE);
    USART_ITConfig(USART3, USART_IT_PE, ENABLE);
    USART_Cmd(USART3, ENABLE);
    // // 6. NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 串口 3 接收中断用于处理 485 总线返回数据。 */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

uint8_t CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}


/**
 * @brief 将串口 3 接收字节写入环形缓冲区。
 */
void USART3_IRQHandler(void)
{
    USART_RxIrqHandler(USART3, &usart3_rx_ring, &usart3_rx_stats);
}

static void USART1_QueueFrame(uint8_t *frame)
{
    uint8_t i;

    if (usart1_frame_count >= USART1_FRAME_QUEUE_SIZE)
    {
        /* ???????????????????????? */
        dbg_frame_queue_overflow++;
        return;
    }

    for (i = 0; i < RX_BUFFER_SIZE; i++)
    {
        usart1_frame_queue[usart1_frame_tail][i] = frame[i];
    }
    usart1_frame_tail++;
    if (usart1_frame_tail >= USART1_FRAME_QUEUE_SIZE)
    {
        usart1_frame_tail = 0;
    }
    usart1_frame_count++;
    frame_ready = 1;
}

static void USART1_ProcessRxBytes(void)
{
    uint8_t byte;

    /* 中断生产、主循环消费，协议解析不在中断中执行。 */
    while (RingBuffer_ReadByte(&usart1_rx_ring, &byte))
    {
        if (!receiving)
        {
            if (byte != 0xAA)
            {
                dbg_parser_bad_head++;
                continue;
            }

            rx_index = 0;
            rx_buffer[rx_index++] = byte;
            receiving = 1;
            continue;
        }

        rx_buffer[rx_index++] = byte;
        if (rx_index >= RX_BUFFER_SIZE)
        {
            /* ???????? 10 ???????????? parse_frame ??? */
            USART1_QueueFrame(rx_buffer);
            rx_index = 0;
            receiving = 0;
        }
    }
}

uint8_t USART1_TakeFrame(uint8_t *frame)
{
    uint8_t i;

    USART1_ProcessRxBytes();

    __disable_irq();
    if (usart1_frame_count == 0)
    {
        frame_ready = 0;
        __enable_irq();
        return 0;
    }

    for (i = 0; i < RX_BUFFER_SIZE; i++)
    {
        frame[i] = usart1_frame_queue[usart1_frame_head][i];
    }
    usart1_frame_head++;
    if (usart1_frame_head >= USART1_FRAME_QUEUE_SIZE)
    {
        usart1_frame_head = 0;
    }
    usart1_frame_count--;
    frame_ready = (usart1_frame_count > 0) ? 1 : 0;
    __enable_irq();

    return 1;
}

void USART2_IRQHandler(void)
{
    uint16_t status;

    status = USART2->SR;

    if (status & USART_SR_RXNE)
    {
        dbg_usart_rx_bytes++;
    }
    if (status & USART_SR_ORE)
    {
        dbg_usart_ore++;
    }
    if (status & USART_SR_FE)
    {
        dbg_usart_fe++;
    }
    if (status & USART_SR_NE)
    {
        dbg_usart_ne++;
    }
    if (status & USART_SR_PE)
    {
        dbg_usart_pe++;
    }

    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        /* ?? USART ??????? SR ?? DR ?????? */
        (void)USART2->SR;
        (void)USART2->DR;
        return;
    }

    if (status & USART_SR_RXNE)
    {
        /* USART2 已不参与主机通讯，仅清空数据寄存器。 */
        (void)USART2->DR;
    }
}
/**
 * @brief 将串口 1 接收字节写入环形缓冲区。
 */
void USART1_IRQHandler(void)
{
    uint16_t status;
    uint8_t byte;

    status = USART1->SR;

    if (status & USART_SR_RXNE)
    {
        dbg_usart_rx_bytes++;
    }
    if (status & USART_SR_ORE)
    {
        dbg_usart_ore++;
    }
    if (status & USART_SR_FE)
    {
        dbg_usart_fe++;
    }
    if (status & USART_SR_NE)
    {
        dbg_usart_ne++;
    }
    if (status & USART_SR_PE)
    {
        dbg_usart_pe++;
    }

    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        /* 按 STM32F1 规定读取 SR 与 DR，清除串口错误状态。 */
        (void)USART1->SR;
        (void)USART1->DR;
        return;
    }

    if (status & USART_SR_RXNE)
    {
        byte = (uint8_t)USART1->DR;
        /* USART1 的485数据进入专用环形缓冲区，由主循环组帧。 */
        if (!RingBuffer_WriteByte(&usart1_rx_ring, byte))
        {
            usart1_rx_stats.rx_overflow_count++;
            dbg_frame_queue_overflow++;
        }
    }
}

/**
 * @brief 轮询处理串口 1 环形缓冲区中的协议与温度数据。
 */
void USART1_Process(void)
{
    uint8_t byte;

    while (RingBuffer_ReadByte(&usart1_rx_ring, &byte))
    {
        if (usart1_frame_parser.index != 0u || byte == USART_PROTOCOL_HEADER)
        {
            USART_ProcessProtocolByte(&usart1_frame_parser, byte, &usart1_rx_stats);
        }
        else
        {
            /* 保留原有 A9 温度帧，但将解析移出中断。 */
            USART1_ProcessTemperatureByte(byte);
        }
    }
}

/**
 * @brief 轮询处理串口 3 环形缓冲区中的协议数据。
 */
void USART3_Process(void)
{
    uint8_t byte;

    while (RingBuffer_ReadByte(&usart3_rx_ring, &byte))
    {
        USART_ProcessProtocolByte(&usart3_frame_parser, byte, &usart3_rx_stats);
    }
}

uint16_t USART1_GetRemoteTemperature(void)
{
    uint16_t temp_value;
    uint32_t temp_tick;

    __disable_irq();
    temp_value = usart1_remote_temperature;
    temp_tick = usart1_remote_temperature_tick;
    __enable_irq();

    if (temp_tick == 0 || (millis() - temp_tick) > USART1_TEMP_TIMEOUT_MS)
    {
        __disable_irq();
        usart1_remote_temperature = 0;
        __enable_irq();
        return 0;
    }

    return temp_value;
}

void USART_SendByte(USART_TypeDef *USARTx, uint8_t data)
{
    uint32_t tx_wait_start;

    if (USARTx == USART1 || USARTx == USART3)
    {
        RS485_SetTransmit(USARTx);
        Delay_us(20);
    }

    // while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
    //     ;
    USART_SendData(USARTx, data);
    tx_wait_start = millis();
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
    {
        /* timeout 现场由协议层统一原子记录，正常发送时序保持不变。 */
        if (USARTx == USART1 &&
            (millis() - tx_wait_start) >= USART2_TX_WAIT_TIMEOUT_MS)
        {
            Protocol_RecordUsart2TxTimeout(1u);
            return;
        }
    }

    if (USARTx == USART1 || USARTx == USART3)
    {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
            ;
        Delay_us(20);
        RS485_SetReceive(USARTx);
    }
}
/* Receive one byte from USART1. */
unsigned char UART1GetByte(unsigned char *GetData)
{
    return RingBuffer_ReadByte(&usart1_rx_ring, GetData);
}
/* Echo bytes received on USART1. */
void UART1Test(void)
{
    unsigned char i = 0;

    while (1)
    {
        while (UART1GetByte(&i))
        {
            USART_SendByte(USART1, i);
        }
    }
}
void USART_SendString(USART_TypeDef *USARTx, char *str)
{
    while (*str)
    {
        USART_SendByte(USARTx, *str++);
    }
}
void USART_SendBuffer(USART_TypeDef *USARTx, uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        USART_SendByte(USARTx, buffer[i]);
    }
}
// 重定向fputc函数
// int fputc(int ch, FILE *f)
// {
//     USART_SendByte(USART1, (uint8_t)ch);
//     return ch;
// }
