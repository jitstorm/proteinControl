
#include <USART.h>
#include "protocol.h"
#include "DELAY.h"
#include "time.h"
#include "RingBuffer.h"

#define FRAME_HEADER 0xA5
#define FRAME_END 0x55
#define RX_BUFFER_SIZE 10
#define USART1_FRAME_QUEUE_SIZE 4
#define RS485_TX_WAIT_TIMEOUT_MS 20u
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
/* USART1 的 PA8 与数据寄存器必须由同一完整帧独占，防止重入发送切换 DE。 */
static volatile uint8_t usart1_tx_busy = 0u;
volatile uint32_t rs485_tx_call_count = 0;
volatile uint32_t rs485_tx_success_count = 0;
volatile uint32_t rs485_tx_txe_timeout_count = 0;
volatile uint32_t rs485_tx_tc_timeout_count = 0;
volatile uint32_t rs485_tx_bytes_count = 0;
/* 记录单次轮询实际等待的最长时间，覆盖发送前 TC、逐字节 TXE 和最终 TC。 */
volatile uint32_t rs485_tx_max_txe_wait_ms = 0u;
volatile uint32_t rs485_tx_max_tc_wait_ms = 0u;
/* 最近一次 USART1 发送尝试的整帧长度、实际写入 DR 的长度和未开始发送失败原因。 */
volatile uint16_t rs485_tx_last_expected_len = 0u;
volatile uint16_t rs485_tx_last_sent_len = 0u;
volatile uint8_t rs485_tx_last_failure_reason = 0u;
/* 失败快照只在未开始发送的失败路径更新，后续成功或 0x00 上报不得覆盖。 */
volatile uint32_t rs485_tx_failure_count = 0u;
volatile uint16_t rs485_tx_last_error_expected_len = 0u;
volatile uint16_t rs485_tx_last_error_sent_len = 0u;
volatile uint8_t rs485_tx_last_error_reason = 0u;
volatile uint8_t rs485_tx_last_error_cmd = 0xFFu;
static volatile uint8_t usart1_tx_diagnostic_cmd = 0xFFu;

#define RS485_TX_FAILURE_NONE 0u
#define RS485_TX_FAILURE_INVALID_ARGUMENT 1u
#define RS485_TX_FAILURE_BUSY 2u
#define RS485_TX_FAILURE_PREVIOUS_TC_TIMEOUT 3u
#define RS485_TX_FAILURE_FIRST_TXE_TIMEOUT 4u

/**
 * 原子保存 USART1 整帧尚未开始时的失败现场。
 *
 * 发送层已经保证 sent_len 大于零后不会返回；因此该记录用于区分 BUSY 等
 * 整帧未开始的失败，且不得被之后成功发送的周期温度帧清除。
 *
 * @param expected_len 本次请求发送的完整帧长度。
 * @param sent_len 失败前已写入 UART 数据寄存器的字节数。
 * @param reason 本次未开始发送的失败原因。
 */
static void USART1_RecordTxFailure(uint16_t expected_len,
                                   uint16_t sent_len,
                                   uint8_t reason)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    rs485_tx_failure_count++;
    rs485_tx_last_error_expected_len = expected_len;
    rs485_tx_last_error_sent_len = sent_len;
    rs485_tx_last_error_reason = reason;
    rs485_tx_last_error_cmd = usart1_tx_diagnostic_cmd;
    if (!primask)
    {
        __enable_irq();
    }
}

/**
 * 设置当前 USART1 帧的协议命令诊断上下文。
 *
 * 该上下文只在发送尚未开始而失败时复制到持久快照，用于确认例如 CMD=0x08
 * 是否因 tx_busy 而整帧未发送。
 *
 * @param cmd 即将发送的协议命令字。
 */
void USART1_SetTxDiagnosticCommand(uint8_t cmd)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    usart1_tx_diagnostic_cmd = cmd;
    if (!primask)
    {
        __enable_irq();
    }
}

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

static uint8_t USART1_TryAcquireTx(void)
{
    uint32_t primask;
    uint8_t acquired = 0u;

    /* 仅在读写 busy 标志时关中断，完整串口发送期间仍允许实时中断运行。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (!usart1_tx_busy)
    {
        usart1_tx_busy = 1u;
        acquired = 1u;
    }
    if (!primask)
    {
        __enable_irq();
    }
    return acquired;
}

static void USART1_ReleaseTx(void)
{
    uint32_t primask;

    /* 释放必须发生在 PA8 已恢复接收之后，禁止下一帧提前接管方向脚。 */
    primask = __get_PRIMASK();
    __disable_irq();
    usart1_tx_busy = 0u;
    if (!primask)
    {
        __enable_irq();
    }
}

/**
 * 保存 USART1 发送等待的历史最大值。
 *
 * 超过软件诊断阈值后仍会等待，以保证已写入首字节的协议帧绝不被截断；
 * 因此该值用于确认是否正是该保帧路径阻塞了主循环。
 *
 * @param is_txe 1 表示等待 TXE，0 表示等待 TC。
 * @param wait_ms 本次等待的实际毫秒数。
 */
static void USART1_RecordMaxWait(uint8_t is_txe, uint32_t wait_ms)
{
    if (is_txe)
    {
        if (wait_ms > rs485_tx_max_txe_wait_ms)
        {
            rs485_tx_max_txe_wait_ms = wait_ms;
        }
        return;
    }

    if (wait_ms > rs485_tx_max_tc_wait_ms)
    {
        rs485_tx_max_tc_wait_ms = wait_ms;
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
    USART_InitStructure.USART_BaudRate = 115200;
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

uint8_t USART_SendByte(USART_TypeDef *USARTx, uint8_t data)
{
    /* 独立单字节发送复用整帧入口，保证异常路径也会归还 485 总线。 */
    return USART_SendBuffer(USARTx, &data, 1u);
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
    uint16_t length = 0u;

    if (str == 0)
    {
        return;
    }
    while (str[length] != '\0')
    {
        length++;
    }
    /* 字符串也必须作为一个发送单元提交，避免逐字节切换 RS485 方向。 */
    (void)USART_SendBuffer(USARTx, (uint8_t *)str, length);
}
/**
 * 以轮询阻塞方式完成一个 UART 发送单元。
 *
 * USART1 是主机 RS485 协议 UART，PA8 为方向控制、PA9 为 TX。发送权仅覆盖一整帧；
 * 因此首字节写入 DR 后绝不能因软件超时退出，否则 PA8 切回接收会截断正在发送的协议帧。
 * TXE 只说明数据寄存器可继续装载，最终必须等待 TC=1 才表示最后一个停止位已离开 PA9。
 *
 * @param USARTx 目标 UART 外设。
 * @param buffer 连续发送数据，本函数同步完成前不会保存该地址。
 * @param length 必须连续发送的字节数。
 * @return 1 表示完整发送并确认 TC；0 表示首字节前未能开始本次发送。
 */
uint8_t USART_SendBuffer(USART_TypeDef *USARTx, uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    uint32_t tx_wait_start;
    uint8_t is_rs485;
    uint8_t is_usart1;
    uint8_t usart1_tx_acquired = 0u;
    uint8_t timeout_recorded;

    is_usart1 = (USARTx == USART1) ? 1u : 0u;
    if (is_usart1)
    {
        rs485_tx_call_count++;
        rs485_tx_last_expected_len = length;
        rs485_tx_last_sent_len = 0u;
        rs485_tx_last_failure_reason = RS485_TX_FAILURE_NONE;
    }

    if ((USARTx == 0) || (buffer == 0) || (length == 0u))
    {
        /* 参数失败时尚未取得发送权，不能触碰可能属于其他帧的 PA8。 */
        if (is_usart1)
        {
            rs485_tx_last_failure_reason = RS485_TX_FAILURE_INVALID_ARGUMENT;
            USART1_RecordTxFailure(length, 0u, RS485_TX_FAILURE_INVALID_ARGUMENT);
        }
        return 0u;
    }

    is_rs485 = ((USARTx == USART1) || (USARTx == USART3)) ? 1u : 0u;

    if (is_usart1)
    {
        /* 取得发送权失败时绝不写 DR、TXE、TC 或 PA8，当前整帧保持完整。 */
        if (!USART1_TryAcquireTx())
        {
            rs485_tx_last_failure_reason = RS485_TX_FAILURE_BUSY;
            USART1_RecordTxFailure(length, 0u, RS485_TX_FAILURE_BUSY);
            return 0u;
        }
        usart1_tx_acquired = 1u;
    }

    if (is_rs485)
    {
        /* 上一次发送必须已物理结束，才能由本次调用独占 DE。 */
        tx_wait_start = millis();
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
        {
            if ((millis() - tx_wait_start) >= RS485_TX_WAIT_TIMEOUT_MS)
            {
                if (is_usart1)
                {
                    rs485_tx_tc_timeout_count++;
                    Protocol_RecordUsart2TxTimeout(2u);
                    rs485_tx_last_failure_reason = RS485_TX_FAILURE_PREVIOUS_TC_TIMEOUT;
                    USART1_RecordTxFailure(length, 0u,
                                           RS485_TX_FAILURE_PREVIOUS_TC_TIMEOUT);
                    USART1_RecordMaxWait(0u, millis() - tx_wait_start);
                }
                goto tx_cleanup;
            }
        }
        if (is_usart1)
        {
            USART1_RecordMaxWait(0u, millis() - tx_wait_start);
        }

        /* 整帧独占 485 方向，禁止在帧内字节之间切回接收。 */
        RS485_SetTransmit(USARTx);
        Delay_us(20);
    }

    for (i = 0; i < length; i++)
    {
        tx_wait_start = millis();
        timeout_recorded = 0u;
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
        {
            if ((millis() - tx_wait_start) >= RS485_TX_WAIT_TIMEOUT_MS)
            {
                if (is_usart1 && !timeout_recorded)
                {
                    rs485_tx_txe_timeout_count++;
                    Protocol_RecordUsart2TxTimeout(1u);
                    timeout_recorded = 1u;
                }
                if (i == 0u)
                {
                    /* 首字节尚未写入，可明确报告本帧未开始。 */
                    if (is_usart1)
                    {
                        rs485_tx_last_failure_reason = RS485_TX_FAILURE_FIRST_TXE_TIMEOUT;
                        USART1_RecordTxFailure(length, 0u,
                                               RS485_TX_FAILURE_FIRST_TXE_TIMEOUT);
                        USART1_RecordMaxWait(1u, millis() - tx_wait_start);
                    }
                    goto tx_cleanup;
                }
                /* 已写入前序字节时只能记录卡顿并继续等 TXE，禁止截断当前帧。 */
            }
        }
        if (is_usart1)
        {
            USART1_RecordMaxWait(1u, millis() - tx_wait_start);
        }
        USART_SendData(USARTx, buffer[i]);
        if (is_usart1)
        {
            rs485_tx_bytes_count++;
            rs485_tx_last_sent_len = (uint16_t)(i + 1u);
        }
    }

    /* 只有最后一个字节的移位寄存器清空后，才允许释放 485 发送使能。 */
    tx_wait_start = millis();
    timeout_recorded = 0u;
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    {
        if ((millis() - tx_wait_start) >= RS485_TX_WAIT_TIMEOUT_MS)
        {
            if (is_usart1 && !timeout_recorded)
            {
                rs485_tx_tc_timeout_count++;
                Protocol_RecordUsart2TxTimeout(2u);
                timeout_recorded = 1u;
            }
            /* 所有字节已写入 DR，必须等待 TC，不能提前切回 PA8 接收而截断帧尾。 */
        }
    }
    if (is_usart1)
    {
        USART1_RecordMaxWait(0u, millis() - tx_wait_start);
    }

    if (is_rs485)
    {
        Delay_us(20);
        RS485_SetReceive(USARTx);
    }
    if (is_usart1)
    {
        /* 只有最终 TC 完成且 PA8 已恢复接收，才计为成功。 */
        rs485_tx_success_count++;
    }
    if (usart1_tx_acquired)
    {
        USART1_ReleaseTx();
    }
    return 1u;

tx_cleanup:
    /* TXE/TC 超时等所有发送失败路径都必须将收发器归还到接收态。 */
    if (is_rs485)
    {
        RS485_SetReceive(USARTx);
    }
    if (usart1_tx_acquired)
    {
        /* 所有 TXE/TC 超时路径同样先归还 PA8，再允许下一帧接管。 */
        USART1_ReleaseTx();
    }
    /* 此处仅能由首字节前的失败路径进入，因此不会形成半帧。 */
    return 0u;
}
// 重定向fputc函数
// int fputc(int ch, FILE *f)
// {
//     USART_SendByte(USART1, (uint8_t)ch);
//     return ch;
// }
