#include "RingBuffer.h"

/**
 * @brief 初始化单生产者、单消费者环形缓冲区。
 * @param ring_buffer 环形缓冲区对象。
 * @param buffer 外部提供的存储区。
 * @param size 存储区的字节数，必须大于零。
 */
void RingBuffer_Init(RingBuffer *ring_buffer, uint8_t *buffer, uint16_t size)
{
    ring_buffer->buffer = buffer;
    ring_buffer->size = size;
    ring_buffer->head = 0u;
    ring_buffer->tail = 0u;
}

/**
 * @brief 向环形缓冲区写入一个字节。
 * @param ring_buffer 环形缓冲区对象。
 * @param data 待写入的数据。
 * @return 写入成功返回 1；缓冲区已满返回 0。
 */
uint8_t RingBuffer_WriteByte(RingBuffer *ring_buffer, uint8_t data)
{
    uint16_t head;

    if (ring_buffer->size == 0u)
    {
        return 0u;
    }

    head = ring_buffer->head;
    if ((uint16_t)(head - ring_buffer->tail) >= ring_buffer->size)
    {
        return 0u;
    }

    /* 先写入数据，再发布新的 head，确保中断生产者与主循环消费者可并发工作。 */
    ring_buffer->buffer[head % ring_buffer->size] = data;
    ring_buffer->head = (uint16_t)(head + 1u);
    return 1u;
}

/**
 * @brief 从环形缓冲区读取一个字节。
 * @param ring_buffer 环形缓冲区对象。
 * @param data 用于接收读取结果的地址。
 * @return 读取成功返回 1；缓冲区为空返回 0。
 */
uint8_t RingBuffer_ReadByte(RingBuffer *ring_buffer, uint8_t *data)
{
    uint16_t tail;

    if (ring_buffer->size == 0u)
    {
        return 0u;
    }

    tail = ring_buffer->tail;
    if (tail == ring_buffer->head)
    {
        return 0u;
    }

    /* 先读取旧 tail 指向的数据，再推进 tail，避免消费者读到尚未写完的数据。 */
    *data = ring_buffer->buffer[tail % ring_buffer->size];
    ring_buffer->tail = (uint16_t)(tail + 1u);
    return 1u;
}

/**
 * @brief 获取环形缓冲区中当前可读取的数据长度。
 * @param ring_buffer 环形缓冲区对象。
 * @return 当前可读取的字节数。
 */
uint16_t RingBuffer_GetLength(const RingBuffer *ring_buffer)
{
    return (uint16_t)(ring_buffer->head - ring_buffer->tail);
}

/**
 * @brief 清空环形缓冲区中的未读取数据。
 * @param ring_buffer 环形缓冲区对象。
 */
void RingBuffer_Clear(RingBuffer *ring_buffer)
{
    /* 仅让消费者追上当前生产位置，不回退 head，避免与接收中断竞争。 */
    ring_buffer->tail = ring_buffer->head;
}
