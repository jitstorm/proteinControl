#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>

typedef struct
{
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

/**
 * @brief 初始化单生产者、单消费者环形缓冲区。
 * @param ring_buffer 环形缓冲区对象。
 * @param buffer 外部提供的存储区。
 * @param size 存储区的字节数，必须大于零。
 */
void RingBuffer_Init(RingBuffer *ring_buffer, uint8_t *buffer, uint16_t size);

/**
 * @brief 向环形缓冲区写入一个字节。
 * @param ring_buffer 环形缓冲区对象。
 * @param data 待写入的数据。
 * @return 写入成功返回 1；缓冲区已满返回 0。
 */
uint8_t RingBuffer_WriteByte(RingBuffer *ring_buffer, uint8_t data);

/**
 * @brief 从环形缓冲区读取一个字节。
 * @param ring_buffer 环形缓冲区对象。
 * @param data 用于接收读取结果的地址。
 * @return 读取成功返回 1；缓冲区为空返回 0。
 */
uint8_t RingBuffer_ReadByte(RingBuffer *ring_buffer, uint8_t *data);

/**
 * @brief 获取环形缓冲区中当前可读取的数据长度。
 * @param ring_buffer 环形缓冲区对象。
 * @return 当前可读取的字节数。
 */
uint16_t RingBuffer_GetLength(const RingBuffer *ring_buffer);

/**
 * @brief 清空环形缓冲区中的未读取数据。
 * @param ring_buffer 环形缓冲区对象。
 */
void RingBuffer_Clear(RingBuffer *ring_buffer);

#endif
