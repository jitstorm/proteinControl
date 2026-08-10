#ifndef __USART_H
#define __USART_H
#include <stm32f10x_usart.h>
#include <stdio.h>
#include <stdint.h>

#define USART1_RX_BUFFER_SIZE 512u
#define USART3_RX_BUFFER_SIZE 512u

typedef struct
{
    volatile uint32_t rx_count;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t frame_ok_count;
    volatile uint32_t frame_error_count;
} USART_RxStats;
void USART1_Init(void);
void USART1_IRQHandler(void);

/**
 * @brief 轮询处理串口 1 环形缓冲区中的接收数据。
 */
void USART1_Process(void);

/**
 * @brief 轮询处理串口 3 环形缓冲区中的接收数据。
 */
void USART3_Process(void);
void USART_SendByte(USART_TypeDef* USARTx, uint8_t data);
void USART_SendString(USART_TypeDef* USARTx, char *str);
void USART_SendBuffer(USART_TypeDef* USARTx, uint8_t *buffer, uint16_t length);
unsigned char UART1GetByte(unsigned char *GetData);
void UART1Test(void);
uint16_t USART1_GetRemoteTemperature(void);
/**
 * @brief 从串口1的485接收队列取出一帧主机协议数据。
 */
uint8_t USART1_TakeFrame(uint8_t *frame);
extern volatile uint8_t frame_ready;
extern uint8_t rx_buffer[10];
extern USART_RxStats usart1_rx_stats;
extern USART_RxStats usart3_rx_stats;
#endif
