#ifndef __USART_H
#define __USART_H
#include <stm32f10x_usart.h>
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
/**
 * @brief 以独立帧方式发送一个字节，并在结束或失败时恢复 RS485 接收态。
 */
uint8_t USART_SendByte(USART_TypeDef* USARTx, uint8_t data);
/**
 * @brief 将字符串作为一个连续发送单元输出，避免 RS485 在字符之间切换方向。
 */
void USART_SendString(USART_TypeDef* USARTx, char *str);
/**
 * 独占指定 UART 完整发送一个缓冲区。
 *
 * 对 USART1/USART3 的 RS485 链路，函数在写入第一个字节前取得发送权，
 * 并在最后一个字节实际离开 TX 引脚（TC=1）后才恢复接收态和释放发送权。
 * 一旦已写入任一字节，TXE/TC 等待即使超过诊断阈值也会继续等待，不能返回半帧。
 *
 * @param USARTx 目标 UART；主机协议使用 USART1。
 * @param buffer 待发送的连续数据；本函数同步返回前不会保留该指针。
 * @param length 本次必须连续发送的字节数。
 * @return 返回 1 表示整帧发送完毕且最终 TC=1；返回 0 表示在首字节前因参数、
 *         发送权忙或前序 TC 超时而未开始本帧。失败原因见 rs485_tx_last_failure_reason。
 */
uint8_t USART_SendBuffer(USART_TypeDef* USARTx, uint8_t *buffer, uint16_t length);

/* USART1 发送阶段最长等待时间，单位为 TIM2 毫秒 tick，仅供 Keil Watch 诊断。 */
extern volatile uint32_t rs485_tx_max_txe_wait_ms;
extern volatile uint32_t rs485_tx_max_tc_wait_ms;
/**
 * 设置下一次 USART1 发送失败快照应记录的协议命令字。
 *
 * 仅用于发送层诊断；不会写入 UART、不会改变线上协议帧。
 *
 * @param cmd 当前准备发送的协议命令字；非协议调试发送使用 0xFF。
 */
void USART1_SetTxDiagnosticCommand(uint8_t cmd);
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
extern volatile uint32_t rs485_tx_call_count;
extern volatile uint32_t rs485_tx_success_count;
extern volatile uint32_t rs485_tx_txe_timeout_count;
extern volatile uint32_t rs485_tx_tc_timeout_count;
extern volatile uint32_t rs485_tx_bytes_count;
extern volatile uint16_t rs485_tx_last_expected_len;
extern volatile uint16_t rs485_tx_last_sent_len;
extern volatile uint8_t rs485_tx_last_failure_reason;
extern volatile uint32_t rs485_tx_failure_count;
extern volatile uint16_t rs485_tx_last_error_expected_len;
extern volatile uint16_t rs485_tx_last_error_sent_len;
extern volatile uint8_t rs485_tx_last_error_reason;
extern volatile uint8_t rs485_tx_last_error_cmd;
#endif
