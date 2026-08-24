#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include "stm32f10x.h" // 引入 STM32 的外设定�?
#include "stm32f10x_spi.h"
// 定义数据包结�?
typedef struct
{
  uint8_t STX;   // 开始字符，十六进制AA
  uint8_t CMD;   // 命令字节，十六进�?0表示握手命令
  uint8_t DATA1; // 数据1
  uint8_t DATA2; // 数据2
  uint8_t DATA3; // 数据3
  uint8_t DATA4; // 数据4
  uint8_t DATA5; // 数据5
  uint8_t DATA6; // 数据6
  uint8_t ETX;   // 结束字符，十六进�?5
  uint8_t SUM;   // 校验�?
} Packet;

// 函数声明
void initPacket(Packet *pkt);
void computeChecksum(Packet *pkt);
void createHandshakeReply(Packet *pkt);
void Protocol_HandleByte(uint8_t byte); // 串口逐字节处�?
void USART_ProtocolHandler(uint8_t *rxBuffer);
void parse_frame(uint8_t *frame);
uint8_t Protocol_TakeCommand(uint8_t *cmd, uint8_t *data);
void handle_command(uint8_t cmd, uint8_t *data);
void send_frame_usart1(uint8_t cmd, uint8_t *data);

/**
 * 初始�?USART2 TX 诊断状态�?
 */
void Protocol_InitTxDiagnostics(void);

/**
 * 显式清除最近一�?USART2 TX timeout 证据�?
 */
void Protocol_ResetUsart2TxTimeoutEvidence(void);

/**
 * 原子记录 USART2 TX timeout 现场�?
 *
 * stage=1 表示 TXE timeout，stage=2 表示 TC timeout�?
 */
void Protocol_RecordUsart2TxTimeout(uint8_t stage);

/**
 * 发送命�?ACK 或查询回复�?
 */
void send_frame(uint8_t cmd, uint8_t *data);

/**
 * 发送周期或状态主动上报�?
 */
void send_frame_event(uint8_t cmd, uint8_t *data);

void send_temperature_frame(uint8_t cmd);
void send_motor16_timeout_event(uint8_t motor_id, uint8_t direction);
extern uint8_t motor16_timeout_report[2];
void send_frame2(uint8_t cmd, uint8_t *data);
void sendPacket(USART_TypeDef *USARTx, const Packet *pkt);
void send_165Data(void);

extern volatile uint32_t dbg_usart_rx_bytes;
extern volatile uint32_t dbg_usart_rx_frames;
extern volatile uint32_t dbg_usart_ore;
extern volatile uint32_t dbg_usart_fe;
extern volatile uint32_t dbg_usart_ne;
extern volatile uint32_t dbg_usart_pe;
extern volatile uint32_t dbg_parser_bad_head;
extern volatile uint32_t dbg_parser_bad_tail;
extern volatile uint32_t dbg_parser_bad_checksum;
extern volatile uint32_t dbg_frame_queue_overflow;
extern volatile uint32_t dbg_cmd_queue_overflow;
extern volatile uint32_t dbg_rx_cmd_count[256];
extern volatile uint32_t dbg_handle_cmd_count[256];
extern volatile uint32_t dbg_tx_cmd_count[256];

/* CMD=0x08 �ֳ����ʹ�� TIM2 �ĺ��뵥��ʱ�ӣ��������� RAM �� Keil Watch �鿴�� */
extern volatile uint32_t dbg_cmd08_rx_time_ms;
extern volatile uint32_t dbg_cmd08_handle_time_ms;
extern volatile uint32_t dbg_cmd08_tx_done_time_ms;
extern volatile uint32_t dbg_cmd08_max_queue_delay_ms;
extern volatile uint32_t dbg_cmd08_max_process_tx_delay_ms;
extern volatile uint32_t dbg_cmd08_max_total_delay_ms;

#endif
