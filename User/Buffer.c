#ifndef BUFFER_CPP
#define BUFFER_CPP
#include "Buffer.h"
#include "USART.h"
// 计算校验和的函数
uint8_t calculateChecksum(uint8_t *packet) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(Packet) - 1; i++) { // 不包括SUM字段本身
    checksum += packet[i];
  }
  return checksum;
}
// 示例：创建一个握手回复包
Packet createHandshakeReply() {
  Packet pkt;
  pkt.CMD = 0x00;      // 握手命令
  pkt.DATA1 = 0x01;    // 示例数据，可以自定义
  pkt.DATA2 = 0x02;
  pkt.DATA3 = 0x03;
  pkt.DATA4 = 0x04;
  pkt.DATA5 = 0x05;
  pkt.DATA6 = 0x06;
  pkt.computeChecksum();
  return pkt;
}
/**
 * 通过统一 UART 发送边界输出兼容 Packet。
 *
 * 此遗留入口不得直接写 USART 数据寄存器；对 USART1 主机 RS485，
 * 必须由 USART_SendBuffer 保证整包发送和最终 TC 确认。
 *
 * @param USARTx 目标 UART。
 * @param pkt 待发送的兼容数据包。
 */
void sendPacket(USART_TypeDef* USARTx, const Packet& pkt) {
  const uint8_t* p = (const uint8_t*)&pkt;
    /* 统一走阻塞整帧发送边界，禁止此兼容入口绕过 USART1 的发送权和最终 TC 确认。 */
    (void)USART_SendBuffer(USARTx, (uint8_t*)p, sizeof(Packet));
}

bool receivePacket(Packet *packet)
{
  if (Serial3.readBytes((char *)packet, sizeof(Packet)) == sizeof(Packet))
  {
    // 验证校验和（不包括校验和字段本身）
    if (packet->SUM == calculateChecksum((uint8_t *)packet))
    {
      return true; // 校验和正确
    }
  }
  return false; // 数据包不完整或校验和不正确
}


#endif
