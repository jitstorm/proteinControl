#ifndef BUFFER_H
#define BUFFER_H
#include <stdint.h>


// 定义数据包结构
 typedef Packet {
  uint8_t STX;   // 开始字符，十六进制AA
  uint8_t CMD;   // 命令字节，十六进制00表示握手命令
  uint8_t DATA1; // 数据1
  uint8_t DATA2; // 数据2
  uint8_t DATA3; // 数据3
  uint8_t DATA4; // 数据4
  uint8_t DATA5; // 数据5
  uint8_t DATA6; // 数据6
  uint8_t ETX;   // 结束字符，十六进制55
  uint8_t SUM;   // 校验和
};
// 函数声明
void initPacket(Packet* pkt);
void computeChecksum(Packet* pkt);
void createHandshakeReply(Packet* pkt);
// 从串行端口接收数据包的函数
bool receivePacket(Packet *packet);

// 发送数据包的函数
void sendPacket(const Packet &packet);

// 计算校验和的函数
uint8_t calculateChecksum(uint8_t *data, uint8_t length);

#include <Buffer.c>
#endif