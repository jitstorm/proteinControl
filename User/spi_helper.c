#include "spi_helper.h"
#include "DELAY.h"

// 595 控制引脚定义
#define NSS_PIN GPIO_Pin_9
#define NSS_PORT GPIOB          // PC4 → RCK（锁存）
#define SPI_MOSI_PIN GPIO_Pin_8 // PB8
#define SPI_CS_PIN GPIO_Pin_9   // PB9
#define SPI_SCK_PIN GPIO_Pin_5  // PA5
void SPI1_InitOnce(void)
{
    static uint8_t initialized = 0;

    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    if (initialized)
        return;
    initialized = 1;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    // SCK (PA5)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // MISO (PA6)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // MOSI (PA7)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // NSS (假设是PA4)
    GPIO_InitStructure.GPIO_Pin = NSS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(NSS_PORT, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
}

void SPI_SendByte(uint8_t data)
{
    // 手动拉低 NSS，表示开始通信
    GPIO_ResetBits(NSS_PORT, NSS_PIN);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(SPI1, data);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    (void)SPI_I2S_ReceiveData(SPI1); // 读取以清标志

    // 通信结束，拉高 NSS
    GPIO_SetBits(NSS_PORT, NSS_PIN);
}
uint8_t SPI_TransferByte(uint8_t data)
{
    uint8_t received;

    GPIO_ResetBits(NSS_PORT, NSS_PIN); // 拉低 NSS 开始通信

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(SPI1, data); // 发送数据

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    received = SPI_I2S_ReceiveData(SPI1); // 接收数据

    GPIO_SetBits(NSS_PORT, NSS_PIN); // 拉高 NSS 结束通信

    return received;
}

// 模拟的
void SPI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // MOSI - PB8
    GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // CS - PB9
    GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 默认状态
    // GPIO_SetBits(GPIOA, SPI_SCK_PIN);    // SCK 高电平（初始空闲，后拉低）
    GPIO_SetBits(GPIOB, SPI_CS_PIN);     // CS 拉高
    GPIO_ResetBits(GPIOB, SPI_MOSI_PIN); // MOSI 先清 0
}

// 模拟 SPI 发送函数（8 位）
// void SPI_Soft_SendByte(uint8_t data)
// {
//     uint8_t i;
//     GPIO_ResetBits(GPIOB, SPI_CS_PIN); // CS 拉低开始通信
//     delay_us(2);

//     for (i = 0; i < 8; i++)
//     {
//         // 先写数据（MSB先）
//         if (data & 0x80)
//             GPIO_SetBits(GPIOB, SPI_MOSI_PIN);
//         else
//             GPIO_ResetBits(GPIOB, SPI_MOSI_PIN);

//         // 产生一个 SCK 上升沿（从机读取数据）
//         GPIO_ResetBits(GPIOA, SPI_SCK_PIN); // SCK=0
//         delay_us(2);
//         GPIO_SetBits(GPIOA, SPI_SCK_PIN); // SCK=1
//         delay_us(2);

//         data <<= 1; // 下一位
//     }
//     GPIO_SetBits(GPIOB, SPI_CS_PIN); // CS 拉高结束
//     delay_us(10);
// }
void SPI_Soft_SendByte(uint8_t data)
{
    uint8_t i;

    GPIO_ResetBits(GPIOA, SPI_SCK_PIN); // SCK 先拉低，重要！
    delay_us(2);

    GPIO_ResetBits(GPIOB, SPI_CS_PIN); // 再拉低 CS 开始传输
    delay_us(2);

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            GPIO_SetBits(GPIOB, SPI_MOSI_PIN);
        else
            GPIO_ResetBits(GPIOB, SPI_MOSI_PIN);

        GPIO_SetBits(GPIOA, SPI_SCK_PIN); // SCK=1，上升沿供从机采样
        delay_us(10);
        GPIO_ResetBits(GPIOA, SPI_SCK_PIN); // SCK=0，准备下一位
        delay_us(10);
        data <<= 1;
    }

    GPIO_SetBits(GPIOB, SPI_CS_PIN); // CS 拉高结束
    delay_us(10);
}
