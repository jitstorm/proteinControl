#include "shift_register.h"
#include "spi_helper.h"
#include "DELAY.h"

// 595 控制引脚定义
#define LATCH_PIN GPIO_Pin_4
#define LATCH_PORT GPIOC // PC4 → RCK（锁存）

#define SCK_PIN GPIO_Pin_5
#define SCK_PORT GPIOA // PA5 → SRCLK（时钟）

#define MOSI_PIN GPIO_Pin_7
#define MOSI_PORT GPIOA // PA7 → SER（数据）

uint8_t HC595Data[SHIFT_REGISTER_COUNT] = {0x00,0x00,0x00,0x00}; // 示例数据
MotorCtrl_t MotorCtrl[MOTOR_NUM];
void ShiftRegister_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI1_InitOnce();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LATCH_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LATCH_PORT, &GPIO_InitStructure);
}

void ShiftRegister_Write(uint8_t data, uint8_t registerIndex)
{
    (void)registerIndex; // 如果不使用索引可以忽略
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )
        ;
    SPI_I2S_SendData(SPI1, data);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    (void)SPI_I2S_ReceiveData(SPI1); // 清除 RX
    GPIO_SetBits(LATCH_PORT, LATCH_PIN);
    GPIO_ResetBits(LATCH_PORT, LATCH_PIN);
}

void ShiftRegister_WriteAll(uint8_t *data)
{
    int i;
    for (i = 0; i < SHIFT_REGISTER_COUNT; i++)
    {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) 
            ;
        SPI_I2S_SendData(SPI1, data[i]);
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
            ;
        (void)SPI_I2S_ReceiveData(SPI1);
    }
 
    // 稍等一下再锁存
    Delay_us(1); // 加一个小于 1us 的延迟，比如 NOP 或你自己的 Delay_us()
 
    GPIO_SetBits(LATCH_PORT, LATCH_PIN);
    Delay_us(1); // 保证 latch 维持高电平一段时间
    GPIO_ResetBits(LATCH_PORT, LATCH_PIN);
}
