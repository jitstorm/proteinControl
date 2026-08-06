#include "max31855.h"
#include "stm32f10x.h"
#include "DELAY.h" // 你需要实现一个基本的延迟函数

// 定义 GPIO 端口和引脚
#define MAX31855_CS_PORT GPIOB
#define MAX31855_CS_PIN GPIO_Pin_2

#define MAX31855_SCK_PORT GPIOC
#define MAX31855_SCK_PIN GPIO_Pin_7

#define MAX31855_MISO_PORT GPIOC
#define MAX31855_MISO_PIN GPIO_Pin_8

// 设置宏操作引脚
#define MAX31855_CS_HIGH() GPIO_SetBits(MAX31855_CS_PORT, MAX31855_CS_PIN)
#define MAX31855_CS_LOW() GPIO_ResetBits(MAX31855_CS_PORT, MAX31855_CS_PIN)

#define MAX31855_SCK_HIGH() GPIO_SetBits(MAX31855_SCK_PORT, MAX31855_SCK_PIN)
#define MAX31855_SCK_LOW() GPIO_ResetBits(MAX31855_SCK_PORT, MAX31855_SCK_PIN)

#define MAX31855_READ_MISO() GPIO_ReadInputDataBit(MAX31855_MISO_PORT, MAX31855_MISO_PIN)

void MAX31855_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE);
    // 配置CS和SCK为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // 配置CS和SCK为推挽输出
    GPIO_InitStructure.GPIO_Pin = MAX31855_CS_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX31855_CS_PORT, &GPIO_InitStructure);
    // 配置CS和SCK为推挽输出
    GPIO_InitStructure.GPIO_Pin = MAX31855_SCK_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX31855_SCK_PORT, &GPIO_InitStructure);
    // 配置MISO为浮空输入
    GPIO_InitStructure.GPIO_Pin = MAX31855_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MAX31855_MISO_PORT, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_6);
    MAX31855_CS_HIGH(); // 默认拉高CS
    MAX31855_SCK_LOW(); // 默认拉低SCK
}



// 读取1位
static uint8_t SPI_ReadBit(void)
{
    uint8_t bit_value;
    MAX31855_SCK_HIGH();
    Delay_us(1); // 你需要实现Delay_us微秒延时函数
    bit_value = MAX31855_READ_MISO();
    MAX31855_SCK_LOW();
    Delay_us(1); // 你需要实现Delay_us微秒延时函数
    return bit_value;
}

// 读取32位原始数据
static uint32_t SPI_Read32Bit(void)
{
    uint32_t dat = 0;
    uint8_t i;
    for (i = 0; i < 32; i++)
    {
        dat <<= 1;
        dat |= SPI_ReadBit();
    }
    return dat;
}

// 获取温度值（单位：℃）
float MAX31855_GetTemperature(void)
{
    uint32_t raw_data;
    int16_t thermocouple_temp;

    MAX31855_CS_LOW();
    Delay_us(1000);
    raw_data = SPI_Read32Bit();
    Delay_us(1000);
    MAX31855_CS_HIGH();
    // 检查故障位
    if (raw_data & 0x00010000)
        return -1000.0; // 表示故障

    // 提取热电偶温度位（前14位），右移18
    thermocouple_temp = (raw_data >> 18) & 0x3FFF;
    if (thermocouple_temp & 0x2000)
    {
        // 负温度，手动符号扩展
        thermocouple_temp |= 0xC000;
    }
    return thermocouple_temp * 0.25f;
}
