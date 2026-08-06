#include <DELAY.h>

static uint8_t fac_us = 0;  // us延时倍乘数
static uint16_t fac_ms = 0; // ms延时倍乘数

// 初始化延时函数
void Delay_Init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); // SysTick使用HCLK/8
    fac_us = SystemCoreClock / 8000000;                   // 计算1us需要多少计数
    fac_ms = (uint16_t)fac_us * 1000;                     // 计算1ms需要多少计数
}

// 微秒延时
void Delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * fac_us;
    SysTick->VAL = 0x00;
    SysTick->CTRL = 0x01; // 开始倒数
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // 等待时间到
    SysTick->CTRL = 0x00;
    SysTick->VAL = 0x00;
}

// 毫秒延时
void Delay_ms(uint16_t nms)
{
    while (nms--)
    {
        Delay_us(1000);
    }
}

void Delay(uint32_t nCount)
{
    for (; nCount != 0; nCount--)
        ;
}

// 粗延时函数，微秒
void delay_us(uint16_t time)
{
    uint16_t i = 0;
    while (time--)
    {
        i = 10; // 自己定义
        while (i--)
            ;
    }
}
// 毫秒级的延时
void delay_ms(uint16_t time)
{
    uint16_t i = 0;
    while (time--)
    {
        i = 12000; // 自己定义
        while (i--)
            ;
    }
}
