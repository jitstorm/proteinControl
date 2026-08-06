#include <CLOCK.h>
void SystemClock_Config(void)
{
    RCC_DeInit();  // 复位RCC配置
    
    // 开启外部高速晶振（HSE）
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_WaitForHSEStartUp() != SUCCESS);  // 等待HSE稳定
    
    // 配置PLL（HSE作为PLL输入，9倍频 -> 72MHz）
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    
    // 使能PLL并等待就绪
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    
    // 设置系统时钟源为PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);  // 检查是否切换成功
    
    // 设置APB1 = 36MHz（最大72MHz/2），APB2 = 72MHz
    RCC_HCLKConfig(RCC_SYSCLK_Div1);      // AHB = 72MHz
    RCC_PCLK1Config(RCC_HCLK_Div2);       // APB1 = 36MHz
    RCC_PCLK2Config(RCC_HCLK_Div1);       // APB2 = 72MHz
    
    // 使能USART1时钟（APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}
