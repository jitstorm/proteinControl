#include <stm32f10x_gpio.h>
void GPIO_Config(void)
{
//   GPIO_InitTypeDef GPIo_InitStructure;

//   // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

//   // GPIo_InitStructure.GPIO_Pin = GPIO_Pin_4;
//   // GPIo_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

//   // RCC_APB2PeriphClockCmd(RCC_APB2Periph_POIOA, ENABLE);

//   // 使能GPIO时钟
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

//   // 配置TX引脚(推挽输出)
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);
  
//   // 配置RX引脚(浮空输入)
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);

  // // 使能GPIO时钟
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  // // 配置TX引脚(推挽输出)
  // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  // GPIO_Init(GPIOB, &GPIO_InitStructure);

  // // 配置RX引脚(浮空输入)
  // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  // GPIO_Init(GPIOB, &GPIO_InitStructure);

  
}

