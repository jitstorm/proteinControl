#include "shift_register_input.h"
#include "spi_helper.h"
#include "pb10_sensor_stop.h"

// 74HC165 控制引脚
#define LOAD_PIN GPIO_Pin_4
#define LOAD_PORT GPIOA
#define CLK_PIN GPIO_Pin_5
#define DATA_PIN GPIO_Pin_6
#define SPI_PORT GPIOA
#define DATA_PORT GPIOA
#define CLK_PORT GPIOA

uint8_t inputData[SHIFT_REGISTER_INPUT_COUNT] = {0}; // ✅ 只在这定义一次

void ShiftRegisterInput_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI1_InitOnce();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = LOAD_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LOAD_PORT, &GPIO_InitStructure);
}

void ShiftRegisterInput_ReadAll(uint8_t *data)
{
  volatile int i;
  GPIO_ResetBits(LOAD_PORT, LOAD_PIN);
  delay_us(10);
  GPIO_SetBits(LOAD_PORT, LOAD_PIN);
  delay_us(10);
  for (i = 0; i < SHIFT_REGISTER_INPUT_COUNT; i++)
  {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
      ;
    SPI_I2S_SendData(SPI1, 0xFF);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
      ;
    data[i] = ~(SPI_I2S_ReceiveData(SPI1)); // 通常为低有效
  }

  /* 只在一轮 HC165 数据全部刷新后判断，避免使用混合快照。 */
  Protocol_CheckPb10StopSensor(data);
}
