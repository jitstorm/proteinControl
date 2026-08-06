#ifndef __SPI_HELPER_H
#define __SPI_HELPER_H

#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
void SPI1_InitOnce(void);
// void SPI_SendByte(uint8_t data);
void SPI_SendByte(uint8_t data);
uint8_t SPI_TransferByte(uint8_t data);
void SPI_GPIO_Init(void);
void SPI_Soft_SendByte(uint8_t data);
#endif
