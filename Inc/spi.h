#ifndef __spi_H
#define __spi_H

#include "stm32f4xx_hal.h"


extern SPI_HandleTypeDef hspi5;

uint8_t SPI_Read(uint8_t address);
void SPI_Read_Buf(uint8_t address, uint8_t *data, uint8_t cnt);
void SPI_Write(uint8_t address, uint8_t data);
void SPI_Write_Buf(uint8_t address, uint8_t *data, uint8_t cnt);
void SPI_Send(uint8_t data);
void SPI_Init_Pins(void);

#endif /*__ spi_H */

