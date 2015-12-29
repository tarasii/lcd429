/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED_SD
#define _DISKIO_DEFINED_SD

#define _USE_WRITE	1	/* 1: Enable disk_write function */
#define _USE_IOCTL	1	/* 1: Enable disk_ioctl fucntion */

#include "diskio.h"
#include "integer.h"

#include "stm32f4xx_hal.h"
#include "pin.h"

extern SPI_HandleTypeDef hspi5;
extern __IO uint32_t TM_Time2;

/* SPI settings */
#ifndef FATFS_SPI
#define FATFS_SPI							hspi5.Instance
#define FATFS_SPI_PINSPACK					0
#endif

/* CS pin settings */
#ifndef FATFS_CS_PIN		
#define FATFS_CS_PORT						GPIOF
#define FATFS_CS_PIN						GPIO_PIN_6
#endif

/* CS pin */
#define FATFS_CS_LOW						GPIO_LOW(FATFS_CS_PORT, FATFS_CS_PIN)
#define FATFS_CS_HIGH						GPIO_HIGH(FATFS_CS_PORT, FATFS_CS_PIN)

#define SPI_WAIT_TX(SPIx)                   while ((SPIx->SR & SPI_FLAG_TXE) == 0 || (SPIx->SR & SPI_FLAG_BSY))
#define SPI_WAIT_RX(SPIx)                   while ((SPIx->SR & SPI_FLAG_RXNE) == 0 || (SPIx->SR & SPI_FLAG_BSY))
#define SPI_CHECK_ENABLED(SPIx)             if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}
#define SPI_CHECK_ENABLED_RESP(SPIx, val)   if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return (val);}

#define TM_DELAY_Time2()				(TM_Time2)
#define TM_DELAY_SetTime2(time)			(TM_Time2 = (time))

#endif

