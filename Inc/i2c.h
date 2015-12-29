/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c3;

void I2C1_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE); 
void I2C1_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE);


#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

