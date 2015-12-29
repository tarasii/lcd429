/* Includes ------------------------------------------------------------------*/
#include "i2c.h"




void I2C1_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE) 
{
    /* -> Start the transmission process */
    /* While the I2C in reception process, user can transmit data through "aTxBuffer" buffer */
    //while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS<<1, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK)
    while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK)
    {
        /*
         * Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */
 
        if (HAL_I2C_GetError(&hi2c3) != HAL_I2C_ERROR_AF)
        {
            //Error_Handler(3);
					  break;
        }
 
    }
 
    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
      while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
      {
      }
}

void I2C1_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE)
{
    /* -> Lets ask for register's address */
    //I2C1_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);
 
    /* -> Put I2C peripheral in reception process */
    //while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK)
    while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */
        if (HAL_I2C_GetError(&hi2c3) != HAL_I2C_ERROR_AF)
        {
            //Error_Handler(4);
					  break;
        }
    }
 
    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     **/
    while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
    {
    }
}

