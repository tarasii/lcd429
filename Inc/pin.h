#include "stdint.h"

#define GPIO_HIGH(GPIOx, GPIO_Pin) 		GPIOx->BSRR = GPIO_Pin
#define GPIO_LOW(GPIOx, GPIO_Pin)			GPIOx->BSRR = GPIO_Pin << 16
#define GPIO_TOGGLE(GPIOx, GPIO_Pin) 	GPIOx->ODR ^= GPIO_Pin
#define GPIO_GETPINVAL(GPIOx, GPIO_Pin)	(((GPIOx)->IDR & (GPIO_Pin)) == 0 ? 0 : 1)

extern uint8_t flagButton;

