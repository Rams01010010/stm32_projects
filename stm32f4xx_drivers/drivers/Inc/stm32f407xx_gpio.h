/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Oct 21, 2024
 *  Author: rams
 *  Description: GPIO Driver Header file.
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/*
 * @description: Holds config details of the pin.
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;



/*
 * @description: Handle structure for GPIO pin.
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; // Points to the pin's port address.
	GPIO_PinConfig_t GPIO_PinConfig; // Holds the pin details.
}GPIO_Handle_t;



/*
 * @description: APIs supported by this driver.
 */

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);					// Enable/Disable clock for GPIOx

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);										// Initialize GPIOx port
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);										// DeInitialize GPIOx port

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);					// Read input data from a pin
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);									// Read input data from a port
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);	// Write data to an output pin
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);										// Write data to an output port
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);										// Toggle pin's state

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);	// IRQ Configuration
void GPIO_IRQHandler(uint8_t pinNumber);										// InterruptServiceRoutine Handling

#endif /* INC_STM32F407XX_GPIO_H_ */
