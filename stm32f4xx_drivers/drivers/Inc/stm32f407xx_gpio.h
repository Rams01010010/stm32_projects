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
 * @description: GPIO Specific macros.
 */
//----------------------------------------------------------
// @GPIO_PIN_MODES
//----------------------------------------------------------
#define GPIO_MODE_IN			0		// Input Mode
#define GPIO_MODE_OUT			1		// Output Mode
#define GPIO_MODE_ALTFUNC		2		// Alternate Function Mode
#define GPIO_MODE_ANALOG		3		// Analog Mode
#define GPIO_MODE_IT_FT			4		// Interrupt Falling-edge Trigger
#define GPIO_MODE_IT_RT			5		// Interrupt Rising-edge Trigger
#define GPIO_MODE_IT_RFT		6		// Interrupt Rising/Falling-edge Trigger
//----------------------------------------------------------
// @GPIO_OUTPUT_TYPES
//----------------------------------------------------------
#define GPIO_OTYPE_PUSHPULL		0		// Output Type - Push/Pull
#define GPIO_OTYPE_OPENDRAIN	1		// Output Type - OpenDrain
//----------------------------------------------------------
// @GPIO_OUTPUT_SPEEDS
//----------------------------------------------------------
#define GPIO_OSPEED_LOW			0		// Speed - Low
#define GPIO_OSPEED_MEDIUM		1		// Speed - Medium
#define GPIO_OSPEED_HIGH		2		// Speed - High
#define GPIO_OSPEED_VERYHIGH	3		// Speed - VeryHigh
//----------------------------------------------------------
// @GPIO_PIN_PUPD
//----------------------------------------------------------
#define GPIO_PIN_NOPUPD		0		// No internal pull up/down resistor
#define GPIO_PIN_PULLUP		1		// Internal PullUp resistor
#define GPIO_PIN_PULLDOWN	2		// Internal PullDown resistor
//----------------------------------------------------------
// @GPIO_PIN_NUMBERs
//----------------------------------------------------------
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15
//----------------------------------------------------------



/*
 * @description: Holds config details of the pin.
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			// Possible values: 0 - 15
	uint8_t GPIO_PinMode;			// Possible values: @GPIO_PIN_MODES
	uint8_t GPIO_PinOpSpeed;		// Possible values: @GPIO_OUTPUT_SPEEDS
	uint8_t GPIO_PinPuPdControl;	// Possible values: @GPIO_PIN_PUPD
	uint8_t GPIO_PinOpType;			// Possible values: @GPIO_OUTPUT_TYPES
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;



/*
 * @description: Handle structure for GPIO pin.
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; 			 // Points to the pin's port address.
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
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);						// Write data to an output port
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);					// Toggle pin's state

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);				// IRQ Interrupt Configuration
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t priority);				// IRQ Priority Configuration
void GPIO_IRQHandler(uint8_t pinNumber);										// InterruptServiceRoutine Handling

#endif /* INC_STM32F407XX_GPIO_H_ */
