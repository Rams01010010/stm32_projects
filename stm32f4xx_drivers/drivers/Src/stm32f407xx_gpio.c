/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Oct 21, 2024
 *  Author: rams
 *  Description: GPIO Driver Source file.
 */

#include "stm32f407xx_gpio.h"


/*
 * @description: Enables or Disables clock for the given GPIO Port.
 *
 * @parameters: *pGPIOx - Base address of the GPIO Port.
 * 				ENorDI - Enable/Disable.
 *
 * @note: Use ENABLE/DISABLE macros
 *
 * @returns: none
 *
 */
void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{

}



/*
 * @description: Initializes given GPIO Pin with given configuration.
 *
 * @parameters: *pGPIOHandle - Address of the GPIO Handle object.
 *
 * @returns: none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}



/*
 * @description: Resets given GPIO Port Registers.
 *
 * @parameters: *pGPIOx - Base address of the GPIO Port.
 *
 * @returns: none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}



/*
 * @description: Reads given Pin's state.
 *
 * @parameters: *pGPIOx - Base address of the Pin's GPIO Port.
 *				pinNumber - Pin Number of the input pin.
 *
 * @returns: Pin's value.
 *
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

}



/*
 * @description: Reads given Port's value.
 *
 * @parameters: *pGPIOx - Base address of the GPIO Port.
 *
 * @returns: Port's value.
 *
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{

}



/*
 * @description: Write output data to the given pin.
 *
 * @parameters: *pGPIOx - Base address of the Pin's GPIO Port.
 *				pinNumber - Pin Number of the output pin.
 *				value - Value to be written.
 *
 * @note: Use GPIO_PIN_SET/GPIO_PIN_RESET macros.
 *
 * @returns: none
 *
 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{

}



/*
 * @description: Write output data to the given port.
 *
 * @parameters: *pGPIOx - Base address of the GPIO Port.
 *				value - Value to be written.
 *
 * @note: Use 16-bit value for 16-pins of the port.
 *
 * @returns: none
 *
 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

}



/*
 * @description: Toggle the given pin.
 *
 * @parameters: *pGPIOx - Base address of the Pin's GPIO Port.
 *				pinNumber - Pin Number of the pin to toggle.
 *
 * @returns: none
 *
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);										// Toggle pin's state



void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI)
{

}



/*
 * @description: Interrupt Service Routine Code that will be called after an interrupt.
 *
 * @parameters: pinNumber - Pin Number of the interrupt pin.
 *
 * @returns: none
 *
 */
void GPIO_IRQHandler(uint8_t pinNumber)
{

}
