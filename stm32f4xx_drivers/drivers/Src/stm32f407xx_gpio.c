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
	if(ENorDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
	}
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
	uint32_t temp; //temporary register

	// 1. Configure MODE
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non-Interrupt Mode

		// Clear the corresponding bits (2*pinNumber, because 2-bits are required for 1-pin in MODER)
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Set the MODER register with that value
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		// Interrupt Mode
		// Will be coded later
	}

	// 2.Configure Speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3.Configure PullUpPullDown
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4.Configure Output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		// configure alternate function registers
		// Create variable to for calculating AFR(L/H) i.e AFR[0/1].
		// and to calculate the starting bit position for a given pin.
		uint32_t afr_reg, afr_bit_position;

		// Since 8 pins are accomodated in AFR(L/H), divide by 8.
		// To decide AFRL(0) and AFRH(1);
		afr_reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		// Since each pin's config is of 4bits, 4*pinNumber.
		afr_bit_position = 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);

		// Clear the pins before setting
		pGPIOHandle->pGPIOx->AFR[afr_reg] &= ~(0xF << afr_bit_position);
		pGPIOHandle->pGPIOx->AFR[afr_reg] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << afr_bit_position);

	}
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
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if(pGPIOx == GPIOI)
		GPIOI_REG_RESET();
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
	uint8_t value = (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x1);
	return value;
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
	uint16_t value = (uint16_t) pGPIOx->IDR;
	return value;
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
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
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
	pGPIOx->ODR = value;
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
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}



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
