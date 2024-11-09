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

		// Make sure to mask GPIO_PinMode to avoid garbage values.
		pGPIOHandle->GPIO_PinConfig.GPIO_PinMode &= (0x3);

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
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1.Configure falling-edge trigger selection register (FTSR)

			// Clear the corresonding bit before setting.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Set the corresponding bit.
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding bit in RTSR.
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1.Configure rising-edge trigger selection register (RTSR)

			// Clear the corresonding bit before setting.
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Set the corresponding bit.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding bit in FTSR.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1.Configure both FTSR and RTSR

			// Clear the corresonding bits in FTSR,RTSR before setting.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Set the corresponding bits in FTSR,RTSR.
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		// Calculate the register to be used. i.e EXTCR{1,2,3,4}.
		uint8_t exti_reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		// Calculate the starting bit position for a given pin number.
		uint8_t exti_bit_position = 4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		// Calculate the port code to be programmed in the EXTICR register.
		uint8_t exti_port_code = GPIO_PORT_TO_CODE(pGPIOHandle->pGPIOx);
		// Enable clock for SYSCFG register
		SYSCFG_PCLK_EN();
		// Set the EXTICR appropriately.
		SYSCFG->EXTICR[exti_reg] = exti_port_code << exti_bit_position;

		// 3. Enable the EXTI Interrupt delivery using IMR
		// Clear the corresonding bit before setting.
		EXTI->IMR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// Set the corresponding bit.
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2.Configure Speed
	// Make sure to mask GPIO_PinOpSpeed to avoid garbage values.
	pGPIOHandle->GPIO_PinConfig.GPIO_PinOpSpeed &= (0x3);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3.Configure PullUpPullDown
	// Make sure to mask GPIO_PinPuPdControl to avoid garbage values.
	pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl &= (0x3);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4.Configure Output type
	// Make sure to mask GPIO_PinOpType to avoid garbage values.
	pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType &= (0x1);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure Alternate Functionality
	// Make sure to mask GPIO_PinAltFuncMode to avoid garbage values.
	pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode &= (0xF);
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



/*
 * @description: Enable or Disable a given IRQ Number in NVIC.
 *
 * @parameters: IRQNumber - IRQ Number (position in vector table).
 * 				ENorDI    - Enable or Disable.
 *
 * @note: Use MACROs ENABLE/DISABLE for ENorDI.
 *
 * @returns: none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	// Create variables for pointing to the NVIC registers.
	volatile uint32_t *pISER_BaseAddr = (uint32_t *) NVIC_ISER_BASEADDR;
	volatile uint32_t *pICER_BaseAddr = (uint32_t *) NVIC_ICER_BASEADDR;
	// Calculate offset values.
	uint8_t reg_offset = IRQNumber / 32;
	uint8_t bit_offset = IRQNumber % 32;

	if(ENorDI == ENABLE)
	{
		if(IRQNumber >= 0 && IRQNumber <= 95)
		{
			// Configure ISER{0,1,2} register appropriately.
			*(pISER_BaseAddr + reg_offset) |= (1 << bit_offset);
		}
	}
	else
	{
		if(IRQNumber >= 0 && IRQNumber <= 95)
		{
			// Configure ICER{0,1,2} register appropriately.
			*(pICER_BaseAddr + reg_offset) |= (1 << bit_offset);
		}
	}
}



/*
 * @description: Set the priority for a given IRQ Number in NVIC.
 *
 * @parameters: IRQNumber   - IRQ Number (position).
 * 				IRQPriority - Priority of the IRQ Number (0 - 15).
 *
 * @note: Since STM32F407 MCU allows only 16 levels of priority, the
 * 		  possible values for IRQPriority ranges from 0 till 15.
 *
 * @returns: none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Create variables for pointing to IPR register and to calculate reg & bit offset.
	volatile uint32_t *pIPR_BaseAddr = (uint32_t *) NVIC_IPR_BASEADDR;
	uint8_t reg_offset = IRQNumber / 4;
	// Since the least 4 bits are not implemented in priority value...
	// Priority starts from bit4.
	uint8_t bit_offset = (IRQNumber % 4) * 8 + (8 - NO_OF_PR_BITS_IMPLEMENTED);

	// Set the priority in the appropriate IPR register
	*(pIPR_BaseAddr + reg_offset) = (IRQPriority << bit_offset);
}



/*
 * @description: Interrupt Request Handling code called after an interrupt occurs.
 * 				 (Clears the pending register EXTI_PR to mark it as handled).
 *
 * @parameters: pinNumber - Pin Number of the interrupt pin.
 *
 * @returns: none
 *
 */
void GPIO_IRQHandler(uint8_t pinNumber)
{
	// If the interrupt on a pin is pending
	if(EXTI->PR & (1 << pinNumber))
	{
		// Clear it by writing '1' to it.
		EXTI->PR |= (1 << pinNumber);
	}
}
