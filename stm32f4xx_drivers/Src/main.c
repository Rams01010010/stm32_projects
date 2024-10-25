/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdint.h>
#include "stm32f407xx_gpio.h"

/*
 * @description: Toggles the orange led (LD3) connected to PD13
 * 				 of discovery board upon pressing user button
 * 				 connected to PA0.
 */
int main(void)
{
	// Create Handle for GPIO configs.
	GPIO_Handle_t gpio;

	// Enable the clock for GPIOD.
	GPIO_ClockControl(GPIOD, ENABLE);
	// Enable the clock for GPIOA.
	GPIO_ClockControl(GPIOA, ENABLE);

	// Configure the parameters for GPIOD.
	gpio.pGPIOx = GPIOD;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio.GPIO_PinConfig.GPIO_PinOpType = GPIO_OTYPE_PUSHPULL;
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PULLDOWN;

	// Initialize GPIOD-13 with required configs.
	GPIO_Init(&gpio);

	/*
	 * @note: The below configurations for user button input
	 * 		  can be skipped, the default mode will be
	 * 		  input mode (reset state).
	 *
	// Configure the parameters for GPIOA.
	gpio.pGPIOx = GPIOA;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;

	// Initialize GPIOA-0 with required configs.
	GPIO_Init(&gpio);
	*/

	while(1)
	{
		// In case the button is pressed.
		if(GPIO_ReadPin(GPIOA, GPIO_PIN_0))
			// Toggle LD3 led.
			GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		// Delay
		for(int i = 0; i < 300000; i++);
	}
}
