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

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*
 * @description: This project demonstrates, outputting HSI clock on MCO1(PA8 pin).
 * 				 The output could be measured using a logic analyzer.
 */

int main(void)
{
	// Initialize pointers to registers
	// RCC Base Address   : 0x4002_3800
	// RCC_CFGR Offset    : 0x08 (08)
	// GPIOA Base Address : 0x4002_0000
	// GPIOA Moder Offset : 0x00 (00)
	// GPIOA AFRH Offset  : 0x24 (36)
	volatile uint32_t *const pRCC = (uint32_t*) (0x40023800);
	volatile uint32_t *const pRCC_CFGR = pRCC + 2;
	volatile uint32_t *const pRCC_AHB1ENR = pRCC + 12;
	volatile uint32_t *const pGPIOA = (uint32_t*) (0x40020000);
	volatile uint32_t *const pGPIOA_MODER = pGPIOA + 0;
	volatile uint32_t *const pGPIOA_AFRH = pGPIOA + 36;

	// Set the CFGR register's MCO1 config bits(22:21) to 00 (HSI Clk Source).
	*pRCC_CFGR &= ~(0x3 << 21);
	// Enable the clock for GPIOA port
	*pRCC_AHB1ENR |= 0x1;
	// Set PA8's mode to Alternate Function mode (10).
	*pGPIOA_MODER &= ~(0x3 << 16);
	*pGPIOA_MODER |=  (0x2 << 16);
	// Set the AFRH register's 8th pin's config bits(3:0) to 0000(AFR0 -> MCO1)
	*pGPIOA_AFRH  &= ~(0xF);

	for(;;);
}
