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

int main(void)
{
	//AHB1ENR    => 0x40023800 + 0x30
	//GPIOD Mode => 0x40020C00 + 0x00
	//GPIOD ODR  => 0x40020C00 + 0x14

	//Initialize pointers to these registers
    uint32_t* pRCC   = (uint32_t*) 0x40023800;
    uint32_t* pGPIOD = (uint32_t*) 0x40020C00;

    uint32_t* pRCC_AHB1ENR = (uint32_t*) ((uint32_t) pRCC   + 0x30);
    uint32_t* pGPIOD_MODER = (uint32_t*) ((uint32_t) pGPIOD + 0x00);
    uint32_t* pGPIOD_ODR   = (uint32_t*) ((uint32_t) pGPIOD + 0x14);


    //Enable Clock for GPIOD
    *pRCC_AHB1ENR |= 1 << 3; //0x00000008


    /*
     Code for Green LED (PD12) Blink


    //Set GPIOD-12 Mode to Output Mode
    //a. clear 24th and 25th bit
    *pGPIOD_MODER &= 0xFCFFFFFF;
    *pGPIOD_MODER |= 0x01000000; //0x01000000

    while(1)
    {
    	//Toggle GPIOD-12
    	*pGPIOD_ODR ^= 0x00001000;

    	for(int i = 0; i < 500000; i++);
    }

    */


    //Code for pattern
    *pGPIOD_MODER &= 0x00FFFFFF;
    *pGPIOD_MODER |= 0x55000000;

    while(1)
    {
    	*pGPIOD_ODR = 0x00000000;

    	for(int led = 12; led < 16; led++){
    		*pGPIOD_ODR |= 1 << led;
    		for(int i = 0; i < 100000; i++);
    	}

    	for(int led = 12; led < 16; led++){
    		*pGPIOD_ODR &= (~1) << led;
    		for(int i = 0; i < 100000; i++);
    	}
    }

}
