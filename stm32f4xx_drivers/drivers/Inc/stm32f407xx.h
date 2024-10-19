/*
 * stm32f407xx.h
 *
 *  Created on: Oct 18, 2024
 *  Author: rams
 *  Description: STM32F407VG Microcontroller Specific header file.
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/* Includes */
#include "stdint.h"



/*
 * @description: Base addresses of embedded memories: FLASH, SRAM1.
 */
#define FLASH_BASEADDR				0x80000000U				/* Flash memory's base address in STM32F407VG */
#define SRAM1_BASEADDR				0x20000000U				/* SRAM1 memory's base address in STM32F407VG */
#define SRAM						SRAM1_BASEADDR			/* Since SRAM1 is mostly used, create an alias for SRAM */



/*
 * @description: Base addresses of AHBx & APBx bus domains
 */
#define PERIPH_BASEADDR				0x40000000U				/* Base address of peripherals in memory map of stm32 */
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR			/* Base address of peripherals hanging in APB1 bus of stm32 */
#define APB2PERIPH_BASEADDR			0x40010000U				/* Base address of peripherals hanging in APB2 bus of stm32 */
#define AHB1PERIPH_BASEADDR			0x40020000U				/* Base address of peripherals hanging in AHB1 bus of stm32 */
#define AHB2PERIPH_BASEADDR			0x50000000U				/* Base address of peripherals hanging in AHB2 bus of stm32 */



/*
 * @description: Base addresses of all peripherals hanging in AHB1 bus
 */

/* Peripheral: GPIO - General Purpose Input-Output*/
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)



/*
 * @description: Base addresses of all peripherals hanging in APB1 bus
 */

/* Peripheral: SPI - Serial Peripheral Interface*/
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

/* Peripheral: I2C - Inter-Integrated Circuit*/
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

/* Peripheral: USART - Universal Synchronous and Asynchronous Receiver-Transmitter*/
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)

/* Peripheral: USART - Universal Asynchronous Receiver-Transmitter*/
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR				(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR				(APB1PERIPH_BASEADDR + 0x7C00)



/*
 * @description: Base addresses of all peripherals hanging in APB2 bus
 */

/* Peripheral: USART - Universal Synchronous and Asynchronous Receiver-Transmitter*/
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

/* Peripheral: SPI - Serial Peripheral Interface*/
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400)

/* Peripheral: External Interrupt/Event Controller*/
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

/* Peripheral: System Configuration Controller*/
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

#endif /* INC_STM32F407XX_H_ */
