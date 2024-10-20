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
#define FLASH_BASEADDR				0x08000000U				/* Flash memory's base address in STM32F407VG */
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
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)



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



/*
 * @description: Base addresses of all peripherals hanging in APB2 bus
 */

/* Peripheral: USART - Universal Synchronous and Asynchronous Receiver-Transmitter*/
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

/* Peripheral: SPI - Serial Peripheral Interface*/
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)

/* Peripheral: External Interrupt/Event Controller*/
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

/* Peripheral: System Configuration Controller*/
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)



/*
 * @description: Registers Definition of GPIO Peripheral
 */

typedef struct
{
	volatile uint32_t MODER;		// Mode Register; Offset : 0x00
	volatile uint32_t OTYPER;		// Output Register; Offset : 0x04
	volatile uint32_t OSPEEDR;		// Output Speed Register; Offset : 0x08
	volatile uint32_t PUPDR;		// Pull-up/Pull-down register; Offset: 0x0C
	volatile uint32_t IDR;			// Input Data Register; Offset: 0x10
	volatile uint32_t ODR;			// Output Data Register; Offset: 0x14
	volatile uint32_t BSRR;			// Bit Set/Reset Register; Offset: 0x18
	volatile uint32_t LCKR;			// Configuration Lock Register; Offset: 0x1C
	volatile uint32_t AFR[2];		// Alternate Function Low(AFR[0]) & High(AFR[1]) registers; Offset: 0x20
}GPIO_RegDef_t;



/*
 * @description: Registers Definition of RCC Peripheral
 */
typedef struct{
	volatile uint32_t CR;			// Clock Control Register; Offset: 0x00
	volatile uint32_t PLLCFGR;		// PLL Configuration Register; Offset: 0x04
	volatile uint32_t CFGR;			// Clock Configuration Register; Offset: 0x08
	volatile uint32_t CIR;			// Clock Interrupt Register; Offset: 0x0C
	volatile uint32_t AHB1RSTR;		// AHB1 Peripheral Reset Register; Offset: 0x10
	volatile uint32_t AHB2RSTR;		// AHB2 Peripheral Reset Register; Offset: 0x14
	volatile uint32_t AHB3RSTR;		// AHB3 Peripheral Reset Register; Offset: 0x18
	uint32_t RESERVED0;				// Reserved; Offset: 0x1C
	volatile uint32_t APB1RSTR;		// APB1 Peripheral Reset Register; Offset: 0x20
	volatile uint32_t APB2RSTR;		// APB2 Peripheral Reset Register; Offset: 0x24
	uint32_t RESERVED1[2];			// Reserved; Offset: 0x28, 0x2C
	volatile uint32_t AHB1ENR;		// AHB1 Peripheral Clock Enable Register; Offset: 0x30
	volatile uint32_t AHB2ENR;		// AHB2 Peripheral Clock Enable Register; Offset: 0x34
	volatile uint32_t AHB3ENR;		// AHB3 Peripheral Clock Enable Register; Offset: 0x38
	uint32_t RESERVED2;				// Reserved; Offset: 0x3C
	volatile uint32_t APB1ENR;		// APB1 Peripheral Clock Enable Register; Offset: 0x40
	volatile uint32_t APB2ENR;		// APB2 Peripheral Clock Enable Register; Offset: 0x44
	uint32_t RESERVED3[2];			// Reserved; Offset: 0x48, 0x4C
	volatile uint32_t AHB1LPENR;	// AHB1 Peripheral Clock Enable In Low Power Mode Register; Offset: 0x50
	volatile uint32_t AHB2LPENR;	// AHB2 Peripheral Clock Enable In Low Power Mode Register; Offset: 0x54
	volatile uint32_t AHB3LPENR;	// AHB3 Peripheral Clock Enable In Low Power Mode Register; Offset: 0x58
	uint32_t RESERVED4;				// Reserved; Offset: 0x5C
	volatile uint32_t APB1LPENR;	// APB1 Peripheral Clock Enable In Low Power Mode Register; Offset: 0x60
	volatile uint32_t APB2LPENR;	// APB2 Peripheral Clock Enable In Low Power Mode Register; Offset: 0x64
	uint32_t RESERVED5[2];			// Reserved; Offset: 0x68,0x6C
	volatile uint32_t BDCR;			// Backup Domain Control Register; Offset: 0x70
	volatile uint32_t CSR;			// Clock Control and Status Register; Offset: 0x74
	uint32_t RESERVED6[2];			// Reserved; Offset: 0x78, 0x7C
	volatile uint32_t SSCGR;		// Spread Spectrum Clock Generation Register; Offset: 0x80
	volatile uint32_t PLLI2SCFGR;	// PLLI2S Configuration Register; Offset: 0x84
}RCC_RegDef_t;



/*
 * @description: GPIO Base addresses typecasted to GPIO_RegDef_t
 */
#define GPIOA						((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t *) GPIOI_BASEADDR)



/*
 * @description: RCC Base address typecasted to RCC_RegDef_t
 */
#define RCC							((RCC_RegDef_t *) RCC_BASEADDR)



/*
 * @description: Clock Enable Macros for Peripherals in AHB1 bus
 */
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))



/*
 * @description: Clock Enable Macros for Peripherals in APB1 bus
 */
// SPI
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
// I2C
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))
// USART
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
// UART
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))



/*
 * @description: Clock Enable Macros for Peripherals in APB2 bus
 */
// USART
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))
// SPI
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
//SYSCFG
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))



/*
 * @description: Clock Disable Macros for Peripherals in AHB1 bus
 */
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))



/*
 * @description: Clock Disable Macros for Peripherals in APB1 bus
 */
// SPI
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
// I2C
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))
// USART
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
// UART
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))



/*
 * @description: Clock Disable Macros for Peripherals in APB2 bus
 */
// USART
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))
// SPI
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
//SYSCFG
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F407XX_H_ */
