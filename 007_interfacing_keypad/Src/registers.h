/*
 * registers.h
 *
 *  Created on: Sep 8, 2024
 *      Author: rams
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

#include<stdint.h>

typedef struct
{
	uint32_t gpioa_en:1;
	uint32_t gpiob_en:1;
	uint32_t gpioc_en:1;
	uint32_t gpiod_en:1;
	uint32_t gpioe_en:1;
	uint32_t gpiof_en:1;
	uint32_t gpiog_en:1;
	uint32_t gpioh_en:1;
	uint32_t gpioi_en:1;
	uint32_t reserved1:3;
	uint32_t crc_en:1;
	uint32_t reserved2:5;
	uint32_t bkpsram_en:1;
	uint32_t reserved3:1;
	uint32_t ccmdataram_en:1;
	uint32_t dma1_en:1;
	uint32_t dma2_en:1;
	uint32_t reserved4:2;
	uint32_t ethmac_en:1;
	uint32_t ethmactx_en:1;
	uint32_t ethmacrx_en:1;
	uint32_t ethmacptp_en:1;
	uint32_t otghs_en:1;
	uint32_t otghsulpi_en:1;
	uint32_t reserved5:1;
}RCC_AHB1ENR_t;

typedef struct
{
	uint32_t pin0:2;
	uint32_t pin1:2;
	uint32_t pin2:2;
	uint32_t pin3:2;
	uint32_t pin4:2;
	uint32_t pin5:2;
	uint32_t pin6:2;
	uint32_t pin7:2;
	uint32_t pin8:2;
	uint32_t pin9:2;
	uint32_t pin10:2;
	uint32_t pin11:2;
	uint32_t pin12:2;
	uint32_t pin13:2;
	uint32_t pin14:2;
	uint32_t pin15:2;
}GPIOx_MODER;

typedef struct
{
	uint32_t pin0:2;
	uint32_t pin1:2;
	uint32_t pin2:2;
	uint32_t pin3:2;
	uint32_t pin4:2;
	uint32_t pin5:2;
	uint32_t pin6:2;
	uint32_t pin7:2;
	uint32_t pin8:2;
	uint32_t pin9:2;
	uint32_t pin10:2;
	uint32_t pin11:2;
	uint32_t pin12:2;
	uint32_t pin13:2;
	uint32_t pin14:2;
	uint32_t pin15:2;
}GPIOx_PUPDR;

typedef struct
{
	uint32_t pin0:1;
	uint32_t pin1:1;
	uint32_t pin2:1;
	uint32_t pin3:1;
	uint32_t pin4:1;
	uint32_t pin5:1;
	uint32_t pin6:1;
	uint32_t pin7:1;
	uint32_t pin8:1;
	uint32_t pin9:1;
	uint32_t pin10:1;
	uint32_t pin11:1;
	uint32_t pin12:1;
	uint32_t pin13:1;
	uint32_t pin14:1;
	uint32_t pin15:1;
	uint32_t reserved:16;
}GPIOx_IDR;

typedef struct
{
	uint32_t pin0:1;
	uint32_t pin1:1;
	uint32_t pin2:1;
	uint32_t pin3:1;
	uint32_t pin4:1;
	uint32_t pin5:1;
	uint32_t pin6:1;
	uint32_t pin7:1;
	uint32_t pin8:1;
	uint32_t pin9:1;
	uint32_t pin10:1;
	uint32_t pin11:1;
	uint32_t pin12:1;
	uint32_t pin13:1;
	uint32_t pin14:1;
	uint32_t pin15:1;
	uint32_t reserved:16;
}GPIOx_ODR;

#endif /* REGISTERS_H_ */
