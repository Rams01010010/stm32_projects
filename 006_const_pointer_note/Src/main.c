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
	/*
	 * This is a `pointer` pointing a `const int` data
	 * which means this pointer is variable but the data
	 * it points to is const. And thats why compiler
	 * allows `x = &c` but not `*x = 'D'`. It throws
	 * error: assignment of read-only location '*x'

	 * Code
		const int b = 'B';
		const int c = 'C';
		const int *x = &b;
		*x = 'D';
		x = &c;
	*/


	/*
	 * This is a `const pointer` pointing a `int` data
	 * which means this pointer is constant but the data
	 * it points to is variable. And thats why compiler
	 * allows `*x = 'D'` but not `x = &c`. It throws
	 * error: assignment of read-only variable 'x'

	 * Code
		int b = 'B';
		int c = 'C';
		int *const x = &b;
		x = &c;
		*x = 'D';
	*/

	/*
	 * This is a `const pointer` pointing a `const int` data
	 * which means this pointer is constant, also the data
	 * it points to is constant. And thats why compiler
	 * doesnt allow both `x = &c` and `*x ='D'`. It throws
	 * error: assignment of read-only variable 'x'
	 * error: assignment of read-only location '*(const int *)x'

	 * Code
		const int b = 'B';
		const int c = 'C';
		const int *const x = &b;
		x = &c;
		*x = 'D';
	*/
}
