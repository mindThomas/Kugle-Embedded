/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef PROCESSOR_INIT_H
#define PROCESSOR_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define BOOTLOADER_MAGIC_ADDR ((uint32_t*) ((uint32_t) 0x2001FFF0))
#define BOOTLOADER_MAGIC_TOKEN 0xDEADBEEF

//Value taken from CD00167594.pdf page 35, system memory start.
#define BOOTLOADER_START_ADDR 0x1fffc400 //for ST32F042

#define A_VALUE 0x12345678

void SystemClock_Config(void);	
void Enter_DFU_Bootloader(void);
	
#ifdef __cplusplus
}
#endif

#endif 
