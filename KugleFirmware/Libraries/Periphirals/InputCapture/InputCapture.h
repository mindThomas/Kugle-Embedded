/* Copyright (C) 2018 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */
 
#ifndef PERIPHIRALS_INPUTCAPTURE_H
#define PERIPHIRALS_INPUTCAPTURE_H

#include "stm32h7xx_hal.h"

class InputCapture
{
	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER5,
			TIMER14
		} timer_t;

	public:
		InputCapture();
		~InputCapture();

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			TIM_HandleTypeDef handle;
			int32_t offsetValue;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER2;
		static hardware_resource_t * resTIMER3;
		static hardware_resource_t * resTIMER4;
	
	private:
		hardware_resource_t * _hRes;
};
	
	
#endif
