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
 
#ifndef PERIPHIRALS_IO_H
#define PERIPHIRALS_IO_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for semaphore support

#define IO_INTERRUPT_PRIORITY	5

class IO
{

public:
	typedef enum interrupt_trigger_t {
		TRIGGER_RISING = 0,
		TRIGGER_FALLING,
		TRIGGER_BOTH
	} interrupt_trigger_t;

	typedef enum pull_t {
		PULL_NONE = 0,
		PULL_UP,
		PULL_DOWN,
	} pull_t;

public:
	IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, bool isInput, pull_t pull);
	IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, bool isInput);
	IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin); // default configure as output
	void RegisterInterrupt(interrupt_trigger_t trigger, SemaphoreHandle_t semaphore);
	void RegisterInterrupt(interrupt_trigger_t trigger, void (*InterruptCallback)(void * params), void * callbackParams);
	void Set(bool state);
	bool Read();
	void High();
	void Low();
	void Toggle();
	~IO();

public:
	void (*_InterruptCallback)(void * params);
	void * _InterruptCallbackParams;
	SemaphoreHandle_t _InterruptSemaphore;

	static IO * interruptObjects[16]; // we only have 16 interrupt lines

private:
	GPIO_TypeDef * _GPIO;
	uint32_t _pin;
	bool _isInput;
	pull_t _pull;

private:
	void ConfigureInterrupt(interrupt_trigger_t level);

public:
	static void InterruptHandler(IO * io);

};
	
	
#endif
