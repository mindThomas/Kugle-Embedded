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
 
#ifndef PERIPHIRALS_TIMER_H
#define PERIPHIRALS_TIMER_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for semaphore support

#define TIMER_INTERRUPT_PRIORITY		5

class Timer
{
	private:
		const uint16_t TIMER_DEFAULT_MAXVALUE = 0xFFFF;

	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER6,
			TIMER7,
			TIMER12,
			TIMER13
		} timer_t;

	public:
		Timer(timer_t timer, uint32_t frequency); // frequency defines the timer count frequency
		~Timer();

		void ConfigureTimerPeripheral();
		void RegisterInterruptSoft(uint32_t frequency, void (*TimerCallbackSoft)());
		void RegisterInterrupt(uint32_t frequency, void (*TimerCallback)());
		void RegisterInterrupt(uint32_t frequency, SemaphoreHandle_t semaphore);
		void SetMaxValue(uint16_t maxValue);

		uint32_t Get();
		void Reset();

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			uint32_t frequency;
			uint16_t maxValue;
			TIM_HandleTypeDef handle;
			TaskHandle_t callbackTaskHandle;
			void (*TimerCallback)();
			SemaphoreHandle_t callbackSemaphore;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER6;
		static hardware_resource_t * resTIMER7;
		static hardware_resource_t * resTIMER12;
		static hardware_resource_t * resTIMER13;

		void (*_TimerCallbackSoft)();

	private:
		hardware_resource_t * _hRes;

	public:
		static void InterruptHandler(Timer::hardware_resource_t * timer);
		static void CallbackThread(void * pvParameters);

};
	
	
#endif
