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
 
#ifndef MODULES_POWERMANAGEMENT_H
#define MODULES_POWERMANAGEMENT_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for processing task

#include "IO.h"
#include "PWM.h"
#include "Battery.h"

class PowerManagement
{
	private:
		const int POWER_LED_PWM_FREQUENCY = 200;
		const int POWER_LED_PWM_RANGE = 100;
		const uint32_t POWER_MANAGEMENT_THREAD_STACK = 128;

	public:
		typedef enum LEDmode_t {
			ON,
			OFF,
			PULSING
		} LEDmode_t;

	public:
		PowerManagement(uint32_t powerManagementTaskPriority);
		~PowerManagement();

		void Enable(bool enable19V, bool enable5V);
		void SetLEDmode(LEDmode_t);

	private:
		IO * _enable19V; // Devices on 19V bus: LiDAR, PC (Intel NUC or Jetson) and Motors
		IO * _enable5V;  // Devices on 5V bus: Xsens IMU, USB Hub
		Battery * _bat1;
		Battery * _bat2;

		IO * _powerButton;
		PWM * _powerLED;

		uint8_t _PulseValue;
		bool _PulseDirectionUp;
		LEDmode_t _ledMode;

		TaskHandle_t _powerManagementTaskHandle;


	private:
		static void PowerButtonInterrupt(void * params);
		static void PowerManagementThread(void * pvParameters);
};
	
	
#endif
