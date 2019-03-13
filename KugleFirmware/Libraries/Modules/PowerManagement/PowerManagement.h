/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef MODULES_POWERMANAGEMENT_H
#define MODULES_POWERMANAGEMENT_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for processing task

#include "IO.h"
#include "PWM.h"
#include "Battery.h"
#include "Parameters.h"

#define POWER_LED_PWM_FREQUENCY 200
#define POWER_LED_PWM_RANGE		100

class PowerManagement
{
	private:
		const uint32_t POWER_MANAGEMENT_THREAD_STACK = 256;

	public:
		typedef enum LEDmode_t {
			LEDMODE_ON,
			LEDMODE_OFF,
			LEDMODE_PULSING,
			LEDMODE_BLINKING,
			LEDMODE_BLINKING_FAST,
		} LEDmode_t;

		typedef enum PowerMode_t {
			POWERMODE_OFF = 0,
			POWERMODE_ALL_ON = 1,
			POWERMODE_5V_ONLY = 2
		} PowerMode_t;

	public:
		PowerManagement(IO& enable19V, IO& enable5V, Battery& bat1, Battery& bat2, PWM& powerLED, uint32_t powerManagementTaskPriority);
		~PowerManagement();

		void SetPowerMode(PowerMode_t powerMode);
		PowerMode_t GetPowerMode();
		void SetLEDmode(LEDmode_t ledMode);

	private:
		IO& _enable19V; // Devices on 19V bus: LiDAR, PC (Intel NUC or Jetson) and Motors
		IO& _enable5V;  // Devices on 5V bus: Xsens IMU, USB Hub
		Battery& _bat1;
		Battery& _bat2;
		PWM& _powerLED;

		PowerMode_t _powerMode;

		uint8_t _PulseValue;
		bool _PulseDirectionUp;
		LEDmode_t _ledMode;

		TaskHandle_t _powerManagementTaskHandle;


	private:
		static void PowerButtonInterrupt(void * params);
		static void ResetButtonInterrupt(void * params);
		static void CalibrateButtonInterrupt(void * params);
		static void PowerManagementThread(void * pvParameters);

		void Enable(bool enable19V, bool enable5V);
		void ButtonHandler();
};
	
	
#endif
