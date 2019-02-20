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
 
#ifndef APPLICATION_FRONTPANEL_H
#define APPLICATION_FRONTPANEL_H

#include "cmsis_os.h"
#include "Parameters.h"
#include "PowerManagement.h"
#include "BalanceController.h"
#include "IO.h"

class FrontPanel
{
	private:
		const int THREAD_STACK_SIZE = 512;
		const uint32_t THREAD_PRIORITY = osPriorityNormal;
		const float DEBOUNCE_TIME = 0.08; // 80 ms

		typedef struct button_t {
			IO * btn;
			void (*callback)(void * params);
			void * callbackParams;
			bool prevState;
			float debounceTime; // can also be used as hold time
			uint32_t debounceEndTime;
		};

	public:
		FrontPanel(PowerManagement& pm_, BalanceController& bc_, IO * powerButton_, IO * resetButton_, IO * calibrateButton_);
		~FrontPanel();

		int Start();
		int Stop(uint32_t timeout = 1000);
		int Restart(uint32_t timeout = 1000);

	private:
		void ButtonHandler(button_t& button);

	private:
		static void Thread(void * pvParameters);
		static void PowerButtonPressed(void * params);
		static void ResetButtonPressed(void * params);
		static void CalibrateButtonPressed(void * params);

	private:
		TaskHandle_t _TaskHandle;
		bool _isRunning;
		bool _shouldStop;

		Parameters * _params;

		PowerManagement& powerManagement;
		BalanceController& balanceController;

		button_t powerButton;
		button_t resetButton;
		button_t calibrateButton;
};
	
	
#endif
