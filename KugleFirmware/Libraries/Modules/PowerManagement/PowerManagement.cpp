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
 
#include "PowerManagement.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for processing task

#include "Debug.h"
#include "IO.h"
#include "PWM.h"

PowerManagement::PowerManagement(uint32_t powerManagementTaskPriority) : _powerManagementTaskHandle(0)
{
	_enable19V = new IO(GPIOE, GPIO_PIN_4); // configure as output
	_enable5V = new IO(GPIOC, GPIO_PIN_3); // configure as output
	_bat1 = 0;
	_bat2 = 0;
	_powerButton = new IO(GPIOB, GPIO_PIN_6, IO::PULL_DOWN); // configure as input
	_powerLED = new PWM(PWM::TIMER17, PWM::CH1, POWER_LED_PWM_FREQUENCY, POWER_LED_PWM_RANGE);

	_powerButton->RegisterInterrupt(IO::TRIGGER_RISING, PowerManagement::PowerButtonInterrupt, this);

	_ledMode = PULSING;
	_PulseValue = 0;
	_PulseDirectionUp = true;
	xTaskCreate(PowerManagement::PowerManagementThread, (char *)"Power Management", POWER_MANAGEMENT_THREAD_STACK, (void*) this, powerManagementTaskPriority, &_powerManagementTaskHandle);
}

PowerManagement::~PowerManagement()
{
	if (_enable19V)
		delete(_enable19V);
	if (_enable5V)
		delete(_enable5V);
	if (_bat1)
		delete(_bat1);
	if (_bat2)
		delete(_bat2);
	if (_powerButton)
		delete(_powerButton);
	if (_powerLED)
		delete(_powerLED);

	if (_powerManagementTaskHandle)
		vTaskDelete(_powerManagementTaskHandle); // stop task
}

void PowerManagement::Enable(bool enable19V, bool enable5V)
{
	if (_enable19V) {
		if (enable19V)
			_enable19V->Set(true);
		else
			_enable19V->Set(false);
	}

	if (_enable5V) {
		if (enable5V)
			_enable5V->Set(true);
		else
			_enable5V->Set(false);
	}
}

void PowerManagement::SetLEDmode(LEDmode_t ledMode)
{
	LEDmode_t prevLedMode = _ledMode;
	_ledMode = ledMode;

	if (_ledMode == ON) {
		if (_powerLED)
			_powerLED->SetRaw(POWER_LED_PWM_RANGE);
	}
	else if (_ledMode == OFF) {
		if (_powerLED)
			_powerLED->SetRaw(0);
	}
	else if (_ledMode == PULSING) {
		if (prevLedMode == ON) {
			_PulseValue = POWER_LED_PWM_RANGE;
			_PulseDirectionUp = false; // down
		} else if (prevLedMode == OFF) {
			_PulseValue = POWER_LED_PWM_RANGE;
			_PulseDirectionUp = true; // up
		}
		vTaskResume(_powerManagementTaskHandle);
	}
}

void PowerManagement::PowerButtonInterrupt(void * params)
{
	//PowerManagement * pm = (PowerManagement *)params;

}

void PowerManagement::PowerManagementThread(void * pvParameters)
{
	PowerManagement * pm = (PowerManagement *)pvParameters;

	if (!pm->_powerLED) {
		while (1) {
			osDelay(10);
		}
	}

	while (1) {
		if (pm->_ledMode != PULSING)
			vTaskSuspend(NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)

		if (pm->_PulseDirectionUp && pm->_PulseValue < pm->POWER_LED_PWM_RANGE)
			pm->_PulseValue++;
		else if (!pm->_PulseDirectionUp && pm->_PulseValue > 0)
			pm->_PulseValue--;
		else
			pm->_PulseDirectionUp = !pm->_PulseDirectionUp;

		pm->_powerLED->SetRaw(pm->_PulseValue);

		osDelay(10);
	}
}
