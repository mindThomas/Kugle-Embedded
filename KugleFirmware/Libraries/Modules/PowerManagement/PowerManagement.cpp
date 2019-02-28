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
 
#include "PowerManagement.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for processing task

#include "Debug.h"

PowerManagement::PowerManagement(IO& enable19V, IO& enable5V, Battery& bat1, Battery& bat2, PWM& powerLED, uint32_t powerManagementTaskPriority) : _powerManagementTaskHandle(0), _enable19V(enable19V), _enable5V(enable5V), _bat1(bat1), _bat2(bat2), _powerLED(powerLED)
{
	_ledMode = LEDMODE_PULSING;
	_PulseValue = 0;
	_PulseDirectionUp = true;

	_powerMode = POWERMODE_OFF;
	Enable(true, false); // start with 19V turned on and 5V off

	xTaskCreate(PowerManagement::PowerManagementThread, (char *)"Power Management", POWER_MANAGEMENT_THREAD_STACK, (void*) this, powerManagementTaskPriority, &_powerManagementTaskHandle);
}

PowerManagement::~PowerManagement()
{
	if (_powerManagementTaskHandle)
		vTaskDelete(_powerManagementTaskHandle); // stop task
}

void PowerManagement::Enable(bool enable19V, bool enable5V)
{
	if (enable19V)
		_enable19V.Set(true);
	else
		_enable19V.Set(false);

	if (enable5V)
		_enable5V.Set(true);
	else
		_enable5V.Set(false);
}

void PowerManagement::SetLEDmode(LEDmode_t ledMode)
{
	LEDmode_t prevLedMode = _ledMode;
	_ledMode = ledMode;

	/*if (_ledMode == ON) {
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
		//vTaskResume(_powerManagementTaskHandle);
	}*/
}

void PowerManagement::SetPowerMode(PowerManagement::PowerMode_t powerMode)
{
	_powerMode = powerMode;
	if (powerMode == POWERMODE_OFF)
		Enable(false, false);
	else if (powerMode == POWERMODE_ALL_ON)
		Enable(true, true);
	else if (powerMode == POWERMODE_5V_ONLY)
		Enable(false, true);
}

PowerManagement::PowerMode_t PowerManagement::GetPowerMode()
{
	return _powerMode;
}

#if 0
void PowerManagement::PowerButtonInterrupt(void * params)
{
	PowerManagement * pm = (PowerManagement *)params;

}

void PowerManagement::ResetButtonInterrupt(void * params)
{
	PowerManagement * pm = (PowerManagement *)params;

}

void PowerManagement::CalibrateButtonInterrupt(void * params)
{
	PowerManagement * pm = (PowerManagement *)params;

}
#endif

void PowerManagement::PowerManagementThread(void * pvParameters)
{
	PowerManagement * pm = (PowerManagement *)pvParameters;

	/* Consider to implement this as a high priority 'watchdog task' (referred to as a 'check' task in all the official demos) that monitors how the cycle counters of each task to ensure they are cycling as expected */
	/* This task could possibly also poll for the real time stats - https://www.freertos.org/a00021.html#vTaskGetRunTimeStats */

	while (1) {
		/*if (pm->_ledMode != PULSING)
			vTaskSuspend(NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)
		*/

		if (pm->_ledMode == PowerManagement::LEDMODE_PULSING) {
			if (pm->_PulseDirectionUp && pm->_PulseValue < POWER_LED_PWM_RANGE)
				pm->_PulseValue++;
			else if (!pm->_PulseDirectionUp && pm->_PulseValue > 0)
				pm->_PulseValue--;
			else
				pm->_PulseDirectionUp = !pm->_PulseDirectionUp;
			pm->_powerLED.SetRaw(pm->_PulseValue);
		}
		else if (pm->_ledMode == PowerManagement::LEDMODE_BLINKING) {
			pm->_PulseValue++;
			if (pm->_PulseValue > POWER_LED_PWM_RANGE)
				pm->_PulseValue = 0;

			if (pm->_PulseValue > POWER_LED_PWM_RANGE/2)
				pm->_powerLED.SetRaw(POWER_LED_PWM_RANGE);
			else
				pm->_powerLED.SetRaw(0);
		}
		else if (pm->_ledMode == PowerManagement::LEDMODE_BLINKING_FAST) {
			pm->_PulseValue++;
			if (pm->_PulseValue > POWER_LED_PWM_RANGE/2)
				pm->_PulseValue = 0;

			if (pm->_PulseValue > POWER_LED_PWM_RANGE/4)
				pm->_powerLED.SetRaw(POWER_LED_PWM_RANGE);
			else
				pm->_powerLED.SetRaw(0);
		}
		else if (pm->_ledMode == PowerManagement::LEDMODE_ON) {
			pm->_powerLED.SetRaw(POWER_LED_PWM_RANGE);
			pm->_PulseValue = POWER_LED_PWM_RANGE;
		}
		else if (pm->_ledMode == PowerManagement::LEDMODE_OFF) {
			pm->_powerLED.SetRaw(0);
			pm->_PulseValue = 0;
		}

		osDelay(6);
	}
}
