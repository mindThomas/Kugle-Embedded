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
 
#include "FrontPanel.h"
#include "cmsis_os.h"

FrontPanel::FrontPanel(PowerManagement& powerManagement_, BalanceController& balanceController_, IO * powerButton_, IO * resetButton_, IO * calibrateButton_) : _TaskHandle(0), _isRunning(false), _shouldStop(false), powerManagement(powerManagement_), balanceController(balanceController_)
{
	powerButton.btn = powerButton_;
	//powerButton.btn->RegisterInterrupt(IO::TRIGGER_RISING, PowerManagement::PowerButtonInterrupt, this);
	powerButton.callback = &PowerButtonPressed;
	powerButton.callbackParams = this;
	powerButton.debounceTime = DEBOUNCE_TIME;
	powerButton.debounceEndTime = 0;
	powerButton.prevState = powerButton.btn->Read();

	calibrateButton.btn = calibrateButton_;
	//calibrateButton.btn->RegisterInterrupt(IO::TRIGGER_RISING, PowerManagement::PowerButtonInterrupt, this);
	calibrateButton.callback = &CalibrateButtonPressed;
	calibrateButton.callbackParams = this;
	calibrateButton.debounceTime = 1.0f; // button should be held for 1 second to initiate
	calibrateButton.debounceEndTime = 0;
	calibrateButton.prevState = calibrateButton.btn->Read();

	resetButton.btn = resetButton_;
	//resetButton.btn->RegisterInterrupt(IO::TRIGGER_RISING, PowerManagement::PowerButtonInterrupt, this);
	resetButton.callback = &ResetButtonPressed;
	resetButton.callbackParams = this;
	resetButton.debounceTime = DEBOUNCE_TIME;
	resetButton.debounceEndTime = 0;
	resetButton.prevState = resetButton.btn->Read();

	_params = new Parameters;

	Start();
}

FrontPanel::~FrontPanel()
{
	_shouldStop = true;
	while (_isRunning) osDelay(10);

	if (_params)
		delete(_params);
}

int FrontPanel::Start()
{
	if (_isRunning) return 0; // task already running
	_shouldStop = false;
	return xTaskCreate( FrontPanel::Thread, (char *)"Front Panel", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);
}

int FrontPanel::Stop(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running

	_shouldStop = true;

	uint32_t timeout_millis = timeout;
	while (_isRunning && timeout_millis > 0) {
		osDelay(1);
		timeout_millis--;
	}
	if (_isRunning) return -1; // timeout trying to stop task

	return 1;
}

int FrontPanel::Restart(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running
	int errCode = Stop(timeout);
	if (errCode != 1) return errCode;
	return Start();
}

void FrontPanel::Thread(void * pvParameters)
{
	FrontPanel * fp = (FrontPanel *)pvParameters;
	fp->_isRunning = true;

	fp->powerManagement.SetPowerMode(PowerManagement::POWERMODE_ALL_ON);
	fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_ON);

	while (!fp->_shouldStop) {
		fp->ButtonHandler(fp->powerButton);
		fp->ButtonHandler(fp->calibrateButton);
		fp->ButtonHandler(fp->resetButton);
		osDelay(10);
	}

	fp->_isRunning = false;
	fp->_TaskHandle = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

void FrontPanel::ButtonHandler(button_t& button)
{
	if (button.btn->Read() != button.prevState) {
		if (button.debounceEndTime == 0) {
			button.debounceEndTime = xTaskGetTickCount() + (configTICK_RATE_HZ * button.debounceTime);
		} else {
			if (xTaskGetTickCount() > button.debounceEndTime) {
				button.debounceEndTime = 0;
				button.prevState = button.btn->Read();

				if (button.prevState) { // button pressed
					if (button.callback)
						button.callback(button.callbackParams);
					while (button.btn->Read()) osDelay(10); // wait until button release
				}
			}
		}
	} else {
		button.debounceEndTime = 0;
	}
}

void FrontPanel::PowerButtonPressed(void * params)
{
	FrontPanel * fp = (FrontPanel*)params;

	fp->_params->Refresh();

	if (fp->_params->behavioural.PowerButtonMode == lspc::ParameterTypes::POWER_OFF) {
		PowerManagement::PowerMode_t powerMode = fp->powerManagement.GetPowerMode();
		fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_PULSING);

		if (powerMode != PowerManagement::POWERMODE_OFF) {
			fp->balanceController.Stop(5000);
			fp->powerManagement.SetPowerMode(PowerManagement::POWERMODE_OFF);
			fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_OFF);
		}
		else {
			fp->balanceController.Start();
			fp->powerManagement.SetPowerMode(PowerManagement::POWERMODE_ALL_ON);
			fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_ON);
		}
	}

	else if (fp->_params->behavioural.PowerButtonMode == lspc::ParameterTypes::START_STOP_QUATERNION_CONTROL) {
		if (fp->_params->controller.mode == lspc::ParameterTypes::OFF) {
			fp->_params->LockForChange();
			fp->_params->controller.mode = lspc::ParameterTypes::QUATERNION_CONTROL;
			fp->_params->UnlockAfterChange();
		}
		else {
			fp->_params->LockForChange();
			fp->_params->controller.mode = lspc::ParameterTypes::OFF;
			fp->_params->UnlockAfterChange();
		}
	}

	else if (fp->_params->behavioural.PowerButtonMode == lspc::ParameterTypes::START_STOP_VELOCITY_CONTROL) {
		if (fp->_params->controller.mode == lspc::ParameterTypes::OFF) {
			fp->_params->LockForChange();
			fp->_params->controller.mode = lspc::ParameterTypes::VELOCITY_CONTROL;
			fp->_params->UnlockAfterChange();
		}
		else {
			fp->_params->LockForChange();
			fp->_params->controller.mode = lspc::ParameterTypes::OFF;
			fp->_params->UnlockAfterChange();
		}
	}
}

void FrontPanel::ResetButtonPressed(void * params)
{
	FrontPanel * fp = (FrontPanel*)params;
	NVIC_SystemReset();
}

void FrontPanel::CalibrateButtonPressed(void * params)
{
	FrontPanel * fp = (FrontPanel*)params;

	PowerManagement::PowerMode_t powerMode = fp->powerManagement.GetPowerMode();
	fp->_params->Refresh();

	if (fp->_params->controller.mode == lspc::ParameterTypes::OFF) {
		fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_BLINKING_FAST);

		fp->balanceController.CalibrateIMU();

		if (powerMode == PowerManagement::POWERMODE_OFF)
			fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_OFF);
		else
			fp->powerManagement.SetLEDmode(PowerManagement::LEDMODE_ON);
	}
}
