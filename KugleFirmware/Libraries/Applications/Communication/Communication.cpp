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
 
#include "Communication.h"
#include "cmsis_os.h"

Communication::Communication() : _TaskHandle(0), _isRunning(false), _shouldStop(false)
{
	Start();
}

Communication::~Communication()
{
	_shouldStop = true;
	while (_isRunning) osDelay(10);
}

int Communication::Start()
{
	if (_isRunning) return 0; // task already running
	_shouldStop = false;
	return xTaskCreate( Communication::Thread, (char *)"Communication", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);
}

int Communication::Stop(uint32_t timeout)
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

int Communication::Restart(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running
	int errCode = Stop(timeout);
	if (errCode != 1) return errCode;
	return Start();
}

void Communication::Thread(void * pvParameters)
{
	Communication * task = (Communication *)pvParameters;
	task->_isRunning = true;

	while (!task->_shouldStop) {
		osDelay(1);
	}

	task->_isRunning = false;
	task->_TaskHandle = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

