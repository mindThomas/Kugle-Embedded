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
	return xTaskCreate( Communication::Thread, (char *)"Communication", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);
}

int Communication::Stop(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running

	_shouldStop = true;
	osDelay(timeout);
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

