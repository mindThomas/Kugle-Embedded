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
 
#include "HealthMonitor.h"
#include "cmsis_os.h"

HealthMonitor::HealthMonitor() : _TaskHandle(0), _isRunning(false), _shouldStop(false)
{
	Start();
}

HealthMonitor::~HealthMonitor()
{
	_shouldStop = true;
	while (_isRunning) osDelay(10);
}

int HealthMonitor::Start()
{
	if (_isRunning) return 0; // task already running
	return xTaskCreate( HealthMonitor::Thread, (char *)"Health Monitor", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);
}

int HealthMonitor::Stop(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running

	_shouldStop = true;
	osDelay(timeout);
	if (_isRunning) return -1; // timeout trying to stop task
	return 1;
}

int HealthMonitor::Restart(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running
	int errCode = Stop(timeout);
	if (errCode != 1) return errCode;
	return Start();
}

void HealthMonitor::Thread(void * pvParameters)
{
	HealthMonitor * task = (HealthMonitor *)pvParameters;
	task->_isRunning = true;

	while (!task->_shouldStop) {
		osDelay(1);
	}

	task->_isRunning = false;
	task->_TaskHandle = 0;
	vTaskDelete(NULL); // delete/stop this current task
}
