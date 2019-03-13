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
 
#ifndef APPLICATION_COMMUNICATION_H
#define APPLICATION_COMMUNICATION_H

#include "cmsis_os.h"

class Communication
{
	private:
		const int THREAD_STACK_SIZE = 256;
		const uint32_t THREAD_PRIORITY = osPriorityNormal;

	public:
		Communication();	
		~Communication();

		int Start();
		int Stop(uint32_t timeout = 1000);
		int Restart(uint32_t timeout = 1000);

	private:
		static void Thread(void * pvParameters);

	private:
		TaskHandle_t _TaskHandle;
		bool _isRunning;
		bool _shouldStop;

};
	
	
#endif
