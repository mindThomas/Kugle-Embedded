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
 
#ifndef APPLICATION_PATHFOLLOWINGCONTROLLER_H
#define APPLICATION_PATHFOLLOWINGCONTROLLER_H

#include "cmsis_os.h"

class PathFollowingController
{
	private:
		const int THREAD_STACK_SIZE = 128;
		const uint32_t THREAD_PRIORITY = osPriorityNormal;

	public:
		PathFollowingController();	
		~PathFollowingController();

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
