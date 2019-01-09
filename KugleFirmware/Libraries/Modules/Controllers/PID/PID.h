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
 
#ifndef MODULES_CONTROLLERS_PID_H
#define MODULES_CONTROLLERS_PID_H

#include <stddef.h>
#include <stdlib.h>

#include "Timer.h"

class PID
{
	public:
		PID(const float Kp, const float Ki, const float Kd, Timer * microsTimer);
		PID(const float Kp, const float Ki = 0.0f, const float Kd = 0.0f);
		~PID();

		void Reset(void);
		float Step(const float state, const float ref);
		float Step(const float state, const float ref, const float dt);

	private:
		Timer * _microsTimer;
		uint16_t _prevTimerValue;

		float Kp_;
		float Ki_;
		float Kd_;

		float prev_error_;
		float integral_;
};
	
	
#endif
