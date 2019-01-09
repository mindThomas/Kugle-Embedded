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
 
#ifndef MODULES_CONTROLLERS_QUATERNIONVELOCITYCONTROL_H
#define MODULES_CONTROLLERS_QUATERNIONVELOCITYCONTROL_H

#include <stddef.h>
#include <stdlib.h>

#include "Parameters.h"
#include "Timer.h"
#include "FirstOrderLPF.h"

class QuaternionVelocityControl
{
	public:
		QuaternionVelocityControl(Parameters& params, Timer * microsTimer, float SamplePeriod, float ReferenceSmoothingTau = 0.1);
		QuaternionVelocityControl(Parameters& params, float SamplePeriod, float ReferenceSmoothingTau = 0.1);
		~QuaternionVelocityControl();

		void Reset();
		void Step(const float q[4], const float dq[4], const float dxy[2], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, float q_ref_out[4]);
		void Step(const float q[4], const float dq[4], const float dxy[2], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float dt, float q_ref_out[4]);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint16_t _prevTimerValue;

		FirstOrderLPF _dx_ref_filt;
		FirstOrderLPF _dy_ref_filt;

	private:
		float q_tilt_integral[4];
};
	
	
#endif
