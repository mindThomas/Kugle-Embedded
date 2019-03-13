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
 
#ifndef MODULES_CONTROLLERS_QUATERNIONVELOCITYCONTROL_H
#define MODULES_CONTROLLERS_QUATERNIONVELOCITYCONTROL_H

#include <stddef.h>
#include <stdlib.h>

#include "Parameters.h"
#include "Timer.h"
#include "FirstOrderLPF.h"
#include "FirstOrderHPF.h"

class QuaternionVelocityControl
{
	public:
		QuaternionVelocityControl(Parameters& params, Timer * microsTimer, float SamplePeriod);
		QuaternionVelocityControl(Parameters& params, float SamplePeriod);
		~QuaternionVelocityControl();

		void Reset();
		void Step(const float q[4], const float dq[4], const float dxy[2], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, float q_ref_out[4]);
		void Step(const float q[4], const float dq[4], const float dxy[2], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float acceleration_limit, const float dt, float q_ref_out[4]);

		void StepWithOmega(const float q[4], const float dq[4], const float dxy[2], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, float q_ref_out[4], float omega_body_ref_out[3]);
		void StepWithOmega(const float q[4], const float dq[4], const float dxy[2], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float acceleration_limit, const float angle_lpf_tau, const float omega_lpf_tau, const bool DoNotSetOmegaRef, const float dt, float q_ref_out[4], float omega_body_ref_out[3]);

		void GetIntegral(float q_integral[4]);
		void GetFilteredVelocityReference(float velocity_reference[2]);
		void GetFilteredVelocityReference_Heading(float velocity_reference_heading[2]);
		void GetFilteredVelocityReference_Inertial(float velocity_reference_inertial[2]);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		FirstOrderLPF _roll_ref_filt;
		FirstOrderLPF _pitch_ref_filt;

		FirstOrderLPF _omega_x_ref_filt;
		FirstOrderLPF _omega_y_ref_filt;

	private:
		float q_tilt_integral[4];
		float VelocityRef_RateLimited[2];
		float Velocity_Reference_Filtered[2];

		float Velocity_Heading_Integral[2];

		float roll_ref_old;
		float pitch_ref_old;

		/* For debugging purposes */
		float Velocity_Reference_Filtered_Inertial[2];
		float Velocity_Reference_Filtered_Heading[2];
};
	
	
#endif
