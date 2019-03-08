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
 
#ifndef MODULES_CONTROLLERS_VELOCITYLQR_H
#define MODULES_CONTROLLERS_VELOCITYLQR_H

#include <stddef.h>
#include <stdlib.h>

#include <arm_math.h>
#include "Parameters.h"
#include "Timer.h"
#include "FirstOrderLPF.h"

class VelocityLQR
{
	public:
		VelocityLQR(Parameters& params, Timer * microsTimer, float SamplePeriod);
		VelocityLQR(Parameters& params, float SamplePeriod);
		~VelocityLQR();

		void Reset();

		void Step(const float xy[2], const float q[4], const float dxy[2], const float dq[4], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float omega_heading, float q_ref_out[4], float omega_body_ref_out[3]);
		void Step(const float xy[2], const float q[4], const float dxy[2], const float dq[4], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float omega_heading, const float velocity_error_clamp, const float angular_velocity_clamp, const float acceleration_limit, const float MaxTilt, const bool VelocityIntegralEnabled, const bool PositionControlAtZeroVelocityReference, const float PositionControlAtZeroVelocityReference_MaximumKickinVelocity, const float StabilizationDetectionVelocity, const float angle_lpf_tau, const float omega_lpf_tau, const bool DoNotSetOmegaRef, const float * gainMatrix, const float dt, float q_ref_out[4], float omega_body_ref_out[3]);

		void GetFilteredVelocityReference(float velocity_reference[2]);
		void GetFilteredVelocityReference_Inertial(float velocity_reference_inertial[2]);
		void GetFilteredVelocityReference_Heading(float velocity_reference_heading[2]);

		bool UnitTest(void);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		FirstOrderLPF _omega_x_ref_filt;
		FirstOrderLPF _omega_y_ref_filt;

	private:
		float roll_ref;
		float pitch_ref;

		bool position_control_enabled;
		float position_reference[2]; // used when 'PositionControlAtZeroVelocityReference' is enabled
		float velocity_error_integral[2];

		/* For debugging purposes */
		float Velocity_Reference_Filtered_Inertial[2];
		float Velocity_Reference_Filtered_Heading[2];

		float VelocityIntegralInitializationTime;
		bool VelocityIntegralAfterPowerup;
};
	
	
#endif
