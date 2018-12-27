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
 
#ifndef MODULES_ESTIMATORS_KINEMATICS_H
#define MODULES_ESTIMATORS_KINEMATICS_H

#include "Parameters.h"
#include "Timer.h"

class Kinematics
{
	private:
		const double M_PI = 3.14159265358979323846264338327950288;

	public:
		Kinematics(Parameters& params, Timer * microsTimer);
		Kinematics(Parameters& params);
		~Kinematics();

		void Reset();
		void Reset(const int32_t encoderTicks[3]);
		void Reset(const float motorAngle[3]);

		void EstimateMotorVelocity(const float motorAngle[3]);
		void EstimateMotorVelocity(const float motorAngle[3], const float dt);
		void EstimateMotorVelocity(const int32_t encoderTicks[3]);
		void EstimateMotorVelocity(const int32_t encoderTicks[3], const float dt);
		void ForwardKinematics(const float q[4], const float dq[4], float xy_velocity[2]);
		void ForwardKinematics(const float dpsi[3], const float q[4], const float dq[4], float xy_velocity[2]);
		void ConvertBallTo2Lvelocity(const float vel_ball[2], const float q[4], const float dq[4], float vel_2L[2]);
		void Convert2LtoBallVelocity(const float vel_2L[2], const float q[4], const float dq[4], float vel_ball[2]);
		
	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint16_t _prevTimerValue;

		float _dpsi[3];
		int32_t _prevEncoderTicks[3];
		float _prevMotorAngle[3];
};
	
	
#endif
