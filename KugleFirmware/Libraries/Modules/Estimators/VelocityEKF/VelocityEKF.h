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
 
#ifndef MODULES_ESTIMATORS_VELOCITYEKF_H
#define MODULES_ESTIMATORS_VELOCITYEKF_H

#include "Parameters.h"
#include "Timer.h"

class VelocityEKF
{
	public:
		VelocityEKF(Parameters& params);
		VelocityEKF(Parameters& params, Timer * microsTimer);
		~VelocityEKF();
	
		void Reset();
		void Reset(const int32_t encoderTicks[3]);
		void Step(const int32_t encoderTicks[3], const float Accelerometer[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4]);
		void Step(const int32_t encoderTicks[3], const float eta_encoder, const float Accelerometer[3], const float Cov_Accelerometer[3*3], const float eta_accelerometer, const float eta_acc_bias, const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float eta_acceleration, const float dt);

		void GetVelocity(float dxy[2]);
		void GetVelocityCovariance(float Cov_dxy[2*2]);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		int32_t _prevEncoderTicks[3];

		/* State estimate */
		float X[7];   // state estimates = { dx, dy, ddx, ddy, acc_bias_x, acc_bias_y, acc_bias_z }
		float P[7*7]; // covariance matrix
};
	
	
#endif
