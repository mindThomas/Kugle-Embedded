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
 
#ifndef MODULES_ESTIMATORS_QEKF_H
#define MODULES_ESTIMATORS_QEKF_H

#include "Parameters.h"
#include "Timer.h"

class QEKF
{
	public:
		QEKF(Parameters& params, Timer * microsTimer);
		QEKF(Parameters& params);
		~QEKF();

		void Reset();
		void Reset(const float accelerometer[3]);
		void Step(const float accelerometer[3], const float gyroscope[3]);
		void Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias);
		void Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias, const float dt);
		void Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias, const float cov_acc[9], const float cov_gyro[9], const float sigma2_bias, const float g, const float dt);

		void GetQuaternion(float q[4]);
		void GetQuaternionDerivative(float dq[4]);
		void GetQuaternionCovariance(float Cov_q[4*4]);
		void GetQuaternionDerivativeCovariance(float Cov_dq[4*4]);

		bool UnitTest(void);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint16_t _prevTimerValue;

		/* State estimate */
		float X[10];    // state estimates = { q[0], q[1], q[2], q[3], dq[0], dq[1], dq[2], dq[3], gyro_bias[0], gyro_bias[1] }
		float P[10*10]; // covariance matrix
};
	
	
#endif
