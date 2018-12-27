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
 
#include "QEKF.h"
#include "QEKF_coder.h"
#include "QEKF_initialize.h"
#include <string.h> // for memcpy
 
QEKF::QEKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

QEKF::QEKF(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

QEKF::~QEKF()
{
}

void QEKF::Reset()
{
	QEKF_initialize(_params.estimator.QEKF_P_init_diagonal, X, P);

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;
}

/**
 * @brief 	Reset attitude estimator to an angle based on an accelerometer measurement
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 */
void QEKF::Reset(const float accelerometer[3])
{
	Reset();
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias)
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaMicros(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	Step(accelerometer, gyroscope, EstimateBias);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param	dt    			   Input: time passed since last estimate
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias, const float dt)
{
	if (dt == 0) return; // no time has passed

	float X_prev[10];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[10*10];
	memcpy(P_prev, P, sizeof(P_prev));

	_QEKF(X_prev, P_prev,
		 gyroscope, accelerometer,
		 dt,
		 _params.estimator.EstimateBias,
		 true,  // normalize accelerometer = true
		 _params.estimator.cov_gyro_mpu, _params.estimator.cov_acc_mpu, _params.estimator.sigma2_bias,
		 _params.model.g,
		 X, P);

    if (_params.estimator.CreateQdotFromQDifference) {
      X[4] = (X[0] - X_prev[0]) / dt; // dq[0]
      X[5] = (X[1] - X_prev[1]) / dt; // dq[1]
      X[6] = (X[2] - X_prev[2]) / dt; // dq[2]
      X[7] = (X[3] - X_prev[3]) / dt; // dq[3]
    }
}

/**
 * @brief 	Get estimated attitude quaternion
 * @param	q[4]		Output: estimated attitude quaternion
 */
void QEKF::GetQuaternion(float q[4])
{
	q[0] = X[0];
	q[1] = X[1];
	q[2] = X[2];
	q[3] = X[3];
}

/**
 * @brief 	Get estimated attitude quaternion derivative
 * @param	dq[4]		Output: estimated attitude quaternion derivative
 */
void QEKF::GetQuaternionDerivative(float dq[4])
{
	dq[0] = X[4];
	dq[1] = X[5];
	dq[2] = X[6];
	dq[3] = X[7];
}

/**
 * @brief 	Get covariance matrix of estimated quaternion
 * @param	Cov_q[4*4]		Output: quaternion estimate covariance
 */
void QEKF::GetQuaternionCovariance(float Cov_q[4*4])
{
    for (int m = 0; m < 4; m++) {
      for (int n = 0; n < 4; n++) {
        Cov_q[4*m + n] = P[10*m + n];
      }
    }
}

/**
 * @brief 	Get covariance matrix of estimated quaternion derivative
 * @param	Cov_dq[4*4]		Output: quaternion derivative estimate covariance
 */
void QEKF::GetQuaternionDerivativeCovariance(float Cov_dq[4*4])
{
    for (int m = 0; m < 4; m++) {
      for (int n = 0; n < 4; n++) {
        Cov_dq[4*m + n] = P[10*m + n + 44];
      }
    }
}
