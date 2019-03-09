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
 
#include "VelocityEKF.h"
#include "VelocityEstimator.h"
#include "VelocityEstimator_initialize.h"
#include <string.h> // for memcpy
#include <cmath>

#include "MathLib.h" // for matrix symmetrization

VelocityEKF::VelocityEKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

VelocityEKF::VelocityEKF(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

VelocityEKF::~VelocityEKF()
{
}

void VelocityEKF::Reset()
{
	VelocityEstimator_initialize(_params.estimator.VelocityEstimator_P_init_diagonal, X, P);

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;

	_prevEncoderTicks[0] = 0;
	_prevEncoderTicks[1] = 0;
	_prevEncoderTicks[2] = 0;
}

void VelocityEKF::Reset(const int32_t encoderTicks[3])
{
	Reset();

	_prevEncoderTicks[0] = encoderTicks[0];
	_prevEncoderTicks[1] = encoderTicks[1];
	_prevEncoderTicks[2] = encoderTicks[2];
}

/**
 * @brief 	Estimate 2L velocity given measured encoder ticks and estimated quaternion, quaternion derivative and possibly COM
 * @param	encoderTicks[3]  	Input: encoder ticks (raw ticks)
 * @param	qEst[4]        		Input: estimated attitude quaternion
 * @param	Cov_qEst[4*4]       Input: covariance of quaternion estimate, output from QEKF
 * @param	qDotEst[4]       	Input: estimated quaternion derivative
 * @param	COMest[3]       	Input: estimated center of mass (COM)
 */
void VelocityEKF::Step(const int32_t encoderTicks[3], const float Accelerometer[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	if (_params.estimator.UseXsensIMU) // use MTI covariance
		Step(encoderTicks, _params.estimator.eta_encoder, Accelerometer, _params.estimator.cov_acc_mti, _params.estimator.eta_accelerometer, _params.estimator.var_acc_bias, qEst, Cov_qEst, qDotEst, _params.estimator.var_acceleration, dt);
	else
		Step(encoderTicks, _params.estimator.eta_encoder, Accelerometer, _params.estimator.cov_acc_mpu, _params.estimator.eta_accelerometer, _params.estimator.var_acc_bias, qEst, Cov_qEst, qDotEst, _params.estimator.var_acceleration, dt);
}

/**
 * @brief 	Estimate 2L velocity given measured encoder ticks and estimated quaternion, quaternion derivative and possibly COM
 * @param	encoderTicks[3]  	Input: encoder ticks (raw ticks)
 * @param	qEst[4]        		Input: estimated attitude quaternion
 * @param	Cov_qEst[4*4]       Input: covariance of quaternion estimate, output from QEKF
 * @param	qDotEst[4]       	Input: estimated quaternion derivative
 * @param	COMest[3]       	Input: estimated center of mass (COM)
 * @param	dt    			   	Input: time passed since last estimate
 */
void VelocityEKF::Step(const int32_t encoderTicks[3], const float eta_encoder, const float Accelerometer[3], const float Cov_Accelerometer[3*3], const float eta_accelerometer, const float eta_acc_bias, const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float eta_acceleration, const float dt)
{
	if (dt == 0) return; // no time has passed

	float X_prev[7];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[7*7];
	memcpy(P_prev, P, sizeof(P_prev));

	float EncoderDiffMeas[3] = {
		float(encoderTicks[0] - _prevEncoderTicks[0]),
		float(encoderTicks[1] - _prevEncoderTicks[1]),
		float(encoderTicks[2] - _prevEncoderTicks[2])
	};

	float q_dot[4] = { 0, 0, 0, 0 };
	if (_params.estimator.UseQdotInVelocityEstimator) {
		q_dot[0] = qDotEst[0];
		q_dot[1] = qDotEst[1];
		q_dot[2] = qDotEst[2];
		q_dot[3] = qDotEst[3];
	}

	VelocityEstimator(X_prev, P_prev,
	      EncoderDiffMeas, eta_encoder,
		  Accelerometer, Cov_Accelerometer, eta_accelerometer,
		  eta_acc_bias,
		  qEst, Cov_qEst, q_dot,
		  eta_acceleration,
	      dt,
	      _params.model.TicksPrRev,
		  _params.model.rk, _params.model.rw, _params.model.g,
	      X, P);

	Math_SymmetrizeSquareMatrix(P, sizeof(X)/sizeof(float));

    _prevEncoderTicks[0] = encoderTicks[0];
    _prevEncoderTicks[1] = encoderTicks[1];
    _prevEncoderTicks[2] = encoderTicks[2];
}

/**
 * @brief 	Get estimated velocity defined in inertial frame
 * @param	dxy[2]		Output: estimated velocity in inertial frame
 */
void VelocityEKF::GetVelocity(float dxy[2])
{
	dxy[0] = X[0];
	dxy[1] = X[1];
}

/**
 * @brief 	Get covariance matrix of estimated velocity
 * @param	Cov_dxy[2*2]		Output: velocity estimate covariance
 */
void VelocityEKF::GetVelocityCovariance(float Cov_dxy[2*2])
{
    for (int m = 0; m < 2; m++) {
      for (int n = 0; n < 2; n++) {
    	  Cov_dxy[2*m + n] = P[7*m + n];
      }
    }
}
