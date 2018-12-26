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
 
#include "VelocityEKF.h"
#include "VelocityEstimator.h"
#include "VelocityEstimator_initialize.h"
#include <string.h> // for memcpy

VelocityEKF::VelocityEKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

VelocityEKF::VelocityEKF(Parameters& params) : _params(params)
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
void VelocityEKF::Step(const int32_t encoderTicks[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float COMest[3])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaMicros(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	if (dt == 0) return; // no time has passed
	Step(encoderTicks, qEst, Cov_qEst, qDotEst, COMest, dt);
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
void VelocityEKF::Step(const int32_t encoderTicks[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float COMest[3], const float dt)
{
	float X_prev[2];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[2*2];
	memcpy(P_prev, P, sizeof(P_prev));

	float EncoderDiffMeas[3] = {
		(float)(encoderTicks[0] - _prevEncoderTicks[0]),
		(float)(encoderTicks[1] - _prevEncoderTicks[1]),
		(float)(encoderTicks[2] - _prevEncoderTicks[2])
	};

    VelocityEstimator(X_prev, P_prev,
      EncoderDiffMeas,
      qEst, Cov_qEst, qDotEst,
      dt,
      _params.model.i_gear, _params.model.EncoderTicksPrRev,
	  _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g,
	  COMest,
      1E-5, // Var_COM
      10.0f, // eta_qQEKF_velocity
      0.0f, // eta_dqQEKF_encoder
      X, P);

    _prevEncoderTicks[0] = encoderTicks[0];
    _prevEncoderTicks[1] = encoderTicks[1];
    _prevEncoderTicks[2] = encoderTicks[2];
}

/**
 * @brief 	Get estimated 2L velocity defined in inertial frame
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
	memcpy(Cov_dxy, P, sizeof(P));
}
