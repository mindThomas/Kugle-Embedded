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
 
#include "COMEKF.h"
#include "COMEstimator.h"
#include "COMEstimator_initialize.h"
#include <string.h> // for memcpy
#include <cmath> // for fmin, fmax

COMEKF::COMEKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

COMEKF::COMEKF(Parameters& params) : _params(params)
{
	Reset();
}

COMEKF::~COMEKF()
{
}

void COMEKF::Reset()
{
	COMEstimator_initialize(_params.estimator.COMEstimator_P_init_diagonal, X, P);

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;

	_prevVelocity[0] = 0;
	_prevVelocity[1] = 0;
}

void COMEKF::Step(const float dxyEst[2], const float Cov_dxy[2*2], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaMicros(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	if (dt == 0) return; // no time has passed
	Step(dxyEst, Cov_dxy, qEst, Cov_qEst, qDotEst, dt);
}

/**
 * @brief 	Estimate center of mass given estimated velocity and attitude quaternion
 * @param	dxyEst[2]        	Input: estimated velocity in inertial frame
 * @param	Cov_dxy[2*2]        Input: covariance of velocity estimate, output from VelocityEKF
 * @param	qEst[4]        		Input: estimated attitude quaternion
 * @param	Cov_qEst[4*4]       Input: covariance of quaternion estimate, output from QEKF
 * @param	qDotEst[4]       	Input: estimated quaternion derivative
 * @param	dt    			   	Input: time passed since last estimate
 */
void COMEKF::Step(const float dxyEst[2], const float Cov_dxy[2*2], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float dt)
{
	float X_prev[2];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[2*2];
	memcpy(P_prev, P, sizeof(P_prev));

	float VelocityDiff[3] = {
		(float)(dxyEst[0] - _prevVelocity[0]),
		(float)(dxyEst[1] - _prevVelocity[1])
	};

    COMEstimator(X_prev, P_prev,
      qEst, Cov_qEst, qDotEst,
      dxyEst, VelocityDiff, Cov_dxy,
      dt,
      _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g,
      X, P);

    _prevVelocity[0] = dxyEst[0];
    _prevVelocity[1] = dxyEst[1];

    /* Limit/clamp COM estimate */
    X[0] = fmax(fmin(X[0], _params.estimator.MaxCOMDeviation), -_params.estimator.MaxCOMDeviation);
    X[1] = fmax(fmin(X[1], _params.estimator.MaxCOMDeviation), -_params.estimator.MaxCOMDeviation);
}

/**
 * @brief 	Get estimated center of mass defined in body frame
 * @param	COM[3]		Output: estimated COM
 */
void COMEKF::GetCOM(float COM[3])
{
	COM[0] = X[0];
	COM[1] = X[1];
	COM[2] = _params.model.l;
}

/**
 * @brief 	Get covariance matrix of estimated center of mass X and Y axis
 * @param	Cov_COM[2*2]		Output: COM estimate covariance
 */
void COMEKF::GetCOMCovariance(float Cov_COM[2*2])
{
	memcpy(Cov_COM, P, sizeof(P));
}
