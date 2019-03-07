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
 
#include "VelocityCOM_EKF.h"
#include "VelocityEstimator_WithCOM.h"
#include "VelocityEstimator_WithCOM_initialize.h"
#include <string.h> // for memcpy
#include <cmath>

#include "MathLib.h" // for matrix symmetrization

VelocityCOM_EKF::VelocityCOM_EKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

VelocityCOM_EKF::VelocityCOM_EKF(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

VelocityCOM_EKF::~VelocityCOM_EKF()
{
}

void VelocityCOM_EKF::Reset()
{
	VelocityEstimator_WithCOM_initialize(_params.estimator.VelocityEstimator_WithCOM_P_init_diagonal, X, P);

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;

	_prevEncoderTicks[0] = 0;
	_prevEncoderTicks[1] = 0;
	_prevEncoderTicks[2] = 0;

	// Initialize COM estimate
	X[2] = _params.model.COM_X;
	X[3] = _params.model.COM_Y;
}

void VelocityCOM_EKF::Reset(const int32_t encoderTicks[3])
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
void VelocityCOM_EKF::Step(const int32_t encoderTicks[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const bool EstimateCoRvelocity, const bool ControllerRunning)
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	if (!ControllerRunning) {
		// Reset COM estimate
		X[2] = 0;
		X[3] = 0;
	}

	Step(encoderTicks, _params.estimator.UseTiltForVelocityPrediction, qEst, Cov_qEst, qDotEst, (ControllerRunning && _params.estimator.UseCOMestimateInVelocityEstimator), _params.estimator.Var_COM, _params.estimator.eta_encoder, EstimateCoRvelocity, (_params.estimator.EnableWheelSlipDetector && _params.estimator.UseWheelSlipDetectorInVelocityEstimator), _params.estimator.WheelSlipAccelerationThreshold, _params.estimator.VelocityEstimatorWheelSlipCovariance, dt);
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
void VelocityCOM_EKF::Step(const int32_t encoderTicks[3], const bool UseTiltForPrediction, const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const bool estimateCOM, const float Var_COM, const float eta_encoder, const bool EstimateCoRvelocity, const bool EnableWheelSlipDetector, const float WheelSlipAccelerationThreshold, const float WheelSlipSetVelocityVariance, const float dt)
{
	if (dt == 0) return; // no time has passed

	float X_prev[4];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[4*4];
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

	VelocityEstimator_WithCOM(X_prev, P_prev,
	      EncoderDiffMeas,
		  qEst, Cov_qEst, q_dot,
	      dt,
	      _params.model.TicksPrRev,
		  _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jw, _params.model.rw, _params.model.l, _params.model.g,
		  _params.model.CoR,
	      Var_COM,
	      eta_encoder,
		  UseTiltForPrediction, EstimateCoRvelocity,
		  EnableWheelSlipDetector, WheelSlipAccelerationThreshold, WheelSlipSetVelocityVariance,
		  estimateCOM,
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
void VelocityCOM_EKF::GetVelocity(float dxy[2])
{
	dxy[0] = X[0];
	dxy[1] = X[1];
}

/**
 * @brief 	Get estimated Center of Mass
 * @param	dxy[2]		Output: estimated velocity in inertial frame
 */
void VelocityCOM_EKF::GetCOM(float COM[3])
{
	COM[0] = X[2];
	COM[1] = X[3];
	COM[2] = _params.model.l;
}

/**
 * @brief 	Get covariance matrix of estimated velocity
 * @param	Cov_dxy[2*2]		Output: velocity estimate covariance
 */
void VelocityCOM_EKF::GetVelocityCovariance(float Cov_dxy[2*2])
{
    for (int m = 0; m < 2; m++) {
      for (int n = 0; n < 2; n++) {
    	  Cov_dxy[2*m + n] = P[4*m + n];
      }
    }
}
