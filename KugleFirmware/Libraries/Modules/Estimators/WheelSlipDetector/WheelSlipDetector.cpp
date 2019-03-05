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
 
#include "WheelSlipDetector.h"
#include "MathLib.h"
#include <math.h>
#include <string.h> // for memcpy
 
#include "Quaternion.h"
#include "arm_math.h"

WheelSlipDetector::WheelSlipDetector(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

WheelSlipDetector::WheelSlipDetector(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

WheelSlipDetector::~WheelSlipDetector()
{
}

void WheelSlipDetector::Reset()
{
	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;

	_motorAngle_prev[0] = 0;
	_motorAngle_prev[1] = 0;
	_motorAngle_prev[2] = 0;
	_dpsi_prev[0] = 0;
	_dpsi_prev[1] = 0;
	_dpsi_prev[2] = 0;

	_slipTime[0] = 0;
	_slipTime[1] = 0;
	_slipTime[2] = 0;
	_wheelSlipDetected[0] = false;
	_wheelSlipDetected[1] = false;
	_wheelSlipDetected[2] = false;
}

void WheelSlipDetector::Step(const float motorAngle[3])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	Step(motorAngle, _params.estimator.WheelSlipAccelerationThreshold, _params.estimator.WheelSlipDetectionTime, _params.estimator.WheelSlipIdleTime, dt);
}

/**
 * @brief 	Run wheel slip detector
 * @param	motorAngle[3]   				Input: latest encoder angle measurement [rad]
 * @param	slipAccelerationThreshold       Input: acceleration threshold being detected as a slip [rad/s^2]
 * @param   slipTimeThreshold      			Input: amount of time which the slip acceleration should occur for a slip to be defined as "detected" [s]
 * @param   slipIdleTime	      			Input: amount of time which no slip should be detected before the wheel slip detection flag is removed [s]
 * @param	dt    			   				Input: time passed since last estimate
 */
void WheelSlipDetector::Step(const float motorAngle[3], const float slipAccelerationThreshold, const float slipTimeThreshold, const float slipIdleTime, const float dt)
{
	if (dt == 0) return; // no time has passed

	/* Compute motor/wheel velocity by numerical differentiation */
	_dpsi[0] = (motorAngle[0] - _motorAngle_prev[0]) / dt;
	_dpsi[1] = (motorAngle[1] - _motorAngle_prev[1]) / dt;
	_dpsi[2] = (motorAngle[2] - _motorAngle_prev[2]) / dt;

	_motorAngle_prev[0] = motorAngle[0];
	_motorAngle_prev[1] = motorAngle[1];
	_motorAngle_prev[2] = motorAngle[2];

	/* Compute wheel acceleration by numerical differentiation */
	_ddpsi[0] = (_dpsi[0] - _dpsi_prev[0]) / dt;
	_ddpsi[1] = (_dpsi[1] - _dpsi_prev[1]) / dt;
	_ddpsi[2] = (_dpsi[2] - _dpsi_prev[2]) / dt;

	_dpsi_prev[0] = _dpsi[0];
	_dpsi_prev[1] = _dpsi[1];
	_dpsi_prev[2] = _dpsi[2];

	bool SlipDetected = false;
	SlipDetected |= fabsf(_ddpsi[0]) > slipAccelerationThreshold;
	SlipDetected |= fabsf(_ddpsi[1]) > slipAccelerationThreshold;
	SlipDetected |= fabsf(_ddpsi[2]) > slipAccelerationThreshold;

	for (uint8_t i = 0; i < 3; i++) {
		if (fabsf(_ddpsi[i]) > slipAccelerationThreshold) {
			if (_slipTime[i] == 0) {
				_slipTime[i] += slipIdleTime-slipTimeThreshold;
			}
			else if (!_wheelSlipDetected[i]) {
				_slipTime[i] += dt;
			}
			else {
				_slipTime[i] = slipIdleTime;
			}
		}
		else if (_wheelSlipDetected[i]) {
			_slipTime[i] -= dt;
			if (_slipTime[i] < 0) _slipTime[i] = 0;
		}
		else {
			_slipTime[i] = 0;
		}

		if (_wheelSlipDetected[i] == false && _slipTime[i] >= slipIdleTime)
			_wheelSlipDetected[i] = true;
		else if (_wheelSlipDetected[i] == true && _slipTime[i] <= 0)
			_wheelSlipDetected[i] = false;
	}
}

bool WheelSlipDetector::SlipDetected()
{
	return (_wheelSlipDetected[0] || _wheelSlipDetected[1] || _wheelSlipDetected[2]);
}

bool WheelSlipDetector::SlipDetected(uint8_t wheelIndex)
{
	if (wheelIndex >= 3) return false;
	return _wheelSlipDetected[wheelIndex];
}
