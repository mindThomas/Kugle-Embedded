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
 
#include "Kinematics.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h> 
 
#include "Parameters.h"

#include "MATLABCoder/ForwardKinematics.h"
#include "Velocity2L.h"
 
Kinematics::Kinematics(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

Kinematics::Kinematics(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

Kinematics::~Kinematics()
{
}

void Kinematics::Reset()
{
	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;

	_prevEncoderTicks[0] = 0;
	_prevEncoderTicks[1] = 0;
	_prevEncoderTicks[2] = 0;

	_prevMotorAngle[0] = 0;
	_prevMotorAngle[1] = 0;
	_prevMotorAngle[2] = 0;

	_dpsi[0] = 0;
	_dpsi[1] = 0;
	_dpsi[2] = 0;
}

void Kinematics::Reset(const int32_t encoderTicks[3])
{
	Reset();

	_prevEncoderTicks[0] = encoderTicks[0];
	_prevEncoderTicks[1] = encoderTicks[1];
	_prevEncoderTicks[2] = encoderTicks[2];

	float EncoderConversionRatio = 2.f * M_PI / _params.model.TicksPrRev;

	_prevMotorAngle[0] = EncoderConversionRatio * _prevEncoderTicks[0];
	_prevMotorAngle[1] = EncoderConversionRatio * _prevEncoderTicks[1];
	_prevMotorAngle[2] = EncoderConversionRatio * _prevEncoderTicks[2];
}

void Kinematics::Reset(const float motorAngle[3])
{
	Reset();

	_prevMotorAngle[0] = motorAngle[0];
	_prevMotorAngle[1] = motorAngle[1];
	_prevMotorAngle[2] = motorAngle[2];

	float EncoderConversionRatio = 2.f * M_PI / _params.model.TicksPrRev;

	_prevEncoderTicks[0] = _prevMotorAngle[0] / EncoderConversionRatio;
	_prevEncoderTicks[1] = _prevMotorAngle[1] / EncoderConversionRatio;
	_prevEncoderTicks[2] = _prevMotorAngle[2] / EncoderConversionRatio;
}

void Kinematics::EstimateMotorVelocity(const float motorAngle[3])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaMicros(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	EstimateMotorVelocity(motorAngle, dt);
}

void Kinematics::EstimateMotorVelocity(const float motorAngle[3], const float dt)
{
	if (dt == 0) return;

	_dpsi[0] = (motorAngle[0] - _prevMotorAngle[0]) / dt;
	_dpsi[1] = (motorAngle[1] - _prevMotorAngle[1]) / dt;
	_dpsi[2] = (motorAngle[2] - _prevMotorAngle[2]) / dt;

	_prevMotorAngle[0] = motorAngle[0];
	_prevMotorAngle[1] = motorAngle[1];
	_prevMotorAngle[2] = motorAngle[2];
}

void Kinematics::EstimateMotorVelocity(const int32_t encoderTicks[3])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaMicros(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	EstimateMotorVelocity(encoderTicks, dt);
}

void Kinematics::EstimateMotorVelocity(const int32_t encoderTicks[3], const float dt)
{
	if (dt == 0) return;

	float EncoderDiffMeas[3] = {
		(float)(encoderTicks[0] - _prevEncoderTicks[0]),
		(float)(encoderTicks[1] - _prevEncoderTicks[1]),
		(float)(encoderTicks[2] - _prevEncoderTicks[2])
	};

	float EncoderConversionRatio = 2.f * M_PI / _params.model.TicksPrRev;
	_dpsi[0] = EncoderConversionRatio * EncoderDiffMeas[0];
	_dpsi[1] = EncoderConversionRatio * EncoderDiffMeas[1];
	_dpsi[2] = EncoderConversionRatio * EncoderDiffMeas[2];

    _prevEncoderTicks[0] = encoderTicks[0];
    _prevEncoderTicks[1] = encoderTicks[1];
    _prevEncoderTicks[2] = encoderTicks[2];
}

void Kinematics::ForwardKinematics(const float q[4], const float dq[4], float xy_velocity[2])
{
	_ForwardKinematics(_dpsi, q, dq, _params.model.rk, _params.model.rw, xy_velocity);
}


void Kinematics::ForwardKinematics(const float dpsi[3], const float q[4], const float dq[4], float xy_velocity[2])
{
	_ForwardKinematics(dpsi, q, dq, _params.model.rk, _params.model.rw, xy_velocity);
}

// Supports overwriting, hence it is OK for the output array, 'vel_2L', to point to the array as the input array, 'vel_ball'
void Kinematics::ConvertBallTo2Lvelocity(const float vel_ball[2], const float q[4], const float dq[4], float vel_2L[2])
{
	_BallTo2Lvelocity(vel_ball, q, dq, _params.model.l, vel_2L);
}

// Supports overwriting, hence it is OK for the output array, 'vel_ball', to point to the array as the input array, 'vel_2L'
void Kinematics::Convert2LtoBallVelocity(const float vel_2L[2], const float q[4], const float dq[4], float vel_ball[2])
{
	_Convert2LtoBallVelocity(vel_2L, q, dq, _params.model.l, vel_ball);
}
