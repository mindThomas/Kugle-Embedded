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
 
#include "PID.h"

PID::PID(const float Kp, const float Ki, const float Kd, Timer * microsTimer) : Kp_(Kp), Ki_(Ki), Kd_(Kd), _microsTimer(microsTimer)
{
	Reset();
}

PID::PID(const float Kp, const float Ki, const float Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd), _microsTimer(0)
{
	Reset();
}

PID::~PID()
{
}

void PID::Reset(void)
{
	prev_error_ = 0;
	integral_ = 0;

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;
}

float PID::Step(const float state, const float ref)
{
	float dt;

	if (!_microsTimer) return 0; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	return Step(state, ref, dt);
}

float PID::Step(const float state, const float ref, const float dt)
{
	float error = state - ref;

	float derror = 0;
	if (dt > 0)
		derror = (error - prev_error_) / dt;

	integral_ += dt * error;

	float out = Kp_*error + Ki_*integral_ + Kd_*derror;
	return out;
}
