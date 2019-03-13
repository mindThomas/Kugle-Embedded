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
 
#ifndef MODULES_ESTIMATORS_WHEELSLIPDETECTOR_H
#define MODULES_ESTIMATORS_WHEELSLIPDETECTOR_H

#include "Parameters.h"
#include "Timer.h"

class WheelSlipDetector
{
	public:
		WheelSlipDetector(Parameters& params, Timer * microsTimer);
		WheelSlipDetector(Parameters& params);
		~WheelSlipDetector();

		void Reset();
		void Step(const float motorAngle[3]);
		void Step(const float motorAngle[3], const float slipAccelerationThreshold, const float slipTimeThreshold, const float slipIdleTime, const float dt);
		bool SlipDetected();
		bool SlipDetected(uint8_t wheelIndex);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		float _motorAngle_prev[3];
		float _dpsi[3];
		float _dpsi_prev[3];
		float _ddpsi[3];

		float _slipTime[3];
		bool _wheelSlipDetected[3];
};
	
	
#endif
