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
 
#ifndef MODULES_ESTIMATORS_VELOCITYCOMEKF_H
#define MODULES_ESTIMATORS_VELOCITYCOMEKF_H

#include "Parameters.h"
#include "Timer.h"

class VelocityCOM_EKF
{
	public:
		VelocityCOM_EKF(Parameters& params);
		VelocityCOM_EKF(Parameters& params, Timer * microsTimer);
		~VelocityCOM_EKF();
	
		void Reset();
		void Reset(const int32_t encoderTicks[3]);
		void Step(const int32_t encoderTicks[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const bool EstimateCoRvelocity, const bool ControllerRunning);
		void Step(const int32_t encoderTicks[3], const bool UseTiltForPrediction, const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const bool estimateCOM, const float Var_COM, const float eta_encoder, const bool EstimateCoRvelocity, const bool EnableWheelSlipDetector, const float WheelSlipAccelerationThreshold, const float WheelSlipSetVelocityVariance, const float dt);

		void GetVelocity(float dxy[2]);
		void GetVelocityCovariance(float Cov_dxy[2*2]);
		void GetCOM(float COM[3]);

	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		int32_t _prevEncoderTicks[3];

		/* State estimate */
		float X[4];   // state estimates = { dx, dy, COM_X, COM_Y }
		float P[4*4]; // covariance matrix
};
	
	
#endif
