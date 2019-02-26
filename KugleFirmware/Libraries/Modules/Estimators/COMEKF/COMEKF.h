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
 
#ifndef MODULES_ESTIMATORS_COMEKF_H
#define MODULES_ESTIMATORS_COMEKF_H

#include "Parameters.h"
#include "Timer.h"

class COMEKF
{
	public:
		COMEKF(Parameters& params);
		COMEKF(Parameters& params, Timer * microsTimer);
		~COMEKF();

		void Reset();
		void Step(const float dxyEst[2], const float Cov_dxy[2*2], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4]);
		void Step(const float dxyEst[2], const float Cov_dxy[2*2], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float dt);

		void GetCOM(float COM[3]);
		void GetCOMCovariance(float Cov_COM[2*2]);
	
	private:
		Parameters& _params;
		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		float _prevVelocity[2];

		/* State estimate */
		float X[2];   // state estimates = { COM_X, COM_Y }
		float P[2*2]; // covariance matrix
};
	
	
#endif
