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
 
#ifndef MODULES_ESTIMATORS_VELOCITYEKF_H
#define MODULES_ESTIMATORS_VELOCITYEKF_H

#include "Parameters.h"

class VelocityEKF
{
	public:
		VelocityEKF(Parameters& params);
		~VelocityEKF();
	
		void Reset();
		void Step(const float dt, const float EncoderDiffMeas[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float COMest[3]);

		float X[2];
		float P[2*2];

	private:
		Parameters& _params;
};
	
	
#endif
