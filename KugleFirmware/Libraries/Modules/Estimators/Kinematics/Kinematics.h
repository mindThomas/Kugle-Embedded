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
 
#ifndef MODULES_ESTIMATORS_KINEMATICS_H
#define MODULES_ESTIMATORS_KINEMATICS_H

#include <stddef.h>
#include <stdlib.h>

#include "Parameters.h"

class Kinematics
{
	public:
		Kinematics(Parameters& params);	
		~Kinematics();

		void ForwardKinematics(const float dpsi[3], const float q[4], const float dq[4], float xy_velocity[2]);
		void ConvertBallTo2Lvelocity(float vel_ball[2], float q[4], float dq[4], float vel_2L[2]);
		void Convert2LtoBallVelocity(float vel_2L[2], float q[4], float dq[4], float vel_ball[2]);
		
	private:
		Parameters& _params;
};
	
	
#endif
