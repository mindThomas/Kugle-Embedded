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
 
#include "Kinematics.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h> 
 
#include "Parameters.h"

#include "ForwardKinematics.h"
#include "Velocity2L.h"
 
Kinematics::Kinematics(Parameters& params) : _params(params)
{
}

Kinematics::~Kinematics()
{
}

void Kinematics::ForwardKinematics(const float dpsi[3], const float q[4], const float dq[4], float xy_velocity[2])
{
	_ForwardKinematics(dpsi, q, dq, _params.model.rk, _params.model.rw, xy_velocity);
}

void Kinematics::ConvertBallTo2Lvelocity(float vel_ball[2], float q[4], float dq[4], float vel_2L[2])
{
	_BallTo2Lvelocity(vel_ball, q, dq, _params.model.l, vel_2L);
}

void Kinematics::Convert2LtoBallVelocity(float vel_2L[2], float q[4], float dq[4], float vel_ball[2])
{
	_Convert2LtoBallVelocity(vel_2L, q, dq, _params.model.l, vel_ball);
}
