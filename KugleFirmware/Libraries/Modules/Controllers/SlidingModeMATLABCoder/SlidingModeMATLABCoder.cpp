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
 
#include "SlidingModeMATLABCoder.h"
#include "SlidingModeControl.h"


SlidingModeMATLABCoder::SlidingModeMATLABCoder(Parameters& params) : _params(params)
{
}

SlidingModeMATLABCoder::~SlidingModeMATLABCoder()
{
}

/**
 * @brief 	Compute control output with Sliding mode controller given a quaternion attitude reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 * @param	S[3]      	  Output: sliding manifold values for the three surfaces used for the attitude control
 */
void SlidingModeMATLABCoder::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], float tau[3], float S[3])
{
	float X[12] = {xy[0], xy[1], q[0], q[1], q[2], q[3], dxy[0], dxy[1], dq[0], dq[1], dq[2], dq[3]};

	SlidingModeControl(X, q_ref,
		_params.model.Jk, _params.model.Mk, _params.model.rk,
		_params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw,
		_params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb,
		_params.model.l, _params.model.g, _params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z,
		_params.controller.K, _params.controller.eta, _params.controller.epsilon, _params.controller.ContinousSwitching,
		tau, S);
}
