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
 
#ifndef MODULES_CONTROLLERS_SLIDINGMODE_H
#define MODULES_CONTROLLERS_SLIDINGMODE_H

#include <stddef.h>
#include <stdlib.h>

#include "Parameters.h"

class SlidingMode
{
	public:
		SlidingMode(Parameters& params);
		~SlidingMode();

		void Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], float tau[3], float S[3]);
		void Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float COM[3], const float q_ref[4], const float omega_ref[3], float tau[3], float S[3]);
		void Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref_in[4], const float omega_ref[3], const float Jk, const float Mk, const float rk, const float Mb, const float Jbx, const float Jby, const float Jbz, const float Jw, const float rw, const float Bvk, const float Bvm, const float Bvb, const float l, const float g_const, const float COM_X, const float COM_Y, const float COM_Z, const float K[3], const float eta[3], const float epsilon[3], const bool continuousSwitching, const bool IncludeEquivalentControl, const bool BodyFrame, float tau[3], float S[3]);
		void HeadingIndependentReferenceManual(const float q_ref[4], const float q[4], float q_ref_out[4]);
		void HeadingIndependentQdot(const float dq[4], const float q[4], float q_dot_out[4]);

		bool UnitTest(void);

	private:
		void Saturation(const float * in, const int size, const float * epsilon, float * out);
		void Sign(float * in, int size, float * out);

	private:
		Parameters& _params;
};
	
	
#endif
