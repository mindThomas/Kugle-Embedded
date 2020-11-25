/* Copyright (C) 2020 Victor Borja. All rights reserved.
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
 * Victor Borja
 * e-mail   :  v.borja1991@gmail.com
 * ------------------------------------------
 */
 
#ifndef MODULES_CONTROLLERS_FEEDBACKLINEARIZATION_H
#define MODULES_CONTROLLERS_FEEDBACKLINEARIZATION_H

#include <stddef.h>
#include <stdlib.h>

#include "Parameters.h"

class FeedbackLinearization
{
	public:
		FeedbackLinearization(Parameters& params);
		~FeedbackLinearization();

		void Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], float tau[3]);
		void Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float COM[3], const float q_ref[4], const float omega_ref[3], float tau[3]);
		void Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref_in[4], const float omega_ref_body[3], const float Jk, const float Mk, const float rk, const float Mb, const float Jbx, const float Jby, const float Jbz, const float Jw, const float rw, const float Bvk, const float Bvm, const float Bvb, const float l, const float g_const, const float COM_X, const float COM_Y, const float COM_Z, const float Kf[6], float tau[3]);

		bool UnitTest(void);

	private:
		void Saturation(const float * in, const int size, float * out);
		void Sign(float * in, int size, float * out);

	private:
		Parameters& _params;
		//PROVISIONAL 0 =Q_DOT_BODY_MANIFOLD , 1=Q_DOT_INERTIAL_MANIFOLD, 2 =OMEGA_BODY_MANIFOLD 3=OMEGA_INERTIAL_MANIFOLD,
		// TODO adjust on messages for body and inertial frame.
		int _frameType;
};
	
	
#endif
