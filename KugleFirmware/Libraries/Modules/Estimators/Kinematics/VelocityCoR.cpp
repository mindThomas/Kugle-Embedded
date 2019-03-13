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

#include "VelocityCoR.h"

#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

void _BallToCoRvelocity(const float vel_ball[2], const float q[4], const float dq[4], float CoR, float vel_CoR[2])
{
	// See "Simple offset estimator" in OneNote for derivation
	// vel_2L = vel_ball + devec*(Phi(dq)*Gamma(q)' + Phi(q)*Gamma(dq)')*[0,0,0,2*l]'
	// vel_2L = vel_ball + devec*Phi(dq)*Gamma(q)'*[0,0,0,2*l]' + devec*Phi(q)*Gamma(dq)'*[0,0,0,2*l]'

	/*vel_2L[0] = vel_ball[0];
	vel_2L[1] = vel_ball[1];

	float p_2L[4] = {0, 0, 0, 2*_l};
	float q_tmp[4];
	float vel_tmp[4];

	Quaternion_GammaT(q, p_2L, q_tmp); // Gamma(q)'*[0,0,0,2*l]'
	Quaternion_devecPhi(dq, q_tmp, vel_tmp); // devec * Phi(dq) * (Gamma(q)'*[0,0,0,2*l]')
	vel_2L[0] += vel_tmp[0];
	vel_2L[1] += vel_tmp[1];

	Quaternion_GammaT(dq, p_2L, q_tmp); // Gamma(dq)'*[0,0,0,2*l]'
	Quaternion_devecPhi(q, q_tmp, vel_tmp); // devec * Phi(q) * (Gamma(dq)'*[0,0,0,2*l]')
	vel_2L[0] += vel_tmp[0];
	vel_2L[1] += vel_tmp[1];*/

	vel_CoR[0] = vel_ball[0] + CoR*(2*dq[0]*q[2] + 2*dq[2]*q[0] + 2*dq[1]*q[3] + 2*dq[3]*q[1]);
    vel_CoR[1] = vel_ball[1] - CoR*(2*dq[0]*q[1] + 2*dq[1]*q[0] - 2*dq[2]*q[3] - 2*dq[3]*q[2]);
}

void _ConvertCoRtoBallVelocity(const float vel_CoR[2], const float q[4], const float dq[4], const float CoR, float vel_ball[2])
{
	vel_ball[0] = vel_CoR[0] - CoR*(2*dq[0]*q[2] + 2*dq[2]*q[0] + 2*dq[1]*q[3] + 2*dq[3]*q[1]);
	vel_ball[1] = vel_CoR[1] + CoR*(2*dq[0]*q[1] + 2*dq[1]*q[0] - 2*dq[2]*q[3] - 2*dq[3]*q[2]);
}
