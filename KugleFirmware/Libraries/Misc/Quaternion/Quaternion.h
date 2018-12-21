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
 
#ifndef MISC_QUATERNION_H
#define MISC_QUATERNION_H

#include <stddef.h>
#include <stdlib.h>

#define M_PI 3.14159265358979323846264338327950288

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)
#define devec(x) (&x[1]) // devectorize from dimension 4 to dimension 3 (take bottom 3 elements) - works only on vector (float array)

// Quaternion class for computations with quaternions of the format
//   q = {q0, q1, q2, q3} = {w, x, y, z} = {w, v}
// Hence a 4-dimensional vector where the scalar value is first and the vector part are the 3 bottom elements
//   w = q0
//   v = {q1, q2, q3}

class Quaternion
{

public:
	Quaternion();	
	Quaternion(const float q[4]);
	Quaternion(float q0, float q1, float q2, float q3);
	~Quaternion();

private:
	union q
	{
		float q[4];
		struct {
			float scalar;
			float vector[3];
		};
	};		
};
	

extern void Quaternion_Phi(const float q[4], const float p[4], float result[4]); // result = q o p = Phi(q)*p
extern void Quaternion_devecPhi(const float q[4], const float p[4], float result[3]); // result = V * q o p = V*Phi(q)*p
extern void Quaternion_PhiT(const float q[4], const float p[4], float result[4]); // result = q* o p = Phi(q)^T*p
extern void Quaternion_devecPhiT(const float q[4], const float p[4], float result[3]); // result = V * q* o p = V*Phi(q)^T*p
extern void Quaternion_mat_PhiT(const float q[4], float mat[4*4]);
extern void Quaternion_mat_devecPhiT(const float q[4], float mat[3*4]);
extern void Quaternion_Gamma(const float p[4], const float q[4], float result[4]); // result = q o p = Gamma(p)*q
extern void Quaternion_GammaT(const float p[4], const float q[4], float result[4]); // result = q o p* = Gamma(p)^T*q
extern void Quaternion_mat_GammaT(const float p[4], float mat[4*4]); // mat = Gamma(p)'
extern void Quaternion_mat_devecGammaT(const float p[4], float mat[3*4]); // mat = devec*Gamma(p)'
extern void Quaternion_Conjugate(const float q[4], float result[4]); // result = q*
extern void Quaternion_Conjugate(float q[4]); // q = q*
extern void Quaternion_Print(const float q[4]);
extern void Quaternion_Normalize(const float q[4], float q_out[4]);
extern void Quaternion_Normalize(float q[4]);
extern void Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll, float q[4]);
extern void Quaternion_quat2eul_zyx(const float q[4], float yaw_pitch_roll[3]);
extern void Quaternion_AngleClamp(float q[4], float angleMax, float q_clamped[4]);

extern float invSqrt(float x);

#endif
