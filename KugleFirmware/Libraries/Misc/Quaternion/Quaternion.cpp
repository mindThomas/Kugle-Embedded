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
 
#include "Quaternion.h"

#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

// Quaternion class for computations with quaternions of the format
//   q = {q0, q1, q2, q3} = {s, v}
// Hence a 4-dimensional vector where the scalar value is first and the vector part are the 3 bottom elements
//   s = q0
//   v = {q1, q2, q3}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* result = q o p = Phi(q)*p */
void Quaternion_Phi(const float q[4], const float p[4], float result[4])
{
    /*Phi = @(q)[q(0) -q(1) -q(2) -q(3);     % for q o p = Phi(q) * p
                 q(1) q(0)  -q(3) q(2);
                 q(2) q(3)  q(0)  -q(1);
                 q(3) -q(2) q(1)  q(0)];
    */
    result[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
    result[1] = q[1]*p[0] + q[0]*p[1] - q[3]*p[2] + q[2]*p[3];
    result[2] = q[2]*p[0] + q[3]*p[1] + q[0]*p[2] - q[1]*p[3];
    result[3] = q[3]*p[0] - q[2]*p[1] + q[1]*p[2] + q[0]*p[3];
}

/* result = V * q o p = V*Phi(q)*p */
void Quaternion_devecPhi(const float q[4], const float p[4], float result[3])
{
		// V (devec) removes the first row of the result
    /*Phi = @(q)[q(0) -q(1) -q(2) -q(3);     % for q o p = Phi(q) * p
                 q(1) q(0)  -q(3) q(2);
                 q(2) q(3)  q(0)  -q(1);
                 q(3) -q(2) q(1)  q(0)];
    */
    result[0] = q[1]*p[0] + q[0]*p[1] - q[3]*p[2] + q[2]*p[3];
    result[1] = q[2]*p[0] + q[3]*p[1] + q[0]*p[2] - q[1]*p[3];
    result[2] = q[3]*p[0] - q[2]*p[1] + q[1]*p[2] + q[0]*p[3];
}

/* result = q* o p = Phi(q)^T*p */
void Quaternion_PhiT(const float q[4], const float p[4], float result[4])
{
    /*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
                 -q(1) q(0)  q(3) -q(2);
                 -q(2) -q(3)  q(0)  q(1);
                 -q(3) q(2) -q(1)  q(0)];
    */
    result[0] = q[0]*p[0] + q[1]*p[1] + q[2]*p[2] + q[3]*p[3];
    result[1] = -q[1]*p[0] + q[0]*p[1] + q[3]*p[2] - q[2]*p[3];
    result[2] = -q[2]*p[0] - q[3]*p[1] + q[0]*p[2] + q[1]*p[3];
    result[3] = -q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3];
}

/* result = V * q* o p = V*Phi(q)^T*p */
void Quaternion_devecPhiT(const float q[4], const float p[4], float result[3])
{
		// V (devec) removes the first row of the result
    /*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
                 -q(1) q(0)  q(3) -q(2);
                 -q(2) -q(3)  q(0)  q(1);
                 -q(3) q(2) -q(1)  q(0)];
    */
    result[0] = -q[1]*p[0] + q[0]*p[1] + q[3]*p[2] - q[2]*p[3];
    result[1] = -q[2]*p[0] - q[3]*p[1] + q[0]*p[2] + q[1]*p[3];
    result[2] = -q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3];
}

/* mat = Phi(q)^T */
void Quaternion_mat_PhiT(const float q[4], float mat[4*4])
{
	/*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
							 -q(1) q(0)  q(3) -q(2);
							 -q(2) -q(3)  q(0)  q(1);
							 -q(3) q(2) -q(1)  q(0)];
	*/
    mat[0]  = q[0];   mat[1]  = q[1];   mat[2]  = q[2];   mat[3]  = q[3];
    mat[4]  = -q[1];  mat[5]  = q[0];   mat[6]  = q[3];  mat[7]  = -q[2];
    mat[8]  = -q[2];  mat[9]  = -q[3];   mat[10] = q[0];   mat[11] = q[1];
    mat[12] = -q[3];  mat[13] = q[2];  mat[14] = -q[1];   mat[15] = q[0];
}

/* mat = devec*Phi(q)^T */
void Quaternion_mat_devecPhiT(const float q[4], float mat[3*4])
{
    /*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
                    -p(1) p(0) -p(3) p(2);
                    -p(2) p(3) p(0) -p(1);
                    -p(3) -p(2) p(1) p(0)];
    */
    mat[0]  = -q[1];  mat[1]  = q[0];   mat[2]  = q[3];  mat[3]  = -q[2];
    mat[4]  = -q[2];  mat[5]  = -q[3];   mat[6] = q[0];   mat[7] = q[1];
    mat[8] = -q[3];  mat[9] = q[2];  mat[10] = -q[1];   mat[11] = q[0];
}

/* result = q o p = Gamma(p)*q */
void Quaternion_Gamma(const float p[4], const float q[4], float result[4])
{
    /*Gamma = @(p)[p(0) -p(1) -p(2) -p(3);   % for q o p = Gamma(p) * q
                   p(1) p(0) p(3) -p(2);
                   p(2) -p(3) p(0) p(1);
                   p(3) p(2) -p(1) p(0)];
    */
    result[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    result[1] = p[1]*q[0] + p[0]*q[1] + p[3]*q[2] - p[2]*q[3];
    result[2] = p[2]*q[0] - p[3]*q[1] + p[0]*q[2] + p[1]*q[3];
    result[3] = p[3]*q[0] + p[2]*q[1] - p[1]*q[2] + p[0]*q[3];
}

/* result = q o p* = Gamma(p)^T*q */
void Quaternion_GammaT(const float p[4], const float q[4], float result[4])
{
    /*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
                    -p(1) p(0) -p(3) p(2);
                    -p(2) p(3) p(0) -p(1);
                    -p(3) -p(2) p(1) p(0)];
    */
    result[0] = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
    result[1] = -p[1]*q[0] + p[0]*q[1] - p[3]*q[2] + p[2]*q[3];
    result[2] = -p[2]*q[0] + p[3]*q[1] + p[0]*q[2] - p[1]*q[3];
    result[3] = -p[3]*q[0] - p[2]*q[1] + p[1]*q[2] + p[0]*q[3];
}

/* mat = Gamma(p)^T */
void Quaternion_mat_GammaT(const float p[4], float mat[4*4])
{
    /*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
                    -p(1) p(0) -p(3) p(2);
                    -p(2) p(3) p(0) -p(1);
                    -p(3) -p(2) p(1) p(0)];
    */
    mat[0]  = p[0];   mat[1]  = p[1];   mat[2]  = p[2];   mat[3]  = p[3];
    mat[4]  = -p[1];  mat[5]  = p[0];   mat[6]  = -p[3];  mat[7]  = p[2];
    mat[8]  = -p[2];  mat[9]  = p[3];   mat[10] = p[0];   mat[11] = -p[1];
    mat[12] = -p[3];  mat[13] = -p[2];  mat[14] = p[1];   mat[15] = p[0];
}

/* mat = devec*Gamma(p)^T */
void Quaternion_mat_devecGammaT(const float p[4], float mat[3*4])
{
    /*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
                    -p(1) p(0) -p(3) p(2);
                    -p(2) p(3) p(0) -p(1);
                    -p(3) -p(2) p(1) p(0)];
    */
    mat[0]  = -p[1];  mat[1]  = p[0];   mat[2]  = -p[3];  mat[3]  = p[2];
    mat[4]  = -p[2];  mat[5]  = p[3];   mat[6]  = p[0];   mat[7]  = -p[1];
    mat[8]  = -p[3];  mat[9]  = -p[2];  mat[10] = p[1];   mat[11] = p[0];
}

/* result = q* */
void Quaternion_Conjugate(const float q[4], float result[4])
{
    result[0] = q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];
}

/* q = q* */
void Quaternion_Conjugate(float q[4])
{
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

void Quaternion_Print(const float q[4])
{
    /*Serial.print("  ");
    Serial.printf("%7.4f\n", q[0]);
    Serial.print("  ");
    Serial.printf("%7.4f\n", q[1]);
    Serial.print("  ");
    Serial.printf("%7.4f\n", q[2]);
    Serial.print("  ");
    Serial.printf("%7.4f\n", q[3]);*/
}

void Quaternion_Normalize(const float q[4], float q_out[4])
{
	float normFactor = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q_out[0] = normFactor * q[0];
	q_out[1] = normFactor * q[1];
	q_out[2] = normFactor * q[2];
	q_out[3] = normFactor * q[3];
}

void Quaternion_Normalize(float q[4])
{
	float normFactor = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] *= normFactor;
	q[1] *= normFactor;
	q[2] *= normFactor;
	q[3] *= normFactor;
}

void Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll, float q[4])
{
    const float cx = cosf(roll/2);
    const float cy = cosf(pitch/2);
    const float cz = cosf(yaw/2);
    const float sx = sinf(roll/2);
    const float sy = sinf(pitch/2);
    const float sz = sinf(yaw/2);

    q[0] = cz*cy*cx+sz*sy*sx;
    q[1] = cz*cy*sx-sz*sy*cx;
    q[2] = cz*sy*cx+sz*cy*sx;
    q[3] = sz*cy*cx-cz*sy*sx;
}

void Quaternion_quat2eul_zyx(const float q[4], float yaw_pitch_roll[3])
{
	// Normalize quaternion
	float q_normalized[4];
	Quaternion_Normalize(q, q_normalized);

	float qw = q_normalized[0];
	float qx = q_normalized[1];
	float qy = q_normalized[2];
	float qz = q_normalized[3];

  float aSinInput = -2*(qx*qz-qw*qy);
	aSinInput = fmax(fmin(aSinInput, 1.f), -1.f);

  yaw_pitch_roll[0] = atan2( 2*(qx*qy+qw*qz), qw*qw + qx*qx - qy*qy - qz*qz ); // yaw
  yaw_pitch_roll[1] = asin( aSinInput ); // pitch
	yaw_pitch_roll[2] = atan2( 2*(qy*qz+qw*qx), qw*qw - qx*qx - qy*qy + qz*qz ); // roll
}

void Quaternion_AngleClamp(float q[4], float angleMax, float q_clamped[4])
{
	// Bound/clamp quaternion rotation amount by angle
	float cosAngle, sinAngle, currentAngle, clampedAngle; // tan = sin/cos
	cosAngle = q[0];
	sinAngle = sqrtf(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); // norm of rotation axis

	if (sinAngle == 0) {
		// Return unit quaternion if there is no tilt, hence norm is zero
		q_clamped[0] = 1;
		q_clamped[1] = 0;
		q_clamped[2] = 0;
		q_clamped[3] = 0;
		return;
	}

	currentAngle = atan2(sinAngle, cosAngle) * 2;
	clampedAngle = fmin(fmax(currentAngle, -angleMax), angleMax);

	// Form clamped quaternion
	q_clamped[0] = cosf(clampedAngle / 2);
	q_clamped[1] = (q[1] / sinAngle) * sinf(clampedAngle / 2);
	q_clamped[2] = (q[2] / sinAngle) * sinf(clampedAngle / 2);
	q_clamped[3] = (q[3] / sinAngle) * sinf(clampedAngle / 2);
}
