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
 
#include "Math.h"
#include <math.h>
#include <stdlib.h>

float Math_Round(float num, unsigned int dec)
{
	//float power = powf(10, dec);
	long long power = 1;
	for (int i = 0; i < dec; i++) {
		power *= 10;
	}

	return roundf(num * power) / power;
}

void Math_Rotate2D(const float rotationAngle, const float in[2], float out[2])
{
	/*
	 * R = [cos(A), -sin(A)
	 * 	    sin(A), cos(A)]
	 *
	 * out = R * in
	 */
	float sinA = sinf(rotationAngle);
	float cosA = cosf(rotationAngle);
	out[0] = cosA*in[0] - sinA*in[1];
	out[1] = sinA*in[0] + cosA*in[1];
}

void Math_Rotate2D(const double rotationAngle, const double in[2], double out[2])
{
	/*
	 * R = [cos(A), -sin(A)
	 * 	    sin(A), cos(A)]
	 *
	 * out = R * in
	 */
	double sinA = sin(rotationAngle);
	double cosA = cos(rotationAngle);
	out[0] = cosA*in[0] - sinA*in[1];
	out[1] = sinA*in[0] + cosA*in[1];
}
