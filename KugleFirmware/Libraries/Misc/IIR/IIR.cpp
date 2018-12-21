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
 
#include "IIR.h"
#include "Debug.h"

template <int ORDER>
IIR<ORDER>::IIR(const float coeff_a[], const float coeff_b[])
{
	if (ORDER != sizeof(coeff_a) / sizeof(float)) {
		ERROR("Mismatch with IIR coefficient lenght");
		return;
	}

	for (int i = 0; i < ORDER+1; i++) {
		a[i] = coeff_a[i];
		b[i] = coeff_b[i];
	}
	for (int i = 0; i < ORDER; i++) {
		u_old[i] = 0;
		y_old[i] = 0;
	}
}

template <int ORDER>
void IIR<ORDER>::Initialize(const float coeff_a[], const float coeff_b[])
{
	if (ORDER != sizeof(coeff_a) / sizeof(float)) {
		ERROR("Mismatch with IIR coefficient lenght");
		return;
	}

	for (int i = 0; i < ORDER+1; i++) {
		a[i] = coeff_a[i];
		b[i] = coeff_b[i];
	}
}

template <int ORDER>
float IIR<ORDER>::Filter(float input)
{
	// a[0]*y[k] + a[1]*y[k-1] + ... = b[0]*u[k] + b[1]*u[k-1] + ...
	float output;
	float ay = 0;
	float bu = 0;

	bu = b[0] * input;
	for (int i = ORDER; i > 0; i--) {
		bu += b[i] * u_old[i-1];
		ay += a[i] * y_old[i-1];

		if (i > 1) {
			u_old[i-1] = u_old[i-2];
			y_old[i-1] = y_old[i-2];
		}
	}

	output = 1/a[0] * (bu - ay);

	u_old[0] = input;
	y_old[0] = output;

	return output;
}
