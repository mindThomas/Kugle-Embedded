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
 
#ifndef MISC_IIR_H
#define MISC_IIR_H

template <int ORDER>
class IIR
{
		public:
			IIR();
			IIR(const float coeff_a[], const float coeff_b[]);
			void Initialize(const float coeff_a[], const float coeff_b[]);
			float Filter(float input);

		private:
			float a[ORDER+1];
			float b[ORDER+1];

			float u_old[ORDER];
			float y_old[ORDER];
};
	
	
#endif
