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
 
#ifndef MISC_FIRSTORDERLPF_H
#define MISC_FIRSTORDERLPF_H

class FirstOrderLPF
{
	public:
		FirstOrderLPF(float Ts, float tau);
		~FirstOrderLPF();
		
		float Filter(float input);

	private:
		const float _Ts;             // Sampling Time
		const float _tau;            // Filter time constant

		const float _coeff_b = 0.0;  // IIR filter coefficient (nominator polynomial)
		const float _coeff_a = 0.0;  // IIR filter coefficient (denominator polynomial)
		
		float _lpfOld = 0.0;   // Holds previous sample output value
		float _inputOld = 0.0; // Holds previous sample output value
};
	
	
#endif
