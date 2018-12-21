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
 
#ifndef MODULES_CONTROLLERS_SLIDINGMODE_MATLABCODER_H
#define MODULES_CONTROLLERS_SLIDINGMODE_MATLABCODER_H

#include <stddef.h>
#include <stdlib.h>

#include "Parameters.h"

class SlidingModeMATLABCoder
{
	public:
		SlidingModeMATLABCoder(Parameters& params);
		~SlidingModeMATLABCoder();

		void Step(float X[12], float q_ref[4], float tau[3], float S[3]);

	private:
		Parameters& _params;
};
	
	
#endif
