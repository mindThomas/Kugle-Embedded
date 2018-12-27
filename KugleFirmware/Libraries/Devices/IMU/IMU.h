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
 
#ifndef DEVICES_IMU_H
#define DEVICES_IMU_H

#include "cmsis_os.h"

class IMU
{
	public:
		typedef struct Measurement_t {
			float Accelerometer[3];
			float Gyroscope[3];
			float Magnetometer[3];
		} Measurement_t;

	public:
		virtual ~IMU() {};

		virtual uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY) { return pdFALSE; };
		virtual void Get(Measurement_t& measurement) {};

	private:
};
	
	
#endif
