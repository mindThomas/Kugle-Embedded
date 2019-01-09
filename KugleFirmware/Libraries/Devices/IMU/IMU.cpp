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
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "IMU.h"
 
// Class for sensor abstraction and sampling
// Should eg. configure MPU-9250 interrupt

#if 0
void MPU9250_Sampling(void const * argument)
{
	SPI * spi = new SPI(SPI::PORT_SPI3, MPU9250_SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
	MPU9250<SPI,MPU9250_SPI> * imu = new MPU9250<SPI,MPU9250_SPI>(spi);

	imu->Configure(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
	imu->setFilt(DLPF_BANDWIDTH_250HZ, DLPF_BANDWIDTH_184HZ, 8);
	imu->ConfigureInterrupt(GPIOE, GPIO_PIN_3);

	while (1) {
		imu->WaitForNewData();
	    imu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	}
}
#endif
