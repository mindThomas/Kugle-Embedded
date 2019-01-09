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
 
#ifndef PERIPHIRALS_I2C_H
#define PERIPHIRALS_I2C_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback

class I2C
{
	private:
		const uint32_t I2C_DEFAULT_FREQUENCY = 400000;	// 400 kHz

	public:
		typedef enum port_t {
			PORT_UNDEFINED = 0,
			PORT_I2C1,
			PORT_I2C3
		} port_t;

	public:
		I2C(port_t port, uint8_t devAddr); // use default frequency (or current configured frequency)
		I2C(port_t port, uint8_t devAddr, uint32_t frequency); // configure with frequency if possible
		~I2C();

		void InitPeripheral(port_t port, uint32_t frequency);
		void DeInitPeripheral();
		void ConfigurePeripheral();
		void Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength);
		void Write(uint8_t reg, uint8_t value);
		void Read(uint8_t reg, uint8_t * buffer, uint8_t readLength);
		uint8_t Read(uint8_t reg);

	public:
		typedef struct hardware_resource_t {
			port_t port;
			uint32_t frequency;
			SemaphoreHandle_t resourceSemaphore;
			SemaphoreHandle_t transmissionFinished;
			bool configured;
			uint8_t instances; // how many objects are using this hardware resource
			I2C_HandleTypeDef handle;
		} hardware_resource_t;

		static hardware_resource_t * resI2C1;
		static hardware_resource_t * resI2C3;

	private:
		hardware_resource_t * _hRes;
		uint8_t _devAddr;
	
	public:
		static void TransmissionCompleteCallback(I2C_HandleTypeDef *I2cHandle);
};
	
	
#endif
