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
 
#ifndef PERIPHIRALS_ADC_H
#define PERIPHIRALS_ADC_H

#include "stm32h7xx_hal.h"

class ADC
{
	private:
		const uint32_t ADC_DEFAULT_RESOLUTION = ADC_RESOLUTION_12B;

	public:
		typedef enum adc_t {
			ADC_UNDEFINED = 0,
			ADC_1,
			ADC_2,
			ADC_3,
		} adc_t;

	public:
		ADC(adc_t adc, uint32_t channel);
		ADC(adc_t adc, uint32_t channel, uint32_t resolution);
		~ADC();

		void InitPeripheral(adc_t adc, uint32_t channel, uint32_t resolution);
		void ConfigureADCPeripheral();
		void ConfigureADCGPIO();
		void ConfigureADCChannel();

		float Read();
		int32_t ReadRaw();

	public:
		typedef struct hardware_resource_t {
			adc_t adc;
			uint32_t resolution;
			uint16_t configuredChannels; // each bit indicate whether the corresponding channel is configured and in use by another object
			ADC_HandleTypeDef handle;
		} hardware_resource_t;

		static hardware_resource_t * resADC1;
		static hardware_resource_t * resADC2;
		static hardware_resource_t * resADC3;

	private:
		hardware_resource_t * _hRes;
		uint32_t _channel;
		uint16_t _range;
};
	
	
#endif
