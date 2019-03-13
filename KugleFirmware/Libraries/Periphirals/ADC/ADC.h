/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
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
		void ConfigureADCChannels();
		void ConfigureDMA(void);
		HAL_StatusTypeDef StartDMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);

		float Read();
		int32_t ReadRaw();

	public:
		typedef struct hardware_resource_t {
			adc_t adc;
			uint32_t resolution;
			uint8_t numberOfConfiguredChannels;
			ADC_HandleTypeDef handle;
			DMA_HandleTypeDef DMA_handle;
			ALIGN_32BYTES (uint16_t buffer[16]);
			uint8_t bufferSize;
			uint8_t map_channel2bufferIndex[20];
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
