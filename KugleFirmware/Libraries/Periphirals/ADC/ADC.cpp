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
 
#include "ADC.h"
#include "stm32h7xx_hal.h"
#include "Debug.h"
#include <string.h> // for memset

ADC::hardware_resource_t * ADC::resADC1 = 0;
ADC::hardware_resource_t * ADC::resADC2 = 0;
ADC::hardware_resource_t * ADC::resADC3 = 0;

ADC::ADC(adc_t adc, uint32_t channel, uint32_t resolution) : _channel(channel)
{
	InitPeripheral(adc, channel, resolution);
}
ADC::ADC(adc_t adc, uint32_t channel) : _channel(channel)
{
	InitPeripheral(adc, channel, ADC_DEFAULT_RESOLUTION);
}

ADC::~ADC()
{
	if (!_hRes) return;
	uint16_t channelBit = 1 << _channel;
	_hRes->configuredChannels &= !channelBit;

	// Stop ADC
	if (HAL_ADC_Stop(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not stop ADC");
		return;
	}

	// Missing deinit of GPIO, eg. HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3)

	if (_hRes->configuredChannels == 0) { // no more channels in use in resource, so delete the resource
		// Delete hardware resource
		timer_t tmpADC = _hRes->adc;
		delete(_hRes);

		switch (tmpADC)
		{
			case ADC_1:
				if (!resADC2)
					__HAL_RCC_ADC12_CLK_DISABLE();
				resADC1 = 0;
				break;
			case ADC_2:
				if (!resADC1)
					__HAL_RCC_ADC12_CLK_DISABLE();
				resADC2 = 0;
				break;
			case ADC_3:
				__HAL_RCC_ADC3_CLK_DISABLE();
				resADC3 = 0;
				break;
			default:
				ERROR("Undefined ADC");
				return;
		}
	}
}

void ADC::InitPeripheral(adc_t adc, uint32_t channel, uint32_t resolution)
{
	bool configureResource = false;

	_hRes = 0;

	switch (adc)
	{
		case ADC_1:
			if (!resADC1) {
				resADC1 = new ADC::hardware_resource_t;
				memset(resADC1, 0, sizeof(ADC::hardware_resource_t));
				configureResource = true;
				_hRes = resADC1;
			}
			else {
				_hRes = resADC1;
			}
			break;
		case ADC_2:
			if (!resADC2) {
				resADC2 = new ADC::hardware_resource_t;
				memset(resADC2, 0, sizeof(ADC::hardware_resource_t));
				configureResource = true;
				_hRes = resADC2;
			}
			else {
				_hRes = resADC2;
			}
			break;
		case ADC_3:
			if (!resADC3) {
				resADC3 = new ADC::hardware_resource_t;
				memset(resADC3, 0, sizeof(ADC::hardware_resource_t));
				configureResource = true;
				_hRes = resADC3;
			}
			else {
				_hRes = resADC3;
			}
			break;
		default:
			ERROR("Undefined ADC");
			return;
	}

	if (configureResource) { // first time configuring peripheral
		_hRes->adc = adc;
		_hRes->configuredChannels = 0;
		_hRes->resolution = resolution;

		ConfigureADCPeripheral();
	}

	// Ensure that the channel is valid and not already in use
	uint16_t channelBit = 1 << channel;
	if ((_hRes->configuredChannels & channelBit) != 0) {
		_hRes = 0;
		ERROR("Channel already configured on selected ADC");
		return;
	}
	if (_hRes->resolution != resolution) {
		_hRes = 0;
		ERROR("ADC already in used with different resolution");
		return;
	}

	ConfigureADCGPIO();
	ConfigureADCChannel();

	if (resolution == ADC_RESOLUTION_8B)
		_range = ((uint32_t)1 << 8) - 1;
	else if (resolution == ADC_RESOLUTION_10B)
		_range = ((uint32_t)1 << 10) - 1;
	else if (resolution == ADC_RESOLUTION_12B)
		_range = ((uint32_t)1 << 12) - 1;
	else if (resolution == ADC_RESOLUTION_14B)
		_range = ((uint32_t)1 << 14) - 1;
	else if (resolution == ADC_RESOLUTION_16B)
		_range = ((uint32_t)1 << 16) - 1;
	else {
		_hRes = 0;
		ERROR("Incorrect ADC resolution");
		return;
	}
}

void ADC::ConfigureADCPeripheral()
{
	if (!_hRes) return;

	if (_hRes->adc == ADC_1) {
		__HAL_RCC_ADC12_CLK_ENABLE();
		_hRes->handle.Instance = ADC1;
	} else if (_hRes->adc == ADC_2) {
		__HAL_RCC_ADC12_CLK_ENABLE();
		_hRes->handle.Instance = ADC2;
	} else if (_hRes->adc == ADC_3) {
		__HAL_RCC_ADC3_CLK_ENABLE();
		_hRes->handle.Instance = ADC3;
	}

	if (_hRes->resolution == ADC_RESOLUTION_8B)
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_8B; /* 8-bit resolution for converted data */
	else if (_hRes->resolution == ADC_RESOLUTION_10B)
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_10B; /* 10-bit resolution for converted data */
	else if (_hRes->resolution == ADC_RESOLUTION_12B)
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_12B; /* 12-bit resolution for converted data */
	else if (_hRes->resolution == ADC_RESOLUTION_14B)
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_14B; /* 14-bit resolution for converted data */
	else if (_hRes->resolution == ADC_RESOLUTION_16B)
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_16B; /* 16-bit resolution for converted data */
	else {
		_hRes = 0;
		ERROR("Incorrect ADC resolution");
		return;
	}

	_hRes->handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; // ADC clock input is configured to be 80 MHz, divided by 4 gives an ADC clock of 20 MHz
	_hRes->handle.Init.ScanConvMode = ADC_SCAN_DISABLE; /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	_hRes->handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; /* EOC flag picked-up to indicate conversion end */
	_hRes->handle.Init.LowPowerAutoWait = DISABLE;
	_hRes->handle.Init.ContinuousConvMode = ENABLE; /* Continuous mode enabled (automatic conversion restart after each conversion) */
	_hRes->handle.Init.NbrOfConversion = 1;
	_hRes->handle.Init.DiscontinuousConvMode = DISABLE;
	_hRes->handle.Init.ExternalTrigConv = ADC_SOFTWARE_START; /* Software start to trig the 1st conversion manually, without external event */
	_hRes->handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	_hRes->handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR; /* Regular Conversion data stored in DR register only */
	_hRes->handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN; /* DR register is overwritten with the last conversion result in case of overrun */
	_hRes->handle.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	_hRes->handle.Init.BoostMode = DISABLE; /* Boost mode can be disabled (to save power) since ADC clock frequency is less than or equal to 20 MHz */
	_hRes->handle.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not initialize ADC");
		return;
	}

	/*Configure the ADC multi-mode */
	/*ADC_MultiModeTypeDef multimode = {0};
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&_hRes->handle, &multimode) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure ADC");
		return;
	}*/

	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&_hRes->handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not calibrate ADC");
		return;
	}

	// Start continuous ADC conversion
	if (HAL_ADC_Start(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not start ADC");
		return;
	}
}

void ADC::ConfigureADCGPIO()
{
	if (!_hRes) return;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

	if (_hRes->adc == ADC_1)
	{
	    /**ADC1 GPIO Configuration
	    PC5     ------> ADC1_INP8
	    PF11     ------> ADC1_INP2
	    */
		if (_channel == ADC_CHANNEL_8) {
			GPIO_InitStruct.Pin = GPIO_PIN_5;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_2) {
			GPIO_InitStruct.Pin = GPIO_PIN_11;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else {
			_hRes = 0;
			ERROR("Invalid ADC channel");
			return;
		}
	}
	else if (_hRes->adc == ADC_2)
	{
	    /**ADC2 GPIO Configuration
	    PC0     ------> ADC2_INP10
	    PC1     ------> ADC2_INP11
	    */
		if (_channel == ADC_CHANNEL_10) {
			GPIO_InitStruct.Pin = GPIO_PIN_0;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_11) {
			GPIO_InitStruct.Pin = GPIO_PIN_1;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else {
			_hRes = 0;
			ERROR("Invalid ADC channel");
			return;
		}
	}
	else if (_hRes->adc == ADC_3)
	{
	    /**ADC3 GPIO Configuration
	    PF3     ------> ADC3_INP5
	    PF4     ------> ADC3_INP9
	    PF5     ------> ADC3_INP4
	    PF10     ------> ADC3_INP6
	    PC2_C     ------> ADC3_INP0
	    */
		if (_channel == ADC_CHANNEL_5) {
			GPIO_InitStruct.Pin = GPIO_PIN_3;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_9) {
			GPIO_InitStruct.Pin = GPIO_PIN_4;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_4) {
			GPIO_InitStruct.Pin = GPIO_PIN_5;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_6) {
			GPIO_InitStruct.Pin = GPIO_PIN_10;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_0) {
			GPIO_InitStruct.Pin = GPIO_PIN_2;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else {
			_hRes = 0;
			ERROR("Invalid ADC channel");
			return;
		}
	}
}

void ADC::ConfigureADCChannel()
{
	if (!_hRes) return;

	ADC_ChannelConfTypeDef sConfig = {0};

	// Stop ADC to be able to configure channel
	if (HAL_ADC_Stop(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not stop ADC");
		return;
	}

	sConfig.Channel = _channel;
	sConfig.Rank = ADC_REGULAR_RANK_1; /* Rank of sampled channel number ADCx_CHANNEL */
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; /* Sampling time (number of clock cycles unit) */
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&_hRes->handle, &sConfig) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure ADC channel");
		return;
	}

	// Start continuous ADC conversion
	if (HAL_ADC_Start(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not start ADC");
		return;
	}
}

// Return ADC reading between 0-1, where 1 corresponds to the ADC's Analog reference (Aref)
float ADC::Read()
{
	int32_t reading = ReadRaw();
	float converted = (float)reading / _range;
	return converted;
}

int32_t ADC::ReadRaw()
{
	if (HAL_ADC_PollForConversion(&_hRes->handle, HAL_MAX_DELAY) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		return -1;
	}
	else
	{
		/* ADC conversion completed */
		return HAL_ADC_GetValue(&_hRes->handle);
	}
}
