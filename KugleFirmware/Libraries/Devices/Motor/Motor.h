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
 
#ifndef DEVICES_MOTOR_H
#define DEVICES_MOTOR_H

#include "stm32h7xx_hal.h"
#include "Encoder.h"

class Motor
{
	private:
		const double M_PI = 3.14159265358979323846264338327950288;

		const uint16_t ENCODER_TICKS_PR_REV;	 // ticks/rev on encoder side - hence one revolution on motor shaft (before gearing)
		const float GEARING_RATIO;

	public:
		Motor(float GearRatio, uint16_t EncoderTicksPrRev);
		virtual ~Motor() {};

		virtual void Enable() {};
		virtual void Disable() {};

		virtual bool SetCurrent(float current) { return false; };

		virtual bool SetTorque(float torqueNewtonMeter) { return false; };
		virtual float GetAppliedTorque() { return 0; };
		virtual float GetCurrent() { return 0; };

		bool SetOutputTorque(float torqueNewtonMeter);
		float GetAppliedOutputTorque();
		float GetAngle();
		int32_t GetEncoderRaw();

	protected: // "private" but accessible by derived classes
		Encoder * _encoder;
		
};
	
	
#endif
