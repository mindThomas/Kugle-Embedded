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
 
#include "ESCON.h"
#include "stm32h7xx_hal.h"

#include "Debug.h"
#include "PWM.h"
#include "IO.h"
#include "ADC.h"
#include "Encoder.h"

ESCON::ESCON(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder) :
	_torqueSetpoint(TorqueSetpoint),
	_enablePin(EnablePin),
	_encoder(encoder),
	_currentFeedback(0),
	_velocityFeedback(0),
	_directionFeedbackPin(0),
	_deleteObjectsAtDestruction(false)
{
	SetTorque(0);
	Disable();
}

ESCON::ESCON(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, ADC * CurrentFeedback, ADC * VelocityFeedback, IO * DirectionFeedbackPin) :
	_torqueSetpoint(TorqueSetpoint),
	_enablePin(EnablePin),
	_encoder(encoder),
	_currentFeedback(CurrentFeedback),
	_velocityFeedback(VelocityFeedback),
	_directionFeedbackPin(DirectionFeedbackPin),
	_deleteObjectsAtDestruction(false)
{
	SetTorque(0);
	Disable();
}

ESCON::ESCON(uint8_t MotorIndex) :
	_torqueSetpoint(0),
	_enablePin(0),
	_encoder(0),
	_currentFeedback(0),
	_velocityFeedback(0),
	_directionFeedbackPin(0),
	_deleteObjectsAtDestruction(true)
{
	// Instantiate periphiral objects according to selected motor index
	if (MotorIndex == 1)
	{
		_torqueSetpoint = new PWM(PWM::TIMER1, PWM::CH1, ESCON_PWM_FREQUENCY, ESCON_PWM_RANGE);
		_enablePin = new IO(GPIOC, GPIO_PIN_6); // configure as output
		_currentFeedback = new ADC(ADC::ADC_3, ADC_CHANNEL_5, ADC_RESOLUTION_12B);
		_velocityFeedback = new ADC(ADC::ADC_3, ADC_CHANNEL_9, ADC_RESOLUTION_12B);
		_encoder = new Encoder(Encoder::TIMER2);
		_directionFeedbackPin = new IO(GPIOD, GPIO_PIN_2, IO::PULL_UP); // configure as input
	}
	else if (MotorIndex == 2)
	{
		_torqueSetpoint = new PWM(PWM::TIMER1, PWM::CH2, ESCON_PWM_FREQUENCY, ESCON_PWM_RANGE);
		_enablePin = new IO(GPIOC, GPIO_PIN_7); // configure as output
		_currentFeedback = new ADC(ADC::ADC_3, ADC_CHANNEL_4, ADC_RESOLUTION_12B);
		_velocityFeedback = new ADC(ADC::ADC_3, ADC_CHANNEL_6, ADC_RESOLUTION_12B);
		_encoder = new Encoder(Encoder::TIMER3);
		_directionFeedbackPin = new IO(GPIOD, GPIO_PIN_8, IO::PULL_UP); // configure as input
	}
	else if (MotorIndex == 3)
	{
		_torqueSetpoint = new PWM(PWM::TIMER1, PWM::CH3, ESCON_PWM_FREQUENCY, ESCON_PWM_RANGE);
		_enablePin = new IO(GPIOC, GPIO_PIN_8); // configure as output
		_currentFeedback = new ADC(ADC::ADC_2, ADC_CHANNEL_10, ADC_RESOLUTION_12B);
		_velocityFeedback = new ADC(ADC::ADC_2, ADC_CHANNEL_11, ADC_RESOLUTION_12B);
		_encoder = new Encoder(Encoder::TIMER4);
		_directionFeedbackPin = new IO(GPIOD, GPIO_PIN_9, IO::PULL_UP); // configure as input
	}
	else
	{
		ERROR("Incorrect motor index");
		return;
	}

	SetTorque(0);
	Disable();
}

ESCON::~ESCON()
{
	if (_deleteObjectsAtDestruction) {
		if (_torqueSetpoint)
			delete(_torqueSetpoint);
		if (_velocityFeedback)
			delete(_velocityFeedback);
		if (_currentFeedback)
			delete(_currentFeedback);
		if (_encoder)
			delete(_encoder);
	}
}

void ESCON::Enable()
{
	if (!_enablePin) return;
	_enablePin->Set(true);
}

void ESCON::Disable()
{
	if (!_enablePin) return;
	_enablePin->Set(false);
}

// Set torque in Newton meters (Nm)
// Returns a boolean indicating whether the applied torque was saturated/clipped
bool ESCON::SetTorque(float torqueNewtonMeter)
{
	if (!_torqueSetpoint) return false;
	bool didClip = false;

	// Torque is set as a current setpoint
	// The current setpoint is calculated from the desired torque using the torque constant
	float currentSetpoint = torqueNewtonMeter / EC60_TORQUE_CONSTANT;

	// The current setpoint is normalized to a range between -1 to 1 using the maximum current defined in ESCON controller
	float normalizedCurrentSetpoint = currentSetpoint / ESCON_MAX_AMP_SETPOINT;

	// Saturate/clip current setpoint
	if (normalizedCurrentSetpoint > 1.0f) {
		normalizedCurrentSetpoint = 1.0f;
		didClip = true;
	}
	else if (normalizedCurrentSetpoint < -1.0f) {
		normalizedCurrentSetpoint = -1.0f;
		didClip = true;
	}

	// The current setpoint is set with a PWM duty cycle between 10% to 90%
	//   10% = Maximum negative current
	//   90% = Maximum positive current
	// The normalized current range from -1 to 1 (range of 2) should thus be squeezed into the PWM range of 0.1 to 0.9 (range of 0.8)
	const float squeezeFactor = 0.8f / 2.0f;

	// Compute the PWM value by applying the squeeze factor and off-setting by the center value of 50% = 0.5
	float PWMvalue = 0.5f + squeezeFactor * normalizedCurrentSetpoint;

	// Update the PWM value
	_torqueSetpoint->Set(PWMvalue);

	return didClip;
}

// Return actual motor current reading in Amps (A)
float ESCON::GetCurrent()
{
	if (!_currentFeedback) return -1.0f;
	float AnalogReading = _currentFeedback->Read();

	// ESCON driver outputs an analog value from 0V to 3.3V
	// This range spans the normalized value from -1 to 1
	// However from the ADC reading we will get a value from 0 to 1, which we thus scale and offset accordingly
	float NormalizedCurrent = 2 * AnalogReading - 1.0f;

	// The normalized current relates to the maximum current (ESCON rating)
	return ESCON_MAX_AMP_SETPOINT * NormalizedCurrent;
}

// Return applied torque (based on current reading) in Newton meters (Nm)
float ESCON::GetAppliedTorque()
{
	float Current = GetCurrent();
	//if (Current < 0) return -1.0f; // error

	// The applied motor current can be converted to torque using the torque constant (Nm/A)
	return EC60_TORQUE_CONSTANT * Current;
}

int32_t ESCON::GetEncoderRaw()
{
	if (!_encoder) return 0;
	return _encoder->Get();
}

// Return motor angle in radians (rad)
float ESCON::GetAngle()
{
	if (!_encoder) return 0;
	int32_t encoderReading = _encoder->Get();

	// The encoder reading is in number of quadrature ticks (counting each edge on the two signal wires, hence 4 ticks pr. repetition)
	float absoluteMotorRevolutions = (float)encoderReading / ENCODER_TICKS_PR_REV;

	// Since the motor is geared the absolute output shaft angle is less than the absolute motor shaft angle
	float absoluteOutputRevolutions = absoluteMotorRevolutions / GEARING_RATIO;

	// Convert to radians
	return 2 * M_PI * absoluteOutputRevolutions;
}

// Return motor velocity in radians pr. second (rad/s)
float ESCON::GetVelocity()
{
	if (!_velocityFeedback) return -1.0f;
	float AnalogReading = _velocityFeedback->Read();

	// ESCON driver outputs an analog value from 0V to 3.3V
	// This range spans the normalized value from -1 to 1
	// However from the ADC reading we will get a value from 0 to 1, which we thus scale and offset accordingly
	float NormalizedVelocity = 2 * AnalogReading - 1.0f;

	// The normalized velocity relates to the maximum angular velocity defined in the ESCON controller
	return ESCON_MAX_RAD_PR_SEC * NormalizedVelocity;
}
