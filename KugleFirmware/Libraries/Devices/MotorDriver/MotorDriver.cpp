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
 
#include "Motor.h"
#include "MotorDriver.h"
#include "stm32h7xx_hal.h"

#include "Debug.h"
#include "PWM.h"
#include "IO.h"
#include "ADC.h"
#include "Encoder.h"

#include <math.h>
#include <vector>

float MotorDriver::currentFeedbacks_[3];
float MotorDriver::normalizedDuties[3];

std::list<MotorDriver *> MotorDriver::motorDrivers;
TaskHandle_t MotorDriver::currentControllerTaskHandle = 0;
bool MotorDriver::currentControllerShouldStop = false;
bool MotorDriver::currentControllerIsRunning = false;

/*
MotorDriver::MotorDriver(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed) :
	ESCON_MAX_AMP_SETPOINT(MaxCurrent),
	MOTOR_TORQUE_CONSTANT(TorqueConstant),
	ENCODER_TICKS_PR_REV(EncoderTicksPrRev),
	GEARING_RATIO(GearRatio),
	ESCON_MAX_RAD_PR_SEC(MaxMotorSpeed),
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

MotorDriver::MotorDriver(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed, ADC * CurrentFeedback, ADC * VelocityFeedback, IO * DirectionFeedbackPin) :
	ESCON_MAX_AMP_SETPOINT(MaxCurrent),
	MOTOR_TORQUE_CONSTANT(TorqueConstant),
	ENCODER_TICKS_PR_REV(EncoderTicksPrRev),
	GEARING_RATIO(GearRatio),
	ESCON_MAX_RAD_PR_SEC(MaxMotorSpeed),
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
*/

MotorDriver::MotorDriver(uint8_t MotorIndex, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed, uint32_t currentControllerTaskPriority) :
	Motor(EncoderTicksPrRev, GearRatio),
	MAX_RAD_PR_SEC(MaxMotorSpeed),
	MAX_AMP_SETPOINT(MaxCurrent),
	MOTOR_TORQUE_CONSTANT(TorqueConstant),
	_PWM(0),
	_EN(0),
	_INA(0),
	_INB(0),
	_INA219_I2C(0),
	_currentSense(0),
	_deleteObjectsAtDestruction(true),
	_currentSenseLPF(0.05f, 0.5f / 3),
	_currentControlEnabled(false),
	_currentSetpoint(0),
	_currentSetpointSemaphore(0)
{
	// Instantiate peripheral objects according to selected motor index
	if (MotorIndex == 1)
	{
		_PWM = new PWM(PWM::TIMER1, PWM::CH1, PWM_FREQUENCY, PWM_RANGE);
		_INA = new IO(GPIOD, GPIO_PIN_14); _INA->ChangeToOpenDrain();
		_INB = new IO(GPIOD, GPIO_PIN_15); _INB->ChangeToOpenDrain();
		_EN = new IO(GPIOF, GPIO_PIN_14); _EN->ChangeToOpenDrain();

		_INA219_I2C = new I2C(I2C::PORT_I2C2, INA219::I2C_ADDRESS1);
		if (_INA219_I2C)
			_currentSense = new INA219(*_INA219_I2C);

		_encoder = new Encoder(Encoder::TIMER2);
	}
	else if (MotorIndex == 2)
	{
		_PWM = new PWM(PWM::TIMER1, PWM::CH2, PWM_FREQUENCY, PWM_RANGE);
		_INA = new IO(GPIOF, GPIO_PIN_13); _INA->ChangeToOpenDrain();
		_INB = new IO(GPIOF, GPIO_PIN_12); _INB->ChangeToOpenDrain();
		_EN = new IO(GPIOE, GPIO_PIN_10); _EN->ChangeToOpenDrain();

		_INA219_I2C = new I2C(I2C::PORT_I2C2, INA219::I2C_ADDRESS2);
		if (_INA219_I2C)
			_currentSense = new INA219(*_INA219_I2C);

		_encoder = new Encoder(Encoder::TIMER3);
	}
	else if (MotorIndex == 3)
	{
		_PWM = new PWM(PWM::TIMER1, PWM::CH3, PWM_FREQUENCY, PWM_RANGE);
		_INA = new IO(GPIOE, GPIO_PIN_14); _INA->ChangeToOpenDrain();
		_INB = new IO(GPIOE, GPIO_PIN_15); _INB->ChangeToOpenDrain();
		_EN = new IO(GPIOE, GPIO_PIN_12); _EN->ChangeToOpenDrain();

		_INA219_I2C = new I2C(I2C::PORT_I2C2, INA219::I2C_ADDRESS3);
		if (_INA219_I2C)
			_currentSense = new INA219(*_INA219_I2C);

		_encoder = new Encoder(Encoder::TIMER4);
	}
	else
	{
		ERROR("Incorrect motor index");
		return;
	}

	_currentSetpointSemaphore = xSemaphoreCreateBinary();
	if (_currentSetpointSemaphore == NULL) {
		ERROR("Could not create current setpoint semaphore");
		return;
	}
	vQueueAddToRegistry(_currentSetpointSemaphore, "Current setpoint");
	xSemaphoreGive( _currentSetpointSemaphore ); // give the semaphore the first time

	_currentFeedbackSemaphore = xSemaphoreCreateBinary();
	if (_currentFeedbackSemaphore == NULL) {
		ERROR("Could not create current feedback semaphore");
		return;
	}
	vQueueAddToRegistry(_currentFeedbackSemaphore, "Current feedback");
	xSemaphoreGive( _currentFeedbackSemaphore ); // give the semaphore the first time

	SetTorque(0);
	Disable();

	if (currentControllerTaskPriority > 0) {
		if (!currentControllerTaskHandle)
			xTaskCreate(MotorDriver::CurrentControllerThread, (char *)"Motor current controller", CURRENT_CONTROLLER_THREAD_STACK_SIZE, (void*)NULL, currentControllerTaskPriority, &currentControllerTaskHandle);

		// Add this object to list of registered current-controlled motor drivers
		motorDrivers.push_back(this);
		_useCurrentControl = true;
	} else {
		_useCurrentControl = false;
	}
}

MotorDriver::~MotorDriver()
{
	// Remove this object from list of registered motor drivers
	motorDrivers.remove(this);

	if (motorDrivers.size() == 0) {
		// No more current drivers so stop controller thread
		currentControllerShouldStop = true;
		while (currentControllerIsRunning) osDelay(10); // wait for the current controller stopping
		currentControllerTaskHandle = 0;
		currentControllerShouldStop = false;
	}

	if (_deleteObjectsAtDestruction) {
		if (_encoder)
			delete(_encoder);
		if (_currentSense)
			delete(_currentSense);
		if (_EN)
			delete(_EN);
		if (_INA)
			delete(_INA);
		if (_INB)
			delete(_INB);
		if (_PWM)
			delete(_PWM);
	}

	if (_currentSetpointSemaphore) {
		vQueueUnregisterQueue(_currentSetpointSemaphore);
		vSemaphoreDelete(_currentSetpointSemaphore);
	}

	if (_currentFeedbackSemaphore) {
		vQueueUnregisterQueue(_currentFeedbackSemaphore);
		vSemaphoreDelete(_currentFeedbackSemaphore);
	}
}

void MotorDriver::Enable()
{
	if (!_EN) return;
	_EN->High();
	_currentControlEnabled = true;
}

void MotorDriver::Disable()
{
	if (!_EN) return;
	_EN->Low();
	_currentControlEnabled = false;
}

void MotorDriver::SetPWM(float pwmDutyPercentage)
{
	if (!_PWM || !_INA || !_INB) return;
	// Set PWM output
	_PWM->Set(fabs(pwmDutyPercentage)); // note maximum duty cycle should ensure that low period is longer than at least 6 us

	// Set direction pins
	if (pwmDutyPercentage > 0) {
		_INA->High();
		_INB->Low();
	} else if (pwmDutyPercentage < 0) {
		_INA->Low();
		_INB->High();
	}/* else {
		// Brake to GND
		_INA->Low();
		_INB->Low();
	}*/
}

// Set torque in Newton meters (Nm)
// Returns a boolean indicating whether the applied torque was saturated/clipped
bool MotorDriver::SetTorque(float torqueNewtonMeter)
{
	bool didClip = false;

	if (xSemaphoreTake( _currentSetpointSemaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading (wait up to 1 ms to get semaphore)
		// Torque is set as a current setpoint
		// The current setpoint is calculated from the desired torque using the torque constant
		_currentSetpoint = torqueNewtonMeter / MOTOR_TORQUE_CONSTANT;

		// Saturate/clip current setpoint
		if (_currentSetpoint > MAX_AMP_SETPOINT) {
			_currentSetpoint = MAX_AMP_SETPOINT;
			didClip = true;
		}
		else if (_currentSetpoint < -MAX_AMP_SETPOINT) {
			_currentSetpoint = -MAX_AMP_SETPOINT;
			didClip = true;
		}

		xSemaphoreGive( _currentSetpointSemaphore ); // give semaphore back

		return didClip;
	} else {
		return true;
	}
}

// Set current in Amps (A)
// Returns a boolean indicating whether the applied torque was saturated/clipped
bool MotorDriver::SetCurrent(float current)
{
	bool didClip = false;

	if (xSemaphoreTake( _currentSetpointSemaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading (wait up to 1 ms to get semaphore)
		_currentSetpoint = current;

		// Saturate/clip current setpoint
		if (_currentSetpoint > MAX_AMP_SETPOINT) {
			_currentSetpoint = MAX_AMP_SETPOINT;
			didClip = true;
		}
		else if (_currentSetpoint < -MAX_AMP_SETPOINT) {
			_currentSetpoint = -MAX_AMP_SETPOINT;
			didClip = true;
		}

		xSemaphoreGive( _currentSetpointSemaphore ); // give semaphore back

		return didClip;
	} else {
		return true;
	}
}

// Return actual motor current reading in Amps (A)
float MotorDriver::GetCurrent()
{
	float currentFeedback = 0; // amps

	if (xSemaphoreTake( _currentFeedbackSemaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading (wait up to 1 ms to get semaphore)
		currentFeedback = _currentFeedback;
	}

	return currentFeedback;
}

// Return applied torque (based on current reading) in Newton meters (Nm)
float MotorDriver::GetAppliedTorque()
{
	float Current = GetCurrent();
	//if (Current < 0) return -1.0f; // error

	// The applied motor current can be converted to torque using the torque constant (Nm/A)
	return MOTOR_TORQUE_CONSTANT * Current;
}

void MotorDriver::SetDirectPWM(float pwmDutyPercentage)
{
	if (!_useCurrentControl)
		SetPWM(pwmDutyPercentage);
}

void MotorDriver::CurrentControllerThread(void * pvParameters)
{
	MotorDriver::currentControllerIsRunning = true;

	int i;
	std::vector<float> currentSetpoints; // hold a local copy of current setpoints
	std::vector<float> currentFeedbacks; // hold the previous value of the current feedback
	float currentFeedback;
	float NormalizedDuty;

	// Create an iterator for motor drivers list
	std::list<MotorDriver *>::iterator it;

	// 1000 Hz current control loop
	while (!MotorDriver::currentControllerShouldStop) {
		if (currentSetpoints.size() != MotorDriver::motorDrivers.size())
			currentSetpoints.resize(MotorDriver::motorDrivers.size(), 0);
		if (currentFeedbacks.size() != MotorDriver::motorDrivers.size())
			currentFeedbacks.resize(MotorDriver::motorDrivers.size(), 0);

		// Iterate through motor drivers list
		for (it = MotorDriver::motorDrivers.begin(), i = 0; it != MotorDriver::motorDrivers.end(); it++, i++)
		{
			// Get current setpoint
			if (xSemaphoreTake( (*it)->_currentSetpointSemaphore, ( TickType_t ) 0) == pdTRUE) { // lock for reading if possible - but do not wait if unavailable
				currentSetpoints[i] = (*it)->_currentSetpoint;
				xSemaphoreGive( (*it)->_currentSetpointSemaphore ); // give semaphore back
			}

			// Read current from current sensor
			currentFeedback = 0;
			if ((*it)->_currentSense)
				if (!(*it)->_currentSense->getCurrentAmps(&currentFeedback)) // store in amps
					currentFeedback = currentFeedbacks[i]; // at error, take previous value

			currentFeedback = (*it)->_currentSenseLPF.Filter(currentFeedback);

			// Store current feedback in variable
			if (xSemaphoreTake( (*it)->_currentFeedbackSemaphore, ( TickType_t ) 0) == pdTRUE) { // lock for writing if possible - but do not wait if unavailable
				(*it)->_currentFeedback = currentFeedback;
				xSemaphoreGive( (*it)->_currentFeedbackSemaphore ); // give semaphore back
			}

			currentFeedbacks[i] = currentFeedback;
			currentFeedbacks_[i] = currentFeedback;

			// Compute Current control law  ---  Proportional controller
			NormalizedDuty = (*it)->K_P * (fabs(currentSetpoints[i]) - currentFeedback);
			if (NormalizedDuty < 0) NormalizedDuty = 0;
			NormalizedDuty *= copysignf(1.0f, currentSetpoints[i]);

			if ((*it)->_currentControlEnabled)
				(*it)->SetPWM(NormalizedDuty);
			else
				(*it)->SetPWM(0.f);

			normalizedDuties[i] = NormalizedDuty;
			if (normalizedDuties[i] > 1) normalizedDuties[i] = 1.0f;
			else if (normalizedDuties[i] < -1) normalizedDuties[i] = -1.0f;
		}

		osDelay(5);
	}

	MotorDriver::currentControllerIsRunning = false;
	MotorDriver::currentControllerTaskHandle = 0;
	vTaskDelete(NULL); // delete/stop this current task
}
