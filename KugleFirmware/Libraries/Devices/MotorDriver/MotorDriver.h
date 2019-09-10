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
 
#ifndef DEVICES_MOTORDRIVER_H
#define DEVICES_MOTORDRIVER_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "PWM.h"
#include "IO.h"
#include "ADC.h"
#include "I2C.h"
#include "INA219.h"
#include "FirstOrderLPF.h"
#include <list>

class MotorDriver : private Motor
{
	private:
		const float K_P = 5; // current controller gain - from current error to normalized PWM (-1 to 1)

		const int PWM_FREQUENCY	= 8000;		// 20 kHz
		const int PWM_RANGE = 2000;				// 0-2000, corresponding to 0.1% resolution

		const float MAX_RAD_PR_SEC; // rad/s
		const float MAX_AMP_SETPOINT;	 // A
		const float MOTOR_TORQUE_CONSTANT;  // Nm/A

		const int CURRENT_CONTROLLER_THREAD_STACK_SIZE = 256;

	public:
		//MotorDriver(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed); // minimal operation general constructor
		//MotorDriver(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed, ADC * CurrentFeedback, ADC * VelocityFeedback, IO * DirectionFeedbackPin); // extended operation general constructor
		MotorDriver(uint8_t MotorIndex, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed, uint32_t currentControllerTaskPriority = 0); // platform specific constructor
		~MotorDriver();

		void Enable();
		void Disable();

		bool SetCurrent(float current);

		bool SetTorque(float torqueNewtonMeter);
		bool SetOutputTorque(float torqueNewtonMeter);
		float GetAppliedTorque();
		float GetCurrent();

		void SetDirectPWM(float pwmDutyPercentage); // only possible when current control mode is not used

	private:
		void SetPWM(float pwmDutyPercentage);

	private:
		PWM * _PWM; 			// PWM on VNH2SP30
		IO * _EN;				// EN on VNH2SP30
		IO * _INA;				// INA on VNH2SP30
		IO * _INB;				// INA on VNH2SP30
		I2C * _INA219_I2C;		// I2C for INA219 current sensor
		INA219 * _currentSense; // Current sense chip
		bool _deleteObjectsAtDestruction;

		FirstOrderLPF _currentSenseLPF;

		bool _useCurrentControl;
		bool _currentControlEnabled;

		float _currentSetpoint;
		SemaphoreHandle_t _currentSetpointSemaphore;

		float _currentFeedback;
		SemaphoreHandle_t _currentFeedbackSemaphore;

	private:
		static void CurrentControllerThread(void * pvParameters);
		static std::list<MotorDriver *> motorDrivers;

		static TaskHandle_t currentControllerTaskHandle;
		static bool currentControllerShouldStop;
		static bool currentControllerIsRunning;

		static float currentFeedbacks_[3];
		static float normalizedDuties[3];
};
	
	
#endif
