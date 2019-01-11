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
 
#ifndef APPLICATION_ATTITUDECONTROLLER_H
#define APPLICATION_ATTITUDECONTROLLER_H

#include "cmsis_os.h"
#include "LSPC.hpp"
#include "ESCON.h"
#include "IMU.h"
#include "Timer.h"
#include "Parameters.h"
#include "QuaternionVelocityControl.h"
#include "Priorities.h"

class AttitudeController
{
	private:
		const int THREAD_STACK_SIZE = 1500; // notice that this much stack is apparently necessary to avoid issues
		const uint32_t THREAD_PRIORITY = ATTITUDE_CONTROLLER_PRIORITY;

	public:
		AttitudeController(IMU& imu_, ESCON& motor1_, ESCON& motor2_, ESCON& motor3_, LSPC& com_, Timer& microsTimer_);
		~AttitudeController();

		int Start();
		int Stop(uint32_t timeout = 1000);
		int Restart(uint32_t timeout = 1000);

		void SetReference(const float q_ref_[4], const float omega_ref_[3]);
		void SetReference(const float omega_ref_[3]);

		void CalibrateIMU(void);

	private:
		void ReferenceGeneration(Parameters& params, QuaternionVelocityControl& velocityController);

	private:
		static void Thread(void * pvParameters);
		static void CalibrateIMUCallback(void * param, const std::vector<uint8_t>& payload);

	private:
		TaskHandle_t _TaskHandle;
		bool _isRunning;
		bool _shouldStop;

	private:
		IMU& imu;
		ESCON& motor1;
		ESCON& motor2;
		ESCON& motor3;
		LSPC& com;
		Timer& microsTimer;

		// State estimates
		float xy[2];
		float q[4];
		float dq[4];
		float dxy[2];
		float COM[3];

		// For references
		bool PropagateQuaternionReference; // if only omega_ref is set, then propagate quaternion reference based on this angular velocity reference
		float q_ref[4];
		float omega_ref[3];
		float headingReference;
		int ReferenceGenerationStep;
		uint32_t prevTimerValue;

};
	
	
#endif
