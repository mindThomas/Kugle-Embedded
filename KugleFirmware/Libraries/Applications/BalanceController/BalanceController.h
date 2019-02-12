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

#ifndef APPLICATION_BALANCECONTROLLER_H
#define APPLICATION_BALANCECONTROLLER_H

#include "cmsis_os.h"
#include "Parameters.h"
#include "LSPC.hpp"
#include "ESCON.h"
#include "IMU.h"
#include "Timer.h"
#include "QuaternionVelocityControl.h"
#include "LQR.h"
#include "SlidingMode.h"
#include "QEKF.h"
#include "MadgwickAHRS.h"
#include "COMEKF.h"
#include "VelocityEKF.h"

class BalanceController
{
	private:
		const int THREAD_STACK_SIZE = 1500; // notice that this much stack is apparently necessary to avoid issues
		const uint32_t THREAD_PRIORITY = BALANCE_CONTROLLER_PRIORITY;

	public:
		typedef enum {
			BODY_FRAME,
			HEADING_FRAME,
			INERTIAL_FRAME
		} referenceFrame_t;

	public:
		BalanceController(IMU& imu_, ESCON& motor1_, ESCON& motor2_, ESCON& motor3_, LSPC& com_, Timer& microsTimer_);
		~BalanceController();

		int Start();
		int Stop(uint32_t timeout = 1000);
		int Restart(uint32_t timeout = 1000);

		void CalibrateIMU(void);

	private:
		void ReferenceGeneration(Parameters& params, QuaternionVelocityControl& velocityController);
		void StabilizeFilters(Parameters& params, IMU& imu, QEKF& qEKF, Madgwick& madgwick, TickType_t loopWaitTicks, float stabilizationTime);

	private:
		static void Thread(void * pvParameters);
		void SendEstimates(void);
		void SendRawSensors(Parameters& params, const IMU::Measurement_t& imuMeas, const float EncoderAngle[3]);
		void SendControllerInfo(const lspc::ParameterTypes::controllerType_t Type, const lspc::ParameterTypes::controllerMode_t Mode, const float Torque[3], const float ComputeTime, const float TorqueDelivered[3]);
		static void CalibrateIMUCallback(void * param, const std::vector<uint8_t>& payload);
		static void VelocityReference_Heading_Callback(void * param, const std::vector<uint8_t>& payload);
		static void VelocityReference_Inertial_Callback(void * param, const std::vector<uint8_t>& payload);

	private:
		TaskHandle_t TaskHandle_;
		bool isRunning_;
		bool shouldStop_;

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
		float GyroBias[2];

		// Internal references
		float q_ref[4];
		float omega_ref_body[3];
		float omega_ref_inertial[3];
		float velocityReference[2];
		float headingVelocityReference;
		float headingReference;
		int ReferenceGenerationStep;

		// Setpoints (settable references)
		// Consider to combine semaphores into 1 common setpoint/mode semaphore for all
		struct BalanceReference_t {
			SemaphoreHandle_t semaphore;
			referenceFrame_t frame;
			float q[4];
			float omega[3];
		} BalanceReference;

		struct VelocityReference_t {
			SemaphoreHandle_t semaphore;
			referenceFrame_t frame;
			float time;
			float dx;
			float dy;
			float dyaw;
		} VelocityReference;

};
	
	
#endif
