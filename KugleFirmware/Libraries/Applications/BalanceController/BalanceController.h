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
#include "Priorities.h"
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
#include "MTI200.h" // for logging only, if available

class BalanceController
{
	private:
		const int THREAD_STACK_SIZE = 2500; // notice that this much stack is apparently necessary to avoid issues
		const uint32_t THREAD_PRIORITY = BALANCE_CONTROLLER_PRIORITY;

	public:
		BalanceController(IMU& imu_, ESCON& motor1_, ESCON& motor2_, ESCON& motor3_, LSPC& com_, Timer& microsTimer_, MTI200 * mti_ = 0);
		~BalanceController();

		int Start();
		int Stop(uint32_t timeout = 1000);
		int Restart(uint32_t timeout = 1000);

		void CalibrateIMU(bool calibrateAccelerometer = false);

	private:
		void ReferenceGeneration(Parameters& params);
		void StabilizeFilters(Parameters& params, IMU& imu, QEKF& qEKF, Madgwick& madgwick, TickType_t loopWaitTicks, float stabilizationTime);

	private:
		static void Thread(void * pvParameters);
		void SendEstimates(void);
		void SendRawSensors(Parameters& params, const IMU::Measurement_t& imuMeas, const float EncoderAngle[3]);
		void SendControllerInfo(const lspc::ParameterTypes::controllerType_t Type, const lspc::ParameterTypes::controllerMode_t Mode, const float Torque[3], const float ComputeTime, const float TorqueDelivered[3]);
		void SendControllerDebug(const float q_integral[4], const float velocity_kinematics[2], const float Torque[3], const float S[3]);
		static void CalibrateIMUCallback(void * param, const std::vector<uint8_t>& payload);
		static void RestartControllerCallback(void * param, const std::vector<uint8_t>& payload);
		static void QuaternionReference_Callback(void * param, const std::vector<uint8_t>& payload);
		static void AngularVelocityReference_Callback(void * param, const std::vector<uint8_t>& payload);
		static void BalanceControllerReference_Callback(void * param, const std::vector<uint8_t>& payload);
		static void VelocityReference_Callback(void * param, const std::vector<uint8_t>& payload);

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
		MTI200 * mti;

		// State estimates
		float xy[2];
		float q[4];
		float dq[4];
		float dxy[2];
		float COM[3];
		float GyroBias[3];

		// Internal or converted estimates
		float omega_body[3];

		// Internal references
		float q_ref[4];
		float q_ref_setpoint[4]; // used in QUATERNION_CONTROL mode to read reference into
		float omega_ref_body[3];
		float omega_ref_inertial[3];
		lspc::ParameterTypes::referenceFrame_t omega_ref_setpoint_frame;
		float omega_ref_setpoint[3];
		bool integrate_omega_ref_into_q_ref;
		lspc::ParameterTypes::referenceFrame_t velocityReferenceFrame;
		float velocityReference[2];
		float headingVelocityReference;
		float headingReference;
		int ReferenceGenerationStep;

		// Setpoints (settable references)
		// Consider to combine semaphores into 1 common setpoint/mode semaphore for all
		struct BalanceReference_t {
			SemaphoreHandle_t semaphore;
			lspc::ParameterTypes::referenceFrame_t frame;
			float time;
			float q[4];
			float omega[3];
			bool angularVelocityOnly; // used to indicate whether angular velocity reference should be integrated to generate quaternion reference (hence discarding the quaternion reference in this package)
		} BalanceReference;

		struct VelocityReference_t {
			SemaphoreHandle_t semaphore;
			lspc::ParameterTypes::referenceFrame_t frame;
			float time;
			float dx;
			float dy;
			float dyaw;
		} VelocityReference;

};
	
	
#endif
