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

#ifndef MODULES_PARAMETERS_H
#define MODULES_PARAMETERS_H

#include "stm32h7xx_hal.h"
#include "ThreadSafeParameter.hpp"
#include "ESCON.h"
#include "EEPROM.h"
#include "LSPC.hpp"

#define PARAMETERS_LENGTH 	((uint32_t)&paramsGlobal->eeprom_ - (uint32_t)&paramsGlobal->ForceDefaultParameters)

class Parameters
{
	public:	
		bool ForceDefaultParameters = true; // always load the default parameters listed below, no matter what is stored in EEPROM
		uint16_t ParametersSize = 0;

		struct debug_t {
			/* Debugging parameters */
			bool EnableLogOutput = false;
			bool EnableRawSensorOutput = true;
			bool UseFilteredIMUinRawSensorOutput = true;
			/* Debugging parameters end */
		} debug;

		struct behavioural_t {
			/* Behavioural parameters */
			bool IndependentHeading = true;
			bool YawVelocityBraking = false; // if independent heading is enabled and q_dot is used, then yaw velocity will be counteracted by enabling this
			bool StepTestEnabled = false;
			bool SineTestEnabled = false;
			lspc::ParameterTypes::powerButtonMode_t PowerButtonMode = lspc::ParameterTypes::START_STOP_QUATERNION_CONTROLLER;
			/* Behavioural parameters end */
		} behavioural;
		
		struct controller_t {
			/* Balance Controller Tuning parameters */
			float SampleRate = 200;
			
			lspc::ParameterTypes::controllerType_t type = lspc::ParameterTypes::SLIDING_MODE_CONTROLLER;  // LQR_CONTROLLER or SLIDING_MODE_CONTROLLER
			lspc::ParameterTypes::controllerMode_t mode = lspc::ParameterTypes::OFF;  // OFF, QUATERNION_CONTROL, ANGULAR_VELOCITY_CONTROL, VELOCITY_CONTROL or PATH_FOLLOWING

			bool EnableTorqueLPF = false;
			float TorqueLPFtau = 0.005; // 0.005 (sliding mode)
			bool TorqueRampUp = true;
			float TorqueRampUpTime = 1.0; // seconds to ramp up Torque after initialization

			bool MotorFailureDetection = true; // detect ESCON motor driver failures (due to current overload, above nominal, for prolonged time)
			float MotorFailureDetectionTime = 0.05; // 50 ms response time for detecting motor failure until the motor driver is reset
			//float MotorFailureThreshold = 0.1; // 10% difference between torque setpoint and delivered torque for longer than MotorFailureDetectionTime will trigger the motor failure event
			float MotorFailureThreshold = 0.5; // more than 0.5 Nm difference between setpoint/requested torque and delivered torque is detected as a failure
			bool StopAtMotorFailure = true; // determines what action to take at failure: the controller should stop (require manual start) or the motor driver should automatically be reset

			bool DisableQdot = false;
			float ReferenceTimeout = 0.1; // if reference is older than 100 ms, do not use it!

			/* Sliding Mode parameters */
			/*float K[3] = {20, 20, 20}; // sliding manifold gain  (S = omega_inertial + K*devec*q_err)
			float eta = 1; // switching gain
			float epsilon = 0.1;  // continous switching law : "radius" of epsilon-tube around the sliding surface, wherein the control law is linear in S*/

			/*float K[3] = {20, 20, 3}; // sliding manifold gain  (S = omega_inertial + K*devec*q_err)
			float eta = 10; // switching gain
			float epsilon = 2;  // continous switching law : "radius" of epsilon-tube around the sliding surface, wherein the control law is linear in S*/
			// u = tau_eq + tau_switching
			// tau_switching = -eta * sat(S/epsilon)
			// In linear region (|S| < epsilon) this turns into
			// tau_switching_linear = -eta/epsilon * S
			// With a maximum torque of 0.8
			float K[3] = {20, 20, 20}; // sliding manifold gain  (S = omega_inertial + K*devec*q_err)
			bool ContinousSwitching = true;
			float eta = 3.0; // switching gain
			float epsilon = 0.6;  // continous switching law : "radius" of epsilon-tube around the sliding surface, wherein the control law is linear in S


			/* LQR parameters */
			/*
			float LQR_K[3*6] = {24.4461099686484,	 0.0000000000000,	 -5.77350269189626,	 2.86355138703516,   0.00000000000000,  -0.579208874021106,
							   -12.2230549843242,	 21.1709522565575,   -5.77350269189626,	-1.43177569351758,	 2.48115307232164,	-0.579208874021105,
							   -12.2230549843242,	-21.1709522565575,   -5.77350269189626,	-1.43177569351758,	-2.48115307232165,	-0.579208874021106};
			*/
			/*float LQR_K[3*6] = {  // Fairly well working gains for Kugle V1
					// Q = diag([200 200 3 1 1 0.1]);
					// R = 0.1 * diag([1 1 1]);
					50.2853305369288,	2.29267873313278e-14,	-3.16227766016839,	5.28615222444946,	2.68374861500103e-15,	-0.62962764938032,
					-25.1426652684644,	43.5483736826777,	-3.16227766016839,	-2.64307611222473,	4.58265789413395,	-0.62962764938032,
					-25.1426652684644,	-43.5483736826776,	-3.16227766016838,	-2.64307611222473,	-4.58265789413395,	-0.62962764938032
			};*/

			float LQR_K[3*8] = {
					103.108742162512,	-1.76733821457925e-13,	-2.5819888974716,	-1.35358797102508e-14,	-2.58198889747157,	18.9597239445678,	-4.09220484536091e-14,	-0.56174108216915,
					-51.5543710812559,	89.3044611421588,	-2.58198889747159,	2.23606797749985,	1.29099444873579,	-9.4798619722839,	16.4430776741983,	-0.561741082169149,
					-51.5543710812559,	-89.3044611421589,	-2.5819888974716,	-2.23606797749985,	1.29099444873577,	-9.47986197228385,	-16.4430776741983,	-0.561741082169151
			};
			float LQR_MaxYawError = 10.0; // yaw error clamp [degrees]
			//float LQR_MaxYawVelocityError = 0.1; // rad/s
			//float LQR_MaxAngleError = 5.0; // general clamp angle [degrees]    (after applying yaw clamp)
			bool LQR_EnableSteadyStateTorque = true; // use steady state torque based on reference

			/* Velocity controller parameters */
			float VelocityController_MaxTilt	= 8.0; // max tilt that velocity controller can set [degrees]
			float VelocityController_MaxIntegralCorrection = 8.0; // max tilt integral effect can compensate with [degrees]
			float VelocityController_VelocityClamp = 0.50; // velocity clamp for the proportional gain - note that at this velocity MaxTilt will be set [meters pr. second]
			float VelocityController_IntegralGain = 0*0.1; // integral gain, which corresponds to the incremental compensation rate (1/gain is the number of seconds it takes the integral to reach a constant offset value)
			float VelocityController_ReferenceLPFtau = 0.1; // time-constant for low pass filter on velocity reference input
			/* Controller Tuning parameters end */
		} controller;

		struct estimator_t {
			/* Estimator Tuning parameters */
			float SampleRate = 200;
			
			bool UseXsensIMU = false;
			bool ConfigureXsensIMUatBoot = false;
			bool UseXsensEstimates = false; // should the orientation estimates computed by the Xsens IMU be used or should the calibrated sensor values be used

			#define EnableSensorLPFfilters_ 	false
			bool EnableSensorLPFfilters = EnableSensorLPFfilters_;
			bool EnableSoftwareLPFfilters = false;
			float SoftwareLPFcoeffs_a[3] = {1.000000000000000, -1.870860377550659, 0.878777573775756};	// 20 Hz LPF
			float SoftwareLPFcoeffs_b[3] = {0.011353393934590, -0.014789591644084, 0.011353393934590};	// Created using:  [num, den] = cheby2(2,40,20/(Fs/2))
			bool CreateQdotFromQDifference = false;
			bool UseMadgwick = false;
			bool EstimateBias = true;
			bool Use2Lvelocity = true;
			bool UseVelocityEstimator = false;
			bool EnableVelocityLPF = true;
			float VelocityLPFcoeffs_a[3] = {1.000000000000000, -1.870860377550659, 0.878777573775756};	// 20 Hz LPF
			float VelocityLPFcoeffs_b[3] = {0.011353393934590, -0.014789591644084, 0.011353393934590};	// Created using:  [num, den] = cheby2(2,40,20/(Fs/2))
			bool UseCOMestimateInVelocityEstimator = false;
			bool EstimateCOM = false;
			float EstimateCOMminVelocity = 0.05; // minimum velocity (checked against estimate) to run COM estimator
			float MaxCOMDeviation = 0.01; // maximum tolerated COM (XY) deviation estimated by COM estimator (given in meters)

			float MadgwickBeta = 0.02; // 0.02  accelerometer influence magnitude on qDot - the smaller the less accelerometer correction
											  // OBS. Depending on the accelerometer LPF this, increasing this value might feel like the system becomes less agressive,
											  //      since it is weighting the LPF filtered accelerometer more

			// Use these tuning parameters to trust the accelerometer or gyroscope more than the other - eg. to reduce trust in the accelerometer due to induced vibrational noise
			float GyroCov_Tuning_Factor = 1.0;
			float AccelCov_Tuning_Factor = 1.0;

			#if EnableSensorLPFfilters_
				// 250 Hz LPF
				float cov_gyro_mpu[9] = {0.5041E-05f*GyroCov_Tuning_Factor,    0.0094E-05f*GyroCov_Tuning_Factor,    0.0165E-05f*GyroCov_Tuning_Factor,
															0.0094E-05f*GyroCov_Tuning_Factor,    0.5200E-05f*GyroCov_Tuning_Factor,    0.0071E-05f*GyroCov_Tuning_Factor,
															0.0165E-05f*GyroCov_Tuning_Factor,    0.0071E-05f*GyroCov_Tuning_Factor,    0.6499E-05f*GyroCov_Tuning_Factor};
				// 92 Hz LPF
				float cov_acc_mpu[9] = {0.2155E-03f*AccelCov_Tuning_Factor,    0.0056E-03f*AccelCov_Tuning_Factor,    0.0033E-03f*AccelCov_Tuning_Factor,
																		 0.0056E-03f*AccelCov_Tuning_Factor,    0.2247E-03f*AccelCov_Tuning_Factor,    0.0018E-03f*AccelCov_Tuning_Factor,
																		 0.0033E-03f*AccelCov_Tuning_Factor,    0.0018E-03f*AccelCov_Tuning_Factor,    0.5446E-03f*AccelCov_Tuning_Factor};
			#else
				// LPF off
				float cov_gyro_mpu[9] = {0.000488851271107,   0.000019498684537,   0.000017952156469,
						   	   	   	     0.000019498684537,   0.000173518012650,  -0.000003619560060,
										 0.000017952156469,  -0.000003619560060,   0.001417022105170};
			    // LPF off
				float cov_acc_mpu[9] = {0.000410857497899,   0.000005195881332,  -0.000007891794621,
						   	   	   	    0.000005195881332,   0.000422554198050,  -0.000004392151709,
									   -0.000007891794621,  -0.000004392151709,   0.001003932854130};
			#endif

			float sigma2_bias = 1E-6;
			float sigma2_omega = 1E-6;
			float sigma2_heading = 3.3846e-05; // 3*sigma == 1 degree

			// X_QEKF = {q0, q1, q2, q3,   dq0, dq1, dq2, dq3,   gyro_bias_x, gyro_bias_y}
			float QEKF_P_init_diagonal[11] = {1E-5, 1E-5, 1E-5, 1E-7,   1E-7, 1E-7, 1E-7,   1E-5, 1E-5, 1E-5}; // initialize q3 variance lower than others, since yaw can not be estimated so we are more certain on the initial value to let gyro integration (dead-reckoning) dominate the "yaw" estimate
			float VelocityEstimator_P_init_diagonal[2] = {1E-1, 1E-1}; // initialize velocity estimator covariance
			float COMEstimator_P_init_diagonal[2] = {1E-12, 1E-12}; // initialize COM estimator covariance
			/* Estimator Tuning parameters end */
		} estimator;
		
		struct model_t {
			/* Model parameters (defined in SI units) */
			double pi = 3.14159265358979323846264338327950288;
			float g = 9.82f;

			// Ball constants
			float rk = 0.129f;
			float Mk = 1.46f;
			float coating = 4e-3; // 4 mm rubber coating around ball
			float Jk = ((2.f * Mk * (rk-coating)*(rk-coating)) / 3.f);			

			// Body center of mass defined with origin in ball center
			/*
			float COM_X = -0.02069e-3;
			float COM_Y = -3.20801e-3;
			float COM_Z = 550.23854e-3 - rk; // subtract rk since the values are extracted from OnShape with origin in contact point (bottom of ball)
			*/

			float l = 0.4213f; // norm(COM)

			float COM_X = 0;
			float COM_Y = 0;
			float COM_Z = l;

			// Center of rotation - used for velocity estimation
			float CoR = 0.8720f;

			// Body constants
			float Mb = (12.892f + 1.844f);
			float Jbx = 3.9096f;
			float Jby = 3.9212f;
			float Jbz = 0.1004f;
		
			// Wheel and motor physical constants
			float rw = 0.05f;
			float Mw = 0.270f;
			float i_gear = 13.0f / 3; // gear ratio = 4.3 : 1   (https://www.maxonmotor.com/maxon/view/product/223081)
			float Jow = 9.f * 1E-4;
			float Jm = 1.21f * 1E-4;
			float Jw = (Jow + i_gear*i_gear*Jm);

			// Friction constants
			float Bvk = 0*0.001f;
			float Bvm = 0*0.001f;
			float Bvb = 0*0.001f;
			
			// Encoder constants
			uint16_t EncoderTicksPrRev = 4*4096;	 // ticks/rev
			float TicksPrRev = i_gear * EncoderTicksPrRev;
			
			// ESCON motor parameters
			float MotorMaxCurrent = 15; // ESCON 50/5 motor driver (https://www.maxonmotor.com/maxon/view/product/control/4-Q-Servokontroller/438725)
			float MotorTorqueConstant = 30.5e-3; // Nm / A   (https://www.maxonmotor.com/maxon/view/product/412819)
			float MotorMaxTorque = MotorTorqueConstant * MotorMaxCurrent; // Nm
			float MotorMaxSpeed = 6000 * 2 * pi / 60;  // rad/s of motor (before gearing)  ==  6000 rpm
			float MaxOutputTorque = i_gear * MotorMaxTorque;
			float SaturationTorque = 0.95f * MaxOutputTorque;
			/* Model parameters end */	
		} model;
		
		struct test_t {
			float tmp = 10;
			float tmp2 = 100;
		} test;


	public:
		Parameters(EEPROM * eeprom = 0, LSPC * com = 0);
		~Parameters();

		void AttachEEPROM(EEPROM * eeprom);
		void AttachLSPC(LSPC * com);

		void Refresh(void);
		void LockForChange(void);
		void UnlockAfterChange(void);

		uint32_t getParameterSizeBytes();

	private:
		void LoadParametersFromEEPROM(EEPROM * eeprom = 0);
		void StoreParameters(void); // stores to EEPROM
		void LookupParameter(uint8_t type, uint8_t param, void ** paramPtr, lspc::ParameterLookup::ValueType_t& valueType, uint8_t& arraySize);

		static void GetParameter_Callback(void * param, const std::vector<uint8_t>& payload);
		static void SetParameter_Callback(void * param, const std::vector<uint8_t>& payload);
		static void StoreParameters_Callback(void * param, const std::vector<uint8_t>& payload);
		static void DumpParameters_Callback(void * param, const std::vector<uint8_t>& payload);

	private:
		EEPROM * eeprom_;
		LSPC * com_;
		SemaphoreHandle_t readSemaphore_;
		SemaphoreHandle_t writeSemaphore_;
		uint32_t changeCounter_;
};
	
#endif
