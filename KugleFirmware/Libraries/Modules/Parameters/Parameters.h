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
			/* Debugging parameters end */
		} debug;

		struct behavioural_t {
			/* Behavioural parameters */
			bool IndependentHeading = false;
			bool YawVelocityBraking = false; // if independent heading is enabled and q_dot is used, then yaw velocity will be counteracted by enabling this
			bool StepTestEnabled = false;
			bool VelocityControllerEnabled = false;
			bool JoystickVelocityControl = true;
			/* Behavioural parameters end */
		} behavioural;
		
		struct controller_t {
			/* Balance Controller Tuning parameters */
			float SampleRate = 200;
			
			lspc::ParameterTypes::controllerType_t type = lspc::ParameterTypes::LQR_CONTROLLER;  // LQR_CONTROLLER or SLIDING_MODE_CONTROLLER
			lspc::ParameterTypes::controllerMode_t mode = lspc::ParameterTypes::OFF;  // OFF, QUATERNION_CONTROL, ANGULAR_VELOCITY_CONTROL, VELOCITY_CONTROL or PATH_FOLLOWING

			bool EnableTorqueLPF = true;
			float TorqueLPFtau = 0.02; // 0.005 (sliding mode)
			bool EnableTorqueSaturation = false; // saturate the torque before filtering
			float TorqueMax = 0.7; // maximum torque - related to motors and motor controller - currently maximum is 15 A = 0.8 Nm
			bool TorqueRampUp = true;
			float TorqueRampUpTime = 1.0; // seconds to ramp up Torque after initialization

			bool DisableQdot = false;

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
			float K[3] = {50, 50, 30}; // sliding manifold gain  (S = omega_inertial + K*devec*q_err)
			bool ContinousSwitching = true;
			float eta = 2.5; // switching gain
			float epsilon = 2.5;  // continous switching law : "radius" of epsilon-tube around the sliding surface, wherein the control law is linear in S


			/* LQR parameters */
			/*
			float LQR_K[3*6] = {24.4461099686484,	 0.0000000000000,	 -5.77350269189626,	 2.86355138703516,   0.00000000000000,  -0.579208874021106,
															  -12.2230549843242,	 21.1709522565575, -5.77350269189626,	-1.43177569351758,	 2.48115307232164,	-0.579208874021105,
																-12.2230549843242,	-21.1709522565575, -5.77350269189626,	-1.43177569351758,	-2.48115307232165,	-0.579208874021106};
			*/
			float LQR_K[3*6] = {22.0217540086999,	1.1248573671829e-14,	-0.182574185835055,	2.71774528131163,	1.44298962102302e-15,	-0.101363312648735,
			-11.01087700435,	19.0713984074259,	-0.182574185835056,	-1.35887264065581,	2.35481794807652,	-0.101363312648735,
			-11.01087700435,	-19.0713984074259,	-0.182574185835055,	-1.35887264065582,	-2.35481794807652,	-0.101363312648735};
			float LQR_MaxYawError = 3.0; // yaw error clamp [degrees]


			/* Velocity controller parameters */
			float VelocityController_MaxTilt	= 5.0; // max tilt that velocity controller can set [degrees]
			float VelocityController_MaxIntegralCorrection = 8.0; // max tilt integral effect can compensate with [degrees]
			float VelocityController_VelocityClamp = 0.15; // velocity clamp for the proportional gain - note that at this velocity MaxTilt will be set [meters pr. second]
			float VelocityController_IntegralGain = 0.3; // integral gain, which corresponds to the incremental compensation rate (1/gain is the number of seconds it takes the integral to reach a constant offset value)
			/* Controller Tuning parameters end */
		} controller;

		struct estimator_t {
			/* Estimator Tuning parameters */
			float SampleRate = 200;
			
			#define EnableSensorLPFfilters_ 	false
			bool EnableSensorLPFfilters = EnableSensorLPFfilters_;
			bool EnableSoftwareLPFfilters = false;
			float SoftwareLPFcoeffs_a[3] = {1.000000000000000, -1.870860377550659, 0.878777573775756};	// 20 Hz LPF
			float SoftwareLPFcoeffs_b[3] = {0.011353393934590, -0.014789591644084, 0.011353393934590};	// Created using:  [num, den] = cheby2(2,40,20/(Fs/2))
			bool CreateQdotFromQDifference = false;
			bool UseMadgwick = false;
			bool EstimateBias = true;
			bool Use2Lvelocity = true; // if velocity estimator is not used
			bool UseVelocityEstimator = true;
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
				float cov_gyro_mpu[9] = {0.2529E-03f*GyroCov_Tuning_Factor,   -0.0064E-03f*GyroCov_Tuning_Factor,    0.1981E-03f*GyroCov_Tuning_Factor,
																			 -0.0064E-03f*GyroCov_Tuning_Factor,    0.9379E-03f*GyroCov_Tuning_Factor,   -0.0038E-03f*GyroCov_Tuning_Factor,
																			  0.1981E-03f*GyroCov_Tuning_Factor,   -0.0038E-03f*GyroCov_Tuning_Factor,    1.6828E-03f*GyroCov_Tuning_Factor};
			  // LPF off
				float cov_acc_mpu[9] = {0.4273E-03f*AccelCov_Tuning_Factor,    0.0072E-03f*AccelCov_Tuning_Factor,    0.0096E-03f*AccelCov_Tuning_Factor,
														 0.0072E-03f*AccelCov_Tuning_Factor,    0.4333E-03f*AccelCov_Tuning_Factor,    0.0041E-03f*AccelCov_Tuning_Factor,
															 0.0096E-03f*AccelCov_Tuning_Factor,    0.0041E-03f*AccelCov_Tuning_Factor,    1.0326E-03f*AccelCov_Tuning_Factor};
			#endif

			float sigma2_bias = 1E-11;

			// X_QEKF = {q0, q1, q2, q3,   dq0, dq1, dq2, dq3,   gyro_bias_x, gyro_bias_y}
			float QEKF_P_init_diagonal[10] = {1E-5, 1E-5, 1E-5, 1E-7,   1E-7, 1E-7, 1E-7, 1E-7,   1E-5, 1E-5}; // initialize q3 variance lower than others, since yaw can not be estimated so we are more certain on the initial value to let gyro integration (dead-reckoning) dominate the "yaw" estimate
			float VelocityEstimator_P_init_diagonal[2] = {1E-1, 1E-1}; // initialize velocity estimator covariance
			float COMEstimator_P_init_diagonal[2] = {1E-12, 1E-12}; // initialize COM estimator covariance
			/* Estimator Tuning parameters end */
		} estimator;
		
		struct model_t {
			/* Model parameters (defined in SI units) */	
			float l = 0.35f;

			float COM_X = 0.0f;
			float COM_Y = 0.0f;
			float COM_Z = l;

			float g = 9.82f;

			float rk = 0.129f;
			float Mk = 1.46f;
			float Jk = ((2.f * Mk * rk*rk) / 3.f);

			float rw = 0.05f;
			float Mw = 0.270f;
			float i_gear = 4.3;
			float Jow = 9.f * 0.0001f;
			float Jm = 1.21f * 0.0001f;
			float Jw = (Jow + i_gear*i_gear*Jm);

			float Mb = (8.205f + 5.856f);

			float Jbx = (0.958f + Mb * l*l);
			float Jby = (0.961f + Mb * l*l);
			float Jbz = 0.31f;

			float Bvk = 0*0.001f;
			float Bvm = 0*0.001f;
			float Bvb = 0*0.001f;

			uint16_t EncoderTicksPrRev = 4*4096;	 // ticks/rev
			float TicksPrRev = i_gear * EncoderTicksPrRev;
			/* Model parameters end */	
		} model;
		
		struct test_t {
			float tmp = 10;
			float tmp2 = 100;
		} test;


	public:
		Parameters(EEPROM * eeprom = 0, LSPC * com = 0);
		~Parameters();

		void Refresh(void);
		void LockForChange(void);
		void UnlockAfterChange(void);

	private:
		void LoadParametersFromEEPROM(EEPROM * eeprom = 0);
		void AttachEEPROM(EEPROM * eeprom);
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
