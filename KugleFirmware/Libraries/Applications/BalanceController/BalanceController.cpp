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
 
#include "BalanceController.h"
#include "cmsis_os.h"

#include "Parameters.h"
#include "Debug.h"
#include "LQR.h"
#include "QuaternionVelocityControl.h"
#include "SlidingMode.h"
#include "IIR.hpp"
#include "QEKF.h"
#include "MadgwickAHRS.h"
#include "COMEKF.h"
#include "VelocityEKF.h"
#include "Kinematics.h"
#include "Quaternion.h"

#include <string> // for memcpy

BalanceController::BalanceController(IMU& imu_, ESCON& motor1_, ESCON& motor2_, ESCON& motor3_, LSPC& com_, Timer& microsTimer_) : TaskHandle_(0), isRunning_(false), shouldStop_(false), imu(imu_), motor1(motor1_), motor2(motor2_), motor3(motor3_), com(com_), microsTimer(microsTimer_)
{
	/* Create setpoint semaphores */
	BalanceReference.semaphore = xSemaphoreCreateBinary();
	if (BalanceReference.semaphore == NULL) {
		ERROR("Could not create Quaternion reference semaphore");
		return;
	}
	vQueueAddToRegistry(BalanceReference.semaphore, "Quaternion reference");
	xSemaphoreGive( BalanceReference.semaphore ); // give the semaphore the first time

	VelocityReference.semaphore = xSemaphoreCreateBinary();
	if (VelocityReference.semaphore == NULL) {
		ERROR("Could not create Velocity reference semaphore");
		return;
	}
	vQueueAddToRegistry(VelocityReference.semaphore, "Velocity reference");
	xSemaphoreGive( VelocityReference.semaphore ); // give the semaphore the first time

	/* Register message type callbacks */
	com.registerCallback(lspc::MessageTypesFromPC::CalibrateIMU, &CalibrateIMUCallback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::RestartController, &RestartControllerCallback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::VelocityReference_Heading, &VelocityReference_Heading_Callback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::VelocityReference_Inertial, &VelocityReference_Inertial_Callback, (void *)this);

	Start();
}

BalanceController::~BalanceController()
{
	shouldStop_ = true;
	while (isRunning_) osDelay(10);

	/* Unregister message callbacks */
	com.unregisterCallback(lspc::MessageTypesFromPC::CalibrateIMU);
	com.unregisterCallback(lspc::MessageTypesFromPC::RestartController);
	com.unregisterCallback(lspc::MessageTypesFromPC::VelocityReference_Heading);
	com.unregisterCallback(lspc::MessageTypesFromPC::VelocityReference_Inertial);

	/* Delete semaphores */
	if (BalanceReference.semaphore) {
		vQueueUnregisterQueue(BalanceReference.semaphore);
		vSemaphoreDelete(BalanceReference.semaphore);
	}

	if (VelocityReference.semaphore) {
		vQueueUnregisterQueue(VelocityReference.semaphore);
		vSemaphoreDelete(VelocityReference.semaphore);
	}
}

int BalanceController::Start()
{
	if (isRunning_) return 0; // task already running
	shouldStop_ = false;
	return xTaskCreate( BalanceController::Thread, (char *)"Balance Controller", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &TaskHandle_);
}

int BalanceController::Stop(uint32_t timeout)
{
	if (!isRunning_) return 0; // task not running

	shouldStop_ = true;

	uint32_t timeout_millis = timeout;
	while (isRunning_ && timeout_millis > 0) {
		osDelay(1);
		timeout_millis--;
	}
	if (isRunning_) return -1; // timeout trying to stop task

	return 1;
}

int BalanceController::Restart(uint32_t timeout)
{
	if (!isRunning_) return 0; // task not running
	int errCode = Stop(timeout);
	if (errCode != 1) return errCode;
	return Start();
}

void BalanceController::Thread(void * pvParameters)
{
	BalanceController * balanceController = (BalanceController *)pvParameters;
	TickType_t xLastWakeTime;
	uint32_t prevTimerValue; // used for measuring dt
	balanceController->isRunning_ = true;

	/* Load initialized objects */
	IMU& imu = balanceController->imu;
	ESCON& motor1 = balanceController->motor1;
	ESCON& motor2 = balanceController->motor2;
	ESCON& motor3 = balanceController->motor3;
	LSPC& com = balanceController->com;
	Timer& microsTimer = balanceController->microsTimer;

	/* Create and load parameters */
	Parameters& params = *(new Parameters);

	/* Controller loop time / sample rate */
	TickType_t loopWaitTicks = configTICK_RATE_HZ / params.controller.SampleRate;

	/* Create and initialize controller and estimator objects */
	LQR& lqr = *(new LQR(params));
	SlidingMode& sm = *(new SlidingMode(params));
	QuaternionVelocityControl& velocityController = *(new QuaternionVelocityControl(params, &balanceController->microsTimer, 1.0f / params.controller.SampleRate, params.controller.VelocityController_ReferenceLPFtau));
	QEKF& qEKF = *(new QEKF(params, &balanceController->microsTimer));
	Madgwick& madgwick = *(new Madgwick(params.controller.SampleRate, params.estimator.MadgwickBeta));
	VelocityEKF& velocityEKF = *(new VelocityEKF(params, &balanceController->microsTimer));
	COMEKF& comEKF = *(new COMEKF(params, &balanceController->microsTimer));
	Kinematics& kinematics = *(new Kinematics(params, &balanceController->microsTimer));
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> accel_x_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> accel_y_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> accel_z_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> gyro_x_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> gyro_y_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> gyro_z_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);

	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> dx_filt(params.estimator.VelocityLPFcoeffs_a, params.estimator.VelocityLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> dy_filt(params.estimator.VelocityLPFcoeffs_a, params.estimator.VelocityLPFcoeffs_b);

	/* Measurement variables */
	IMU::Measurement_t imuRaw;
	IMU::Measurement_t imuCorrected;
	IMU::Measurement_t imuFiltered;
	int32_t EncoderTicks[3];
	float EncoderAngle[3];

	/* Estimate covariance variables */
	float Cov_q[4*4];
	float Cov_dxy[2*2];

	/* Temporary variables */
	float dxy_kinematics_old[2];

	/* Control output variables */
	float Torque[3];
	float TorqueDelivered[3];
	int MotorDriverFailureCounts[3];
	float S[3]; // Sliding surface values
	float TorqueRampUpGain = 0;
	bool TorqueRampUpFinished = false;

	/* Control output filtering objects */
	FirstOrderLPF& Motor1_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));
	FirstOrderLPF& Motor2_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));
	FirstOrderLPF& Motor3_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));

#if 0  // UNIT TESTS DISABLED
	if (!lqr.UnitTest()) {
		ERROR("LQR Unit test failed!");
	}

	if (!sm.UnitTest()) {
		ERROR("Sliding Mode Unit test failed!");
	}

	if (!qEKF.UnitTest()) {
		ERROR("qEKF Unit test failed!");
	}
#endif

	/* Reset estimators */
	imu.Get(imuCorrected);
	imu.CorrectMeasurement(imuCorrected);
	EncoderTicks[0] = motor1.GetEncoderRaw();
	EncoderTicks[1] = motor2.GetEncoderRaw();
	EncoderTicks[2] = motor3.GetEncoderRaw();
	EncoderAngle[0] = motor1.GetAngle();
	EncoderAngle[1] = motor2.GetAngle();
	EncoderAngle[2] = motor3.GetAngle();
	qEKF.Reset(imuCorrected.Accelerometer); // reset attitude estimator to current attitude, based on IMU
	madgwick.Reset(imuCorrected.Accelerometer[0], imuCorrected.Accelerometer[1], imuCorrected.Accelerometer[2]);
	velocityEKF.Reset(EncoderTicks);
	comEKF.Reset();
	kinematics.Reset(EncoderAngle);

	balanceController->StabilizeFilters(params, imu, qEKF, madgwick, loopWaitTicks, 1.0f); // stabilize estimators for 1 second

	/* Reset COM estimate */
	balanceController->COM[0] = params.model.COM_X;
	balanceController->COM[1] = params.model.COM_Y;
	balanceController->COM[2] = params.model.COM_Z;

	/* Reset position estimate */
	balanceController->xy[0] = 0;
	balanceController->xy[1] = 0;

	/* Reset reference variables */
	balanceController->q_ref[0] = 1; // attitude reference = just upright
	balanceController->q_ref[1] = 0;
	balanceController->q_ref[2] = 0;
	balanceController->q_ref[3] = 0;
	balanceController->omega_ref_inertial[0] = 0; // zero angular velocity reference
	balanceController->omega_ref_inertial[1] = 0;
	balanceController->omega_ref_inertial[2] = 0;
	balanceController->omega_ref_body[0] = 0; // zero angular velocity reference
	balanceController->omega_ref_body[1] = 0;
	balanceController->omega_ref_body[2] = 0;
	balanceController->headingReference = 0; // consider to replace this with current heading (based on estimate of stabilized QEKF filter)
	balanceController->ReferenceGenerationStep = 0; // only used if test reference generation is enabled

	/* Reset quaternion reference input */
	xSemaphoreTake( balanceController->BalanceReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating
	balanceController->BalanceReference.frame = BODY_FRAME;
	balanceController->BalanceReference.time = BODY_FRAME;
	balanceController->BalanceReference.q[0] = 1; // attitude reference = just upright
	balanceController->BalanceReference.q[1] = 0;
	balanceController->BalanceReference.q[2] = 0;
	balanceController->BalanceReference.q[3] = 0;
	balanceController->BalanceReference.omega[0] = 0;
	balanceController->BalanceReference.omega[1] = 0;
	balanceController->BalanceReference.omega[2] = 0;
	xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back

	/* Reset velocity reference input */
	xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating
	balanceController->VelocityReference.frame = BODY_FRAME;
	balanceController->VelocityReference.time = 0;
	balanceController->VelocityReference.dx = 0;
	balanceController->VelocityReference.dy = 0;
	balanceController->VelocityReference.dyaw = 0;
	xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back

	/* Reset temporary variables */
	dxy_kinematics_old[0] = 0;
	dxy_kinematics_old[1] = 0;
	S[0] = 0;
	S[1] = 0;
	S[2] = 0;

	/* Disable motor outputs and set 0 torque - motors will be enabled when the controller is running (defined by the controller mode and type) */
	motor1.SetTorque(0);
	motor2.SetTorque(0);
	motor3.SetTorque(0);
	motor1.Disable();
	motor2.Disable();
	motor3.Disable();
	MotorDriverFailureCounts[0] = 0;
	MotorDriverFailureCounts[1] = 0;
	MotorDriverFailureCounts[2] = 0;

	float volatile dt_compute, dt_compute2;

/*#pragma GCC push_options
#pragma GCC optimize("O0")
// Code here
#pragma GCC pop_options*/
/* For functions the following post attribute to the function declaration (in C/C++ file) can be made to disable optimization
__attribute__((optimize("O0")))
*/

	/* Main control loop */
	xLastWakeTime = xTaskGetTickCount();
	prevTimerValue = microsTimer.Get();
	while (!balanceController->shouldStop_) {
		/* Wait until time has been reached to make control loop periodic */
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);
		params.Refresh(); // load current parameters from global parameter object

		uint32_t timerPrev = HAL_tic();
		float dt = microsTimer.GetDeltaTime(prevTimerValue);
		prevTimerValue = microsTimer.Get();

		/* Get measurements (sample) */
		imu.Get(imuRaw);
	    /* Adjust the measurements according to the calibration */
		imuCorrected = imuRaw;
		imu.CorrectMeasurement(imuCorrected);

		imuFiltered = imuCorrected;
		if (params.estimator.EnableSoftwareLPFfilters) {
			imuFiltered.Accelerometer[0] = accel_x_filt.Filter(imuCorrected.Accelerometer[0]);
			imuFiltered.Accelerometer[1] = accel_y_filt.Filter(imuCorrected.Accelerometer[1]);
			imuFiltered.Accelerometer[2] = accel_z_filt.Filter(imuCorrected.Accelerometer[2]);
			imuFiltered.Gyroscope[0] = gyro_x_filt.Filter(imuCorrected.Gyroscope[0]);
			imuFiltered.Gyroscope[1] = gyro_y_filt.Filter(imuCorrected.Gyroscope[1]);
			imuFiltered.Gyroscope[2] = gyro_z_filt.Filter(imuCorrected.Gyroscope[2]);
		}

		EncoderTicks[0] = motor1.GetEncoderRaw();
		EncoderTicks[1] = motor2.GetEncoderRaw();
		EncoderTicks[2] = motor3.GetEncoderRaw();
		EncoderAngle[0] = motor1.GetAngle();
		EncoderAngle[1] = motor2.GetAngle();
		EncoderAngle[2] = motor3.GetAngle();

		if (params.debug.EnableRawSensorOutput) {
			if (params.debug.UseFilteredIMUinRawSensorOutput)
				balanceController->SendRawSensors(params, imuFiltered, EncoderAngle);
			else
				balanceController->SendRawSensors(params, imuCorrected, EncoderAngle);
		}

		/* Attitude estimation */
		if (params.estimator.UseMadgwick) {
			madgwick.updateIMU(imuFiltered.Gyroscope[0], imuFiltered.Gyroscope[1], imuFiltered.Gyroscope[2], imuFiltered.Accelerometer[0], imuFiltered.Accelerometer[1], imuFiltered.Accelerometer[2]);
			madgwick.getQuaternion(balanceController->q);
			madgwick.getQuaternionDerivative(balanceController->dq);
			// Hack for Madgwick quaternion estimate covariance
	        for (int m = 0; m < 4; m++) {
	          for (int n = 0; n < 4; n++) {
	        	  Cov_q[4*m + n] = 0;
	          }
	        }
	        for (int d = 0; d < 4; d++) {
	        	Cov_q[4*d + d] = 3*1E-7; // set q covariance when MADGWICK is used
	        }
		} else { // use QEKF
			qEKF.Step(imuFiltered.Accelerometer, imuFiltered.Gyroscope, params.estimator.EstimateBias);
			qEKF.GetQuaternion(balanceController->q);
			qEKF.GetQuaternionDerivative(balanceController->dq);
			qEKF.GetGyroBias(balanceController->GyroBias);
			qEKF.GetQuaternionCovariance(Cov_q);
		}

		/* Quaternion derivative LPF filtering */
	    /*dq[0] = dq0_filt.Filter(dq[0]);
	    dq[1] = dq1_filt.Filter(dq[1]);
	    dq[2] = dq2_filt.Filter(dq[2]);
	    dq[3] = dq3_filt.Filter(dq[3]);*/

		/* Independent heading requires dq to be decoupled around the yaw/heading axis */
	    if (params.behavioural.IndependentHeading && !params.behavioural.YawVelocityBraking && !params.controller.DisableQdot) {
	      float dq_tmp[4];
	      dq_tmp[0] = balanceController->dq[0];
	      dq_tmp[1] = balanceController->dq[1];
	      dq_tmp[2] = balanceController->dq[2];
	      dq_tmp[3] = balanceController->dq[3];
	      HeadingIndependentQdot(dq_tmp, balanceController->q, balanceController->dq);
	    }

	    if (params.controller.DisableQdot) {
	    	balanceController->dq[0] = 0;
	    	balanceController->dq[1] = 0;
	    	balanceController->dq[2] = 0;
	    	balanceController->dq[3] = 0;
	    }

	    /* Compute kinematics-based velocity and update position estimate by trapezoidal integration */
		float dxy_kinematics[2];
    	// compute velocity from encoder-based motor velocities and forward kinematics
    	kinematics.EstimateMotorVelocity(EncoderAngle);
		kinematics.ForwardKinematics(balanceController->q, balanceController->dq, dxy_kinematics);
		// Update position estimate by trapezoidal integration of kinematics-based velocity estimate
	    balanceController->xy[0] += (dxy_kinematics_old[0] + dxy_kinematics[0]) / (2.0f * params.controller.SampleRate);
	    balanceController->xy[1] += (dxy_kinematics_old[1] + dxy_kinematics[1]) / (2.0f * params.controller.SampleRate);
	    // Save previous velocity estimate for usage in trapezoidal integration for position estimation
	    dxy_kinematics_old[0] = dxy_kinematics[0];
	    dxy_kinematics_old[1] = dxy_kinematics[1];

	    /* If enabled, re-use the kinematics-based estimate as the velocity estimate for the controller */
	    if (!params.estimator.UseVelocityEstimator) {
			if (params.estimator.Use2Lvelocity) {
				kinematics.ConvertBallTo2Lvelocity(dxy_kinematics, balanceController->q, balanceController->dq, balanceController->dxy);            // put 2L velocity into dxy
			} else {
				// put ball velocity into dxy
				balanceController->dxy[0] = dxy_kinematics[0];
				balanceController->dxy[1] = dxy_kinematics[1];
			}

			if (params.estimator.EnableVelocityLPF) {
				balanceController->dxy[0] = dx_filt.Filter(balanceController->dxy[0]);
				balanceController->dxy[1] = dy_filt.Filter(balanceController->dxy[1]);
			}
	    }

	    /* Velocity estimation using velocity EKF */
	    if (params.estimator.UseVelocityEstimator) {
	    	velocityEKF.Step(EncoderTicks, balanceController->q, Cov_q, balanceController->dq, balanceController->COM); // velocity estimator estimates 2L velocity
	    	velocityEKF.GetVelocity(balanceController->dxy);
	    	velocityEKF.GetVelocityCovariance(Cov_dxy);

	    	// OBS. dxy was in the original design supposed to be ball velocity, but the velocity estimator estimates the 2L velocity which gives indirect "stabilization"
	    	// In the Sliding Mode controller this velocity is (only) used to calculated "feedforward" torque to counteract friction
	        /*if (!params.estimator.Use2Lvelocity) { // however if the 2L velocity is not desired, it is here converted back to ball velocity
	        	kinematics.Convert2LtoBallVelocity(balanceController->dxy, balanceController->q, balanceController->dq, balanceController->dxy);
	        }*/
	    }

	    /* Center of Mass estimation */
	    if (params.estimator.EstimateCOM && params.estimator.UseVelocityEstimator) { // can only estimate COM if velocity is also estimated (due to need of velocity estimate covariance)
	    	comEKF.Step(balanceController->dxy, Cov_dxy, balanceController->q, Cov_q, balanceController->dq);
	    	comEKF.GetCOM(balanceController->COM);
	    }

		/* Send State Estimates message */
		balanceController->SendEstimates();

		/*Debug::printf("qEKF = [%.3f, %.3f, %.3f, %.3f]\n", balanceController->q[0], balanceController->q[1], balanceController->q[2], balanceController->q[3]);
		float YPR[3];
		Quaternion_quat2eul_zyx(balanceController->q, YPR);
		Debug::printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", rad2deg(YPR[2]), rad2deg(YPR[1]), rad2deg(YPR[0]));
		//Debug::printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", microsTimer.GetTime(), imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2], imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2]);*/

	    /* Reference generation - get references */
	    balanceController->ReferenceGeneration(params); // this function updates q_ref and omega_ref

	    /* Compute internal q_ref, omega_ref_body and omega_ref_inertial based on mode and setpoints */
		/* Quaternion control enabled */
		if (params.controller.mode == lspc::ParameterTypes::QUATERNION_CONTROL) {
			// Get quaternion references
	    	if (xSemaphoreTake( balanceController->BalanceReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading
				if ((microsTimer.GetTime() - balanceController->BalanceReference.time) < params.controller.ReferenceTimeout) {
					balanceController->q_ref[0] = balanceController->BalanceReference.q[0];
					balanceController->q_ref[1] = balanceController->BalanceReference.q[1];
					balanceController->q_ref[2] = balanceController->BalanceReference.q[2];
					balanceController->q_ref[3] = balanceController->BalanceReference.q[3];

				    if (params.behavioural.IndependentHeading) {
				    	HeadingIndependentReferenceManual(balanceController->q_ref, balanceController->q, balanceController->q_ref);
				    }

					if (balanceController->BalanceReference.frame == BODY_FRAME) {
						balanceController->omega_ref_body[0] = balanceController->BalanceReference.omega[0];
						balanceController->omega_ref_body[1] = balanceController->BalanceReference.omega[1];
						balanceController->omega_ref_body[2] = balanceController->BalanceReference.omega[2];
						Quaternion_RotateVector_Body2Inertial(balanceController->q, balanceController->omega_ref_body, balanceController->omega_ref_inertial);
					}
					else if (balanceController->BalanceReference.frame == INERTIAL_FRAME) {
						balanceController->omega_ref_inertial[0] = balanceController->BalanceReference.omega[0];
						balanceController->omega_ref_inertial[1] = balanceController->BalanceReference.omega[1];
						balanceController->omega_ref_inertial[2] = balanceController->BalanceReference.omega[2];
						Quaternion_RotateVector_Inertial2Body(balanceController->q, balanceController->omega_ref_inertial, balanceController->omega_ref_body);
					}
					else { // unknown frame for quaternion control mode
						balanceController->omega_ref_body[0] = 0;
						balanceController->omega_ref_body[1] = 0;
						balanceController->omega_ref_body[2] = 0;
						balanceController->omega_ref_inertial[0] = 0;
						balanceController->omega_ref_inertial[1] = 0;
						balanceController->omega_ref_inertial[2] = 0;
					}
				}
	    		xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back
	    	}
		}

	    /* Velocity control enabled */
	    if (params.controller.mode == lspc::ParameterTypes::VELOCITY_CONTROL) {
			// Get velocity reference
	    	if (xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading
	    		if ((microsTimer.GetTime() - balanceController->VelocityReference.time) < params.controller.ReferenceTimeout) {
	    			balanceController->velocityReference[0] = balanceController->VelocityReference.dx;
	    			balanceController->velocityReference[1] = balanceController->VelocityReference.dy;
	    			balanceController->headingVelocityReference = balanceController->VelocityReference.dyaw;
	    		}
	    		xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back
	    	}

	    	balanceController->headingReference += balanceController->headingVelocityReference * dt;
	    	balanceController->omega_ref_inertial[0] = 0;
	    	balanceController->omega_ref_inertial[1] = 0;
	    	balanceController->omega_ref_inertial[2] = balanceController->headingVelocityReference;
			Quaternion_RotateVector_Inertial2Body(balanceController->q, balanceController->omega_ref_inertial, balanceController->omega_ref_body);

			if (params.behavioural.IndependentHeading) {
				balanceController->headingReference = HeadingFromQuaternion(balanceController->q);
			}

			velocityController.Step(balanceController->q, balanceController->dq, balanceController->dxy, balanceController->velocityReference, true, balanceController->headingReference, balanceController->q_ref);
	    }

	    /* Compute control output based on references */
	    if (params.controller.type == lspc::ParameterTypes::LQR_CONTROLLER && params.controller.mode != lspc::ParameterTypes::OFF) {
	    	lqr.Step(balanceController->q, balanceController->dq, balanceController->xy, balanceController->dxy, balanceController->COM, balanceController->q_ref, balanceController->omega_ref_body, Torque);
		} else if (params.controller.type == lspc::ParameterTypes::SLIDING_MODE_CONTROLLER && params.controller.mode != lspc::ParameterTypes::OFF) {
			// OBS. When running the Sliding Mode controller, inertial angular velocity reference is needed
	    	sm.Step(balanceController->q, balanceController->dq, balanceController->xy, balanceController->dxy, balanceController->COM, balanceController->q_ref, balanceController->omega_ref_inertial, Torque, S);
		} else {
			// Undefined controller mode, eg. OFF - set torque output to 0
			Torque[0] = 0;
			Torque[1] = 0;
			Torque[2] = 0;
		}

	    /* Check if any of the torque outputs is NaN - if so, turn off the outputs */
	    if (isnan(Torque[0]) || isnan(Torque[1]) || isnan(Torque[2])) {
	    	Torque[0] = 0;
	    	Torque[1] = 0;
	    	Torque[2] = 0;
	    }

	    /* Clamp the torque between configurable limits less than or equal to max output torque */
    	Torque[0] = fmax(fmin(Torque[0], params.model.SaturationTorque), -params.model.SaturationTorque);
    	Torque[1] = fmax(fmin(Torque[1], params.model.SaturationTorque), -params.model.SaturationTorque);
    	Torque[2] = fmax(fmin(Torque[2], params.model.SaturationTorque), -params.model.SaturationTorque);

	    /* Initial Torque ramp up */
	    if (params.controller.TorqueRampUp) {
	    	if (params.controller.mode == lspc::ParameterTypes::OFF) {
				TorqueRampUpGain = 0;
				TorqueRampUpFinished = false;
	    	}
	    	else if (!TorqueRampUpFinished) { // if controller is running and ramp up is not finished
	    		Torque[0] *= TorqueRampUpGain;
	    		Torque[1] *= TorqueRampUpGain;
	    		Torque[2] *= TorqueRampUpGain;

	    		TorqueRampUpGain += 1.0f / (params.controller.TorqueRampUpTime * params.controller.SampleRate); // ramp up rate
	    		if (TorqueRampUpGain >= 1.0) {
	    			TorqueRampUpFinished = true;
	    		}
	    	}
	    }

	    /* Torque output LPF filtering */
	    if (params.controller.EnableTorqueLPF && params.controller.mode != lspc::ParameterTypes::OFF) {
			Torque[0] = Motor1_LPF.Filter(Torque[0]);
			Torque[1] = Motor2_LPF.Filter(Torque[1]);
			Torque[2] = Motor3_LPF.Filter(Torque[2]);
	    } else {
	    	Motor1_LPF.Reset();
	    	Motor2_LPF.Reset();
	    	Motor3_LPF.Reset();
	    }

	    if (params.controller.mode != lspc::ParameterTypes::OFF) {
			/* Set control output */
			motor1.SetOutputTorque(Torque[0]);
			motor2.SetOutputTorque(Torque[1]);
			motor3.SetOutputTorque(Torque[2]);

	    	/* Ensure that motor drivers are enabled */
	    	motor1.Enable();
	    	motor2.Enable();
	    	motor3.Enable();

			/* Measure delivered torque feedback from ESCON drivers */
			TorqueDelivered[0] = motor1.GetAppliedOutputTorque();
			TorqueDelivered[1] = motor2.GetAppliedOutputTorque();
			TorqueDelivered[2] = motor3.GetAppliedOutputTorque();

			/* Detect motor driver failure mode - and if so, reset motor driver */
			if (params.controller.MotorFailureDetection) {
				for (int i = 0; i < 3; i++) {
					//if (fabs(Torque[i]) > 0.1 && fabs((Torque[i] / TorqueDelivered[i]) - 1.0) > params.controller.MotorFailureThreshold) {
					if (fabs(Torque[i]) > params.controller.MotorFailureThreshold && fabs(Torque[i] - TorqueDelivered[i]) > params.controller.MotorFailureThreshold) {
						MotorDriverFailureCounts[i]++;
					} else {
						MotorDriverFailureCounts[i] = 0;
					}
					if (MotorDriverFailureCounts[i] > (params.controller.SampleRate * params.controller.MotorFailureDetectionTime)) {
						MotorDriverFailureCounts[i] = 0;
						if (params.controller.StopAtMotorFailure) {
							// Disable motors
					    	motor1.Disable();
					    	motor2.Disable();
					    	motor3.Disable();

					    	// Change controller mode to OFF
					    	params.LockForChange();
					    	params.controller.mode = lspc::ParameterTypes::OFF;
					    	params.UnlockAfterChange();
						} else { // failing motor driver should be reset
							if (i == 0) {
								motor1.Disable();
								Motor1_LPF.Reset();
							}
							else if (i == 1) {
								motor2.Disable();
								Motor2_LPF.Reset();
							}
							else if (i == 2) {
								motor3.Disable();
								Motor3_LPF.Reset();
							}
						}
					}
				}
			}
	    } else {
	    	/* Controller is disabled, so disable motor drivers */
	    	motor1.Disable();
	    	motor2.Disable();
	    	motor3.Disable();

	    	/* Delivered torque can not be measured when the motors are disabled */
	    	TorqueDelivered[0] = 0;
	    	TorqueDelivered[1] = 0;
	    	TorqueDelivered[2] = 0;

	    	/* Reset failure counts */
	    	MotorDriverFailureCounts[0] = 0;
	    	MotorDriverFailureCounts[1] = 0;
	    	MotorDriverFailureCounts[2] = 0;

	    	/* Reset reference variables */
	    	balanceController->q_ref[0] = 1; // attitude reference = just upright
	    	balanceController->q_ref[1] = 0;
	    	balanceController->q_ref[2] = 0;
	    	balanceController->q_ref[3] = 0;
	    	balanceController->omega_ref_inertial[0] = 0; // zero angular velocity reference
	    	balanceController->omega_ref_inertial[1] = 0;
	    	balanceController->omega_ref_inertial[2] = 0;
	    	balanceController->omega_ref_body[0] = 0; // zero angular velocity reference
	    	balanceController->omega_ref_body[1] = 0;
	    	balanceController->omega_ref_body[2] = 0;
	    	balanceController->headingReference = 0; // consider to replace this with current heading (based on estimate of stabilized QEKF filter)
	    	balanceController->ReferenceGenerationStep = 0; // only used if test reference generation is enabled
	    }

		/* Measure compute time */
		dt_compute = microsTimer.GetDeltaTime(prevTimerValue);
		dt_compute2 = HAL_toc(timerPrev);

		/* Send controller info package */
		balanceController->SendControllerInfo(params.controller.type, params.controller.mode, Torque, dt_compute, TorqueDelivered);
		//Debug::printf("Balance controller compute time: %9.7f s\n", dt_compute);
		//Debug::printf("Applied torque: %4.2f\t%4.2f\t%4.2f\n", TorqueApplied[0], TorqueApplied[1], TorqueApplied[2]);

		/* Send IMU Log (test package) for MATH dump */
		float mathDumpArray[] = {microsTimer.GetTime(),
				imuCorrected.Accelerometer[0],
				imuCorrected.Accelerometer[1],
				imuCorrected.Accelerometer[2],
				imuCorrected.Gyroscope[0],
				imuCorrected.Gyroscope[1],
				imuCorrected.Gyroscope[2],
				imuCorrected.Magnetometer[0],
				imuCorrected.Magnetometer[1],
				imuCorrected.Magnetometer[2],
								 EncoderAngle[0],
								 EncoderAngle[1],
								 EncoderAngle[2],
								 TorqueDelivered[0],
								 TorqueDelivered[1],
								 TorqueDelivered[2],
								 balanceController->xy[0],
								 balanceController->xy[1],
								 balanceController->q[0],
								 balanceController->q[1],
								 balanceController->q[2],
								 balanceController->q[3],
								 balanceController->dxy[0],
								 balanceController->dxy[1],
								 balanceController->dq[0],
								 balanceController->dq[1],
								 balanceController->dq[2],
								 balanceController->dq[3],
								 balanceController->GyroBias[0],
								 balanceController->GyroBias[1],
								 balanceController->GyroBias[2],
								 balanceController->COM[0],
								 balanceController->COM[1],
								 balanceController->COM[2],
								 S[0],
								 S[1],
								 S[2],
								 Torque[0],
								 Torque[1],
								 Torque[2],
								 balanceController->q_ref[0],
								 balanceController->q_ref[1],
								 balanceController->q_ref[2],
								 balanceController->q_ref[3],
								 balanceController->omega_ref_body[0],
								 balanceController->omega_ref_body[1],
								 balanceController->omega_ref_body[2]
							};
		com.TransmitAsync(lspc::MessageTypesToPC::MathDump, (uint8_t *)&mathDumpArray, sizeof(mathDumpArray));
	}
	/* End of control loop */

	/* Disable motor outputs */
	motor1.Disable();
	motor2.Disable();
	motor3.Disable();

	/* Clear controller and estimator objects */
	delete(&params);
	delete(&lqr);
	delete(&sm);
	delete(&velocityController);
	delete(&qEKF);
	delete(&madgwick);
	delete(&velocityEKF);
	delete(&comEKF);
	delete(&kinematics);
	delete(&Motor1_LPF);
	delete(&Motor2_LPF);
	delete(&Motor3_LPF);

	/* Stop and delete task */
	balanceController->isRunning_ = false;
	balanceController->TaskHandle_ = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

/* Initialize/stabilize estimators for certain stabilization time */
void BalanceController::StabilizeFilters(Parameters& params, IMU& imu, QEKF& qEKF, Madgwick& madgwick, TickType_t loopWaitTicks, float stabilizationTime)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t finishTick = xLastWakeTime + configTICK_RATE_HZ * stabilizationTime;

	IMU::Measurement_t imuMeas;

	while (xLastWakeTime < finishTick) {
		// Wait until time has been reached to make control loop periodic
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);

		/* Get measurements (sample) */
		imu.Get(imuMeas);
	    /* Adjust the measurements according to the calibration */
		imu.CorrectMeasurement(imuMeas);

		// Compute attitude estimate
		if (params.estimator.UseMadgwick) {
			madgwick.updateIMU(imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2], imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2], 0.1); // use larger beta to make filter converge to current angle by trusting the accelerometer more
		} else {
			qEKF.Step(imuMeas.Accelerometer, imuMeas.Gyroscope, false); // do not estimate bias while stabilizing the filter
		}
	}
}

/* Reference generation based on selected test */
void BalanceController::ReferenceGeneration(Parameters& params)
{
    /*if (params.behavioural.JoystickVelocityControl) {
		// Get velocity references from input (eg. joystick)
		float ref_dx = 0; // default values if the reference semaphore could not be obtained
		float ref_dy = 0;
		float ref_dyaw = 0;
    	if (xSemaphoreTake( VelocityReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading
    		ref_dx = VelocityReference.dx;
    		ref_dy = VelocityReference.dy;
    		ref_dyaw = VelocityReference.dyaw;
    		xSemaphoreGive( VelocityReference.semaphore ); // give semaphore back
    	}
    }

    else if (params.behavioural.VelocityControllerEnabled) {
		float VelRef[2] = {0,0};

		if (params.behavioural.StepTestEnabled) {
			ReferenceGenerationStep++;
			if (ReferenceGenerationStep > 8*params.controller.SampleRate) // reset after 8 seconds
				ReferenceGenerationStep = 0;

			if (ReferenceGenerationStep < 4*params.controller.SampleRate) { // from 0-4 seconds
				VelRef[0] = 0;
			}
			else if (ReferenceGenerationStep < 8*params.controller.SampleRate) { // from 4-8 seconds
				VelRef[0] = 0.2;
			}
		}

		velocityController.Step(q, dq, dxy, VelRef, true, deg2rad(0.0f), q_ref);
	}
    else */
	if (params.behavioural.StepTestEnabled) {
		float quaternion_reference[4];
    	ReferenceGenerationStep++;
		if (ReferenceGenerationStep > 8*params.controller.SampleRate) // reset after 8 seconds
			ReferenceGenerationStep = 0;

		if (ReferenceGenerationStep < 2*params.controller.SampleRate) { // from 0-2 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(0), quaternion_reference);
		}
		else if (ReferenceGenerationStep < 4*params.controller.SampleRate) { // from 2-4 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(3), quaternion_reference);
		}
		else if (ReferenceGenerationStep < 6*params.controller.SampleRate) { // from 4-6 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(0), quaternion_reference);
		}
		else if (ReferenceGenerationStep < 8*params.controller.SampleRate) { // from 6-8 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(-3), quaternion_reference);
		}

		if (xSemaphoreTake( BalanceReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for updating
			/* Update references with input values from message */
			BalanceReference.time = microsTimer.GetTime();
			BalanceReference.omega[0] = 0;
			BalanceReference.omega[1] = 0;
			BalanceReference.omega[2] = 0;
			BalanceReference.q[0] = quaternion_reference[0];
			BalanceReference.q[1] = quaternion_reference[1];
			BalanceReference.q[2] = quaternion_reference[2];
			BalanceReference.q[3] = quaternion_reference[3];
			BalanceReference.frame = BODY_FRAME;
			xSemaphoreGive( BalanceReference.semaphore ); // give semaphore back
		}
    }
	else if (params.behavioural.SineTestEnabled) {
			float quaternion_reference[4];
			const float SineFrequency = 0.5; // hz
			Quaternion_eul2quat_zyx(0, 0, deg2rad(5) * sinf(2*M_PI *  (float)ReferenceGenerationStep * SineFrequency / params.controller.SampleRate), quaternion_reference);
	    	ReferenceGenerationStep++;

	    	if ((float)ReferenceGenerationStep * SineFrequency >= params.controller.SampleRate) {
	    		ReferenceGenerationStep = 0;
	    	}

			if (xSemaphoreTake( BalanceReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for updating
				/* Update references with input values from message */
				BalanceReference.time = microsTimer.GetTime();
				BalanceReference.omega[0] = deg2rad(5) * 2*M_PI*SineFrequency * cosf(2*M_PI *  (float)ReferenceGenerationStep * SineFrequency / params.controller.SampleRate);
				BalanceReference.omega[1] = 0;
				BalanceReference.omega[2] = 0;
				BalanceReference.q[0] = quaternion_reference[0];
				BalanceReference.q[1] = quaternion_reference[1];
				BalanceReference.q[2] = quaternion_reference[2];
				BalanceReference.q[3] = quaternion_reference[3];
				BalanceReference.frame = BODY_FRAME;
				xSemaphoreGive( BalanceReference.semaphore ); // give semaphore back
			}
	    }
}

void BalanceController::SendEstimates(void)
{
	lspc::MessageTypesToPC::StateEstimates_t msg;

	msg.time = microsTimer.GetTime();
	msg.q.w = q[0];
	msg.q.x = q[1];
	msg.q.y = q[2];
	msg.q.z = q[3];
	msg.dq.w = dq[0];
	msg.dq.x = dq[1];
	msg.dq.y = dq[2];
	msg.dq.z = dq[3];
	msg.pos.x = xy[0];
	msg.pos.y = xy[1];
	msg.vel.x = dxy[0];
	msg.vel.y = dxy[1];

	com.TransmitAsync(lspc::MessageTypesToPC::StateEstimates, (uint8_t *)&msg, sizeof(msg));
}

void BalanceController::SendRawSensors(Parameters& params, const IMU::Measurement_t& imuMeas, const float EncoderAngle[3])
{
	lspc::MessageTypesToPC::RawSensor_IMU_MPU9250_t imu_msg;
	lspc::MessageTypesToPC::RawSensor_Encoders_t encoders_msg;

	imu_msg.time = microsTimer.GetTime();
	imu_msg.accelerometer.x = imuMeas.Accelerometer[0];
	imu_msg.accelerometer.y = imuMeas.Accelerometer[1];
	imu_msg.accelerometer.z = imuMeas.Accelerometer[2];
	memcpy(imu_msg.accelerometer.cov, params.estimator.cov_acc_mpu, sizeof(params.estimator.cov_acc_mpu));
	imu_msg.gyroscope.x = imuMeas.Gyroscope[0];
	imu_msg.gyroscope.y = imuMeas.Gyroscope[1];
	imu_msg.gyroscope.z = imuMeas.Gyroscope[2];
	memcpy(imu_msg.gyroscope.cov, params.estimator.cov_gyro_mpu, sizeof(params.estimator.cov_gyro_mpu));
	/* ToDo: Fix when Magnetometer measurements are working */
	imu_msg.magnetometer.x = 0;//imuMeas.Magnetometer[0];
	imu_msg.magnetometer.y = 0;//imuMeas.Magnetometer[1];
	imu_msg.magnetometer.z = 0;//imuMeas.Magnetometer[2];

	encoders_msg.time = microsTimer.GetTime();
	encoders_msg.angle1 = EncoderAngle[0];
	encoders_msg.angle2 = EncoderAngle[1];
	encoders_msg.angle3 = EncoderAngle[2];

	com.TransmitAsync(lspc::MessageTypesToPC::RawSensor_IMU_MPU9250, (uint8_t *)&imu_msg, sizeof(imu_msg));
	com.TransmitAsync(lspc::MessageTypesToPC::RawSensor_Encoders, (uint8_t *)&encoders_msg, sizeof(encoders_msg));
}

void BalanceController::SendControllerInfo(const lspc::ParameterTypes::controllerType_t Type, const lspc::ParameterTypes::controllerMode_t Mode, const float Torque[3], const float ComputeTime, const float TorqueDelivered[3])
{
	lspc::MessageTypesToPC::ControllerInfo_t msg;

	msg.time = microsTimer.GetTime();
	msg.type = Type;
	msg.mode = Mode;
	msg.torque1 = Torque[0];
	msg.torque2 = Torque[1];
	msg.torque3 = Torque[2];
	msg.compute_time = ComputeTime;
	msg.delivered_torque1 = TorqueDelivered[0];
	msg.delivered_torque2 = TorqueDelivered[1];
	msg.delivered_torque3 = TorqueDelivered[2];

	com.TransmitAsync(lspc::MessageTypesToPC::ControllerInfo, (uint8_t *)&msg, sizeof(msg));
}


void BalanceController::CalibrateIMUCallback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;

	lspc::MessageTypesFromPC::CalibrateIMU_t msg;
	lspc::MessageTypesToPC::CalibrateIMUAck_t msgAck;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	/* Should both check whether the magic key in the payload is correct and that the system is not balancing - otherwise calibration can not be performed */
	if (msg.magic_key != 0x12345678) {
		// Send acknowledge back to PC
		msgAck.acknowledged = false;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::CalibrateIMUAck, (uint8_t *)&msgAck, sizeof(msgAck));
		return;
	}

	/* Check current control mode as a calibration can not be performed if the robot is balancing */
	Parameters * params = new Parameters;
	lspc::ParameterTypes::controllerMode_t mode = params->controller.mode;
	delete(params);
	if (mode != lspc::ParameterTypes::OFF) {
		// Send acknowledge back to PC
		msgAck.acknowledged = false;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::CalibrateIMUAck, (uint8_t *)&msgAck, sizeof(msgAck));
		return;
	}

	bool restartAfterCalibration = false;
	if (balanceController->isRunning_) {
		balanceController->Stop();
		restartAfterCalibration = true;
	}

	/* Send acknowledge message back to PC */
	msgAck.acknowledged = true;
	balanceController->com.TransmitAsync(lspc::MessageTypesToPC::CalibrateIMUAck, (uint8_t *)&msgAck, sizeof(msgAck));

	balanceController->imu.Calibrate(true);

	if (restartAfterCalibration) {
		Debug::print("Restarting controller in 5 seconds...\n");
		osDelay(5000);

		balanceController->Start();
	}
}


void BalanceController::RestartControllerCallback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;

	lspc::MessageTypesFromPC::RestartController_t msg;
	lspc::MessageTypesToPC::RestartControllerAck_t msgAck;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	/* Should both check whether the magic key in the payload is correct and that the system is not balancing - otherwise calibration can not be performed */
	if (msg.magic_key != 0x12345678) {
		// Send acknowledge back to PC
		msgAck.acknowledged = false;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::RestartControllerAck, (uint8_t *)&msgAck, sizeof(msgAck));
		return;
	}

	/* Check current control mode as a calibration can not be performed if the robot is balancing */
	Parameters * params = new Parameters;
	lspc::ParameterTypes::controllerMode_t mode = params->controller.mode;
	delete(params);
	if (mode != lspc::ParameterTypes::OFF) {
		// Send acknowledge back to PC
		msgAck.acknowledged = false;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::RestartControllerAck, (uint8_t *)&msgAck, sizeof(msgAck));
		return;
	}

	if (balanceController->isRunning_) {
		balanceController->Stop();

		/* Send acknowledge message back to PC */
		msgAck.acknowledged = true;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::RestartControllerAck, (uint8_t *)&msgAck, sizeof(msgAck));

		Debug::print("Restarting controller...\n");
		balanceController->Start();
	}
	else { // controller not running, so send NACK
		/* Send acknowledge message back to PC */
		msgAck.acknowledged = false;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::RestartControllerAck, (uint8_t *)&msgAck, sizeof(msgAck));

	}
}

void BalanceController::VelocityReference_Heading_Callback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;

	/*uint8_t * buffer = const_cast<uint8_t *>(payload.data());
	uint32_t length = payload.size();*/

	volatile lspc::MessageTypesFromPC::VelocityReference_Heading_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	/* Update references with input values from message */
	balanceController->VelocityReference.time = balanceController->microsTimer.GetTime();
	balanceController->VelocityReference.dx = msg.vel.x;
	balanceController->VelocityReference.dy = msg.vel.y;
	balanceController->VelocityReference.dyaw = msg.vel.yaw;
	balanceController->VelocityReference.frame = HEADING_FRAME;

	xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back
}

void BalanceController::VelocityReference_Inertial_Callback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;

	/*uint8_t * buffer = const_cast<uint8_t *>(payload.data());
	uint32_t length = payload.size();*/

	volatile lspc::MessageTypesFromPC::VelocityReference_Heading_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	/* Update references with input values from message */
	balanceController->VelocityReference.time = balanceController->microsTimer.GetTime();
	balanceController->VelocityReference.dx = msg.vel.x;
	balanceController->VelocityReference.dy = msg.vel.y;
	balanceController->VelocityReference.dyaw = msg.vel.yaw;
	balanceController->VelocityReference.frame = INERTIAL_FRAME;

	xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back
}
