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
 
#include "BalanceController.h"
#include "cmsis_os.h"

#include "Parameters.h"
#include "Debug.h"
#include "LQR.h"
#include "SlidingMode.h"
#include "FeedbackLinearization.h"
#include "QuaternionVelocityControl.h"
#include "VelocityLQR.h"
#include "IIR.hpp"
#include "QEKF.h"
#include "MadgwickAHRS.h"
#include "COMEKF.h"
#include "VelocityEKF.h"
#include "Kinematics.h"
#include "WheelSlipDetector.h"
#include "Quaternion.h"

#include <string> // for memcpy
#include <cmath> // for fminf, fmaxf

BalanceController::BalanceController(IMU& imu_, ESCON& motor1_, ESCON& motor2_, ESCON& motor3_, LSPC& com_, Timer& microsTimer_, MTI200 * mti_) : TaskHandle_(0), isRunning_(false), shouldStop_(false), imu(imu_), motor1(motor1_), motor2(motor2_), motor3(motor3_), com(com_), microsTimer(microsTimer_), mti(mti_)
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
	com.registerCallback(lspc::MessageTypesFromPC::QuaternionReference, &QuaternionReference_Callback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::AngularVelocityReference, &AngularVelocityReference_Callback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::BalanceControllerReference, &BalanceControllerReference_Callback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::VelocityReference, &VelocityReference_Callback, (void *)this);

	Start();
}

BalanceController::~BalanceController()
{
	shouldStop_ = true;
	while (isRunning_) osDelay(10);

	/* Unregister message callbacks */
	com.unregisterCallback(lspc::MessageTypesFromPC::CalibrateIMU);
	com.unregisterCallback(lspc::MessageTypesFromPC::RestartController);
	com.unregisterCallback(lspc::MessageTypesFromPC::QuaternionReference);
	com.unregisterCallback(lspc::MessageTypesFromPC::AngularVelocityReference);
	com.unregisterCallback(lspc::MessageTypesFromPC::BalanceControllerReference);
	com.unregisterCallback(lspc::MessageTypesFromPC::VelocityReference);

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
	float timestamp, dt_compute, dt_compute2;
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
	FeedbackLinearization& fbl = *(new FeedbackLinearization(params));
	QuaternionVelocityControl& velocityController = *(new QuaternionVelocityControl(params, &balanceController->microsTimer, 1.0f / params.controller.SampleRate));
	VelocityLQR& velocityLQR = *(new VelocityLQR(params, &balanceController->microsTimer, 1.0f / params.controller.SampleRate));
	QEKF& qEKF = *(new QEKF(params, &balanceController->microsTimer));
	Madgwick& madgwick = *(new Madgwick(params.controller.SampleRate, params.estimator.MadgwickBeta));
	VelocityEKF& velocityEKF = *(new VelocityEKF(params, &balanceController->microsTimer));
	COMEKF& comEKF = *(new COMEKF(params, &balanceController->microsTimer));
	Kinematics& kinematics = *(new Kinematics(params, &balanceController->microsTimer));
	WheelSlipDetector& wheelSlipDetector = *(new WheelSlipDetector(params, &balanceController->microsTimer));
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
	IMU::Estimates_t imuEstimates;
	MTI200::LastMeasurement_t MTImeas; // for logging
	IMU::Estimates_t MTIest;
	int32_t EncoderTicks[3];
	float EncoderAngle[3];

	/* Estimate covariance variables */
	float Cov_q[4*4];
	float Cov_dq[4*4];
	float Cov_bias[3*3];
	float Cov_dxy[2*2];
	float Cov_COM[2*2];

	/* Temporary variables */
	float dxy_kinematics_old[2];
	float q_tilt_integral[4]; // for logging of velocity controller

	/* Control output variables */
	float Torque[3];
	float TorqueDelivered[3];
	float WheelSlipRampGain = 1.0; // used for disabling during wheel slip
	float EquivalentControlPct = 1.0;
	int MotorDriverFailureCounts[3];
	float S[3]; // Sliding surface values
	float TorqueRampUpGain = 0;
	bool TorqueRampUpFinished = false;

	/* Control output filtering objects */
	FirstOrderLPF& Motor1_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));
	FirstOrderLPF& Motor2_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));
	FirstOrderLPF& Motor3_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));

#if 0 // UNIT TESTS DISABLED
	if (!fbl.UnitTest()) {
		ERROR("FBL Unit test failed!");
	}

/*
	if (!lqr.UnitTest()) {
		ERROR("LQR Unit test failed!");
	}

	if (!sm.UnitTest()) {
		ERROR("Sliding Mode Unit test failed!");
	}

	if (!qEKF.UnitTest()) {
		ERROR("qEKF Unit test failed!");
	}
*/
#endif

	/* Reset estimators */
	imu.Get(imuCorrected);
	imu.CorrectMeasurement(imuCorrected, !params.estimator.UseXsensIMU, !params.estimator.UseXsensIMU, !params.estimator.UseXsensIMU); // do not correct gyroscope bias if Xsens IMU is used (since this is corrected internally by the Xsens Kalman filter)
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

	balanceController->StabilizeFilters(params, imu, qEKF, madgwick, velocityEKF, loopWaitTicks, 1.0f); // stabilize estimators for 1 second

	/* Reset COM estimate */
	balanceController->COM[0] = params.model.COM_X;
	balanceController->COM[1] = params.model.COM_Y;
	balanceController->COM[2] = params.model.COM_Z;

	/* Reset position estimate */
	balanceController->xy[0] = 0;
	balanceController->xy[1] = 0;

	/* Reset reference variables */
	HeadingQuaternion(balanceController->q, balanceController->q_ref); // attitude reference = upright with current heading
	balanceController->q_ref_setpoint[0] = balanceController->q_ref[0];
	balanceController->q_ref_setpoint[1] = balanceController->q_ref[1];
	balanceController->q_ref_setpoint[2] = balanceController->q_ref[2];
	balanceController->q_ref_setpoint[3] = balanceController->q_ref[3];
	balanceController->omega_ref_inertial[0] = 0; // zero angular velocity reference
	balanceController->omega_ref_inertial[1] = 0;
	balanceController->omega_ref_inertial[2] = 0;
	balanceController->omega_ref_body[0] = 0; // zero angular velocity reference
	balanceController->omega_ref_body[1] = 0;
	balanceController->omega_ref_body[2] = 0;
	balanceController->omega_ref_setpoint_frame = lspc::ParameterTypes::UNKNOWN_FRAME;
	balanceController->omega_ref_setpoint[0] = 0;
	balanceController->omega_ref_setpoint[1] = 0;
	balanceController->omega_ref_setpoint[2] = 0;
	balanceController->integrate_omega_ref_into_q_ref = false;
	balanceController->headingReference = HeadingFromQuaternion(balanceController->q);
	balanceController->headingVelocityReference = 0;
	balanceController->velocityReferenceFrame = lspc::ParameterTypes::UNKNOWN_FRAME;
	balanceController->velocityReference[0] = 0;
	balanceController->velocityReference[1] = 0;
	balanceController->velocityReference_inertial[0] = 0;
	balanceController->velocityReference_inertial[1] = 0;
	balanceController->ReferenceGenerationStep = 0; // only used if test reference generation is enabled

	/* Reset quaternion reference input */
	xSemaphoreTake( balanceController->BalanceReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating
	balanceController->BalanceReference.frame = lspc::ParameterTypes::BODY_FRAME;
	balanceController->BalanceReference.time = 0;
	HeadingQuaternion(balanceController->q, balanceController->BalanceReference.q); // attitude reference = upright with current heading
	balanceController->BalanceReference.omega[0] = 0;
	balanceController->BalanceReference.omega[1] = 0;
	balanceController->BalanceReference.omega[2] = 0;
	balanceController->BalanceReference.angularVelocityOnly = false;
	xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back

	/* Reset velocity reference input */
	xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating
	balanceController->VelocityReference.frame = lspc::ParameterTypes::BODY_FRAME;
	balanceController->VelocityReference.time = 0;
	balanceController->VelocityReference.dx = 0;
	balanceController->VelocityReference.dy = 0;
	balanceController->VelocityReference.dyaw = 0;
	xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back

	/* Reset temporary variables */
	dxy_kinematics_old[0] = 0;
	dxy_kinematics_old[1] = 0;
	q_tilt_integral[0] = 1;
	q_tilt_integral[1] = 0;
	q_tilt_integral[2] = 0;
	q_tilt_integral[3] = 0;
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
	EquivalentControlPct = 1.0;
	WheelSlipRampGain = 1.0;

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
		timestamp = microsTimer.GetTime();

		/* Get measurements (sample) */
		imu.Get(imuRaw);
		/* Adjust the measurements according to the calibration */
		imuCorrected = imuRaw;
		imu.CorrectMeasurement(imuCorrected, !params.estimator.UseXsensIMU, !params.estimator.UseXsensIMU, !params.estimator.UseXsensIMU); // do not correct gyroscope bias if Xsens IMU is used (since this is corrected internally by the Xsens Kalman filter)

		imuFiltered = imuCorrected;
		if (params.estimator.EnableSoftwareLPFfilters && !params.estimator.UseXsensIMU) {
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
			if (params.estimator.UseHeadingEstimateFromXsensIMU && balanceController->mti) {
				balanceController->mti->GetEstimates(imuEstimates);
				float Xsens_Heading = HeadingFromQuaternion(imuEstimates.q);
				qEKF.Step(imuFiltered.Accelerometer, imuFiltered.Gyroscope, Xsens_Heading, params.estimator.EstimateBias);
			} else {
				qEKF.Step(imuFiltered.Accelerometer, imuFiltered.Gyroscope, params.estimator.EstimateBias);
			}
			qEKF.GetQuaternion(balanceController->q);
			qEKF.GetQuaternionDerivative(balanceController->dq);
			qEKF.GetGyroBias(balanceController->GyroBias);
			qEKF.GetQuaternionCovariance(Cov_q);
			qEKF.GetQuaternionDerivativeCovariance(Cov_dq);
			qEKF.GetBiasCovariance(Cov_bias);
		}

		/* Replace estimates with Xsens estimates */
		if (params.estimator.UseXsensIMU && params.estimator.UseXsensQuaternionEstimate) {
			imu.GetEstimates(imuEstimates);
			/* Extract estimated angular velocity before replacing quaternion with Xsens estimate */
			Quaternion_GetAngularVelocity_Body(balanceController->q, balanceController->dq, balanceController->omega_body);

			/* Replace quaternion estimate with Xsens estimate */
			balanceController->q[0] = imuEstimates.q[0];
			balanceController->q[1] = imuEstimates.q[1];
			balanceController->q[2] = imuEstimates.q[2];
			balanceController->q[3] = imuEstimates.q[3];

			/* Recompute quaternion derivative based on angular velocity estimate and newly updated quaternion estimate */
			Quaternion_GetDQ_FromBody(balanceController->q, balanceController->omega_body, balanceController->dq);
		}

		Quaternion_GetAngularVelocity_Body(balanceController->q, balanceController->dq, balanceController->omega_body);

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
		if (params.estimator.PositionEstimateDefinedInCoR) {
			kinematics.ConvertBallToCoRvelocity(dxy_kinematics, balanceController->q, balanceController->dq, dxy_kinematics);
		}
		// Update position estimate by trapezoidal integration of kinematics-based velocity estimate
		balanceController->xy[0] += (dxy_kinematics_old[0] + dxy_kinematics[0]) / (2.0f * params.controller.SampleRate);
		balanceController->xy[1] += (dxy_kinematics_old[1] + dxy_kinematics[1]) / (2.0f * params.controller.SampleRate);
		// Save previous velocity estimate for usage in trapezoidal integration for position estimation
		dxy_kinematics_old[0] = dxy_kinematics[0];
		dxy_kinematics_old[1] = dxy_kinematics[1];

		/* If enabled, re-use the kinematics-based estimate as the velocity estimate for the controller */
		if (!params.estimator.UseVelocityEstimator) {
			if (params.estimator.UseCoRvelocity && !params.estimator.PositionEstimateDefinedInCoR) {
				kinematics.ConvertBallToCoRvelocity(dxy_kinematics, balanceController->q, balanceController->dq, balanceController->dxy);            // put CoR velocity into dxy
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
			velocityEKF.Step(EncoderTicks, imuFiltered.Accelerometer, balanceController->q, Cov_q, balanceController->dq); // velocity estimator estimates the velocity of the center of the ball
			velocityEKF.GetVelocity(balanceController->dxy);
			velocityEKF.GetVelocityCovariance(Cov_dxy);
		}

		/* Center of Mass estimation */
		if (params.estimator.EstimateCOM && params.estimator.UseVelocityEstimator) { // can only estimate COM if velocity is also estimated (due to need of velocity estimate covariance)
			comEKF.Step(balanceController->dxy, Cov_dxy, balanceController->q, Cov_q, balanceController->dq);
			comEKF.GetCOM(balanceController->COM);
			comEKF.GetCOMCovariance(Cov_COM);
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
					balanceController->omega_ref_setpoint_frame = balanceController->BalanceReference.frame;
					balanceController->omega_ref_setpoint[0] = balanceController->BalanceReference.omega[0];
					balanceController->omega_ref_setpoint[1] = balanceController->BalanceReference.omega[1];
					balanceController->omega_ref_setpoint[2] = balanceController->BalanceReference.omega[2];

					if (balanceController->BalanceReference.angularVelocityOnly == false) {
						balanceController->integrate_omega_ref_into_q_ref = false;
						balanceController->q_ref_setpoint[0] = balanceController->BalanceReference.q[0];
						balanceController->q_ref_setpoint[1] = balanceController->BalanceReference.q[1];
						balanceController->q_ref_setpoint[2] = balanceController->BalanceReference.q[2];
						balanceController->q_ref_setpoint[3] = balanceController->BalanceReference.q[3];
					} else {
						balanceController->integrate_omega_ref_into_q_ref = true;
					}
				}
				else {
					// Reference too old - setting default upright reference
					HeadingQuaternion(balanceController->q_ref_setpoint, balanceController->q_ref_setpoint); // set q_ref_setpoint to the previously set heading and upright
					balanceController->omega_ref_setpoint_frame = lspc::ParameterTypes::INERTIAL_FRAME;
					balanceController->omega_ref_setpoint[0] = 0;
					balanceController->omega_ref_setpoint[1] = 0;
					balanceController->omega_ref_setpoint[2] = 0;
					balanceController->integrate_omega_ref_into_q_ref = false;
				}
				xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back
			}

			if (balanceController->omega_ref_setpoint_frame == lspc::ParameterTypes::BODY_FRAME) {
				balanceController->omega_ref_body[0] = balanceController->omega_ref_setpoint[0];
				balanceController->omega_ref_body[1] = balanceController->omega_ref_setpoint[1];
				balanceController->omega_ref_body[2] = balanceController->omega_ref_setpoint[2];
				Quaternion_RotateVector_Body2Inertial(balanceController->q, balanceController->omega_ref_body, balanceController->omega_ref_inertial);
			}
			else if (balanceController->BalanceReference.frame == lspc::ParameterTypes::INERTIAL_FRAME) {
				balanceController->omega_ref_inertial[0] = balanceController->omega_ref_setpoint[0];
				balanceController->omega_ref_inertial[1] = balanceController->omega_ref_setpoint[1];
				balanceController->omega_ref_inertial[2] = balanceController->omega_ref_setpoint[2];
				Quaternion_RotateVector_Inertial2Body(balanceController->q, balanceController->omega_ref_inertial, balanceController->omega_ref_body);
			}
			else { // unknown frame for quaternion control mode
				balanceController->omega_ref_body[0] = balanceController->omega_ref_setpoint[0];
				balanceController->omega_ref_body[1] = balanceController->omega_ref_setpoint[1];
				balanceController->omega_ref_body[2] = balanceController->omega_ref_setpoint[2];
				balanceController->omega_ref_inertial[0] = 0;
				balanceController->omega_ref_inertial[1] = 0;
				balanceController->omega_ref_inertial[2] = 0;
			}

			if (balanceController->integrate_omega_ref_into_q_ref) {
				// integrate omega_ref to update q_ref_setpoint
				if (balanceController->omega_ref_setpoint_frame == lspc::ParameterTypes::BODY_FRAME)
					Quaternion_Integration_Body(balanceController->q_ref, balanceController->omega_ref_body, dt, balanceController->q_ref_setpoint);
				else if (balanceController->BalanceReference.frame == lspc::ParameterTypes::INERTIAL_FRAME)
					Quaternion_Integration_Inertial(balanceController->q_ref, balanceController->omega_ref_inertial, dt, balanceController->q_ref_setpoint);
			}

			if (params.behavioural.IndependentHeading || (params.estimator.EnableIndependentHeadingAtWheelSlip && wheelSlipDetector.SlipDetected())) {
				HeadingIndependentReferenceManual(balanceController->q_ref_setpoint, balanceController->q, balanceController->q_ref);
			} else {
				balanceController->q_ref[0] = balanceController->q_ref_setpoint[0];
				balanceController->q_ref[1] = balanceController->q_ref_setpoint[1];
				balanceController->q_ref[2] = balanceController->q_ref_setpoint[2];
				balanceController->q_ref[3] = balanceController->q_ref_setpoint[3];
			}
		}
		else {
			// We are not in QUATERNION_CONTROL mode - se reset references related to the QUATERNION_CONTROL mode
			HeadingQuaternion(balanceController->q, balanceController->q_ref_setpoint); // set q_ref_setpoint to the current heading and upright
			balanceController->omega_ref_setpoint_frame = lspc::ParameterTypes::INERTIAL_FRAME;
			balanceController->omega_ref_setpoint[0] = 0;
			balanceController->omega_ref_setpoint[1] = 0;
			balanceController->omega_ref_setpoint[2] = 0;
			balanceController->integrate_omega_ref_into_q_ref = false;
		}

		/* Velocity control enabled */
		if (params.controller.mode == lspc::ParameterTypes::VELOCITY_CONTROL) {
			// Get velocity reference
			if (xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for reading
				if ((microsTimer.GetTime() - balanceController->VelocityReference.time) < params.controller.ReferenceTimeout) {
					balanceController->velocityReferenceFrame = balanceController->VelocityReference.frame;
					balanceController->velocityReference[0] = balanceController->VelocityReference.dx;
					balanceController->velocityReference[1] = balanceController->VelocityReference.dy;
					balanceController->headingVelocityReference = balanceController->VelocityReference.dyaw;
				}
				else {
					// Reference too old - setting default upright reference
					balanceController->velocityReferenceFrame = lspc::ParameterTypes::INERTIAL_FRAME;
					balanceController->velocityReference[0] = 0;
					balanceController->velocityReference[1] = 0;
					balanceController->headingVelocityReference = 0;
				}
				xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back
			}

			balanceController->headingReference += balanceController->headingVelocityReference * dt;
			balanceController->omega_ref_inertial[0] = 0;
			balanceController->omega_ref_inertial[1] = 0;
			balanceController->omega_ref_inertial[2] = balanceController->headingVelocityReference;
			Quaternion_RotateVector_Inertial2Body(balanceController->q, balanceController->omega_ref_inertial, balanceController->omega_ref_body);

			if (params.behavioural.IndependentHeading || (params.estimator.EnableIndependentHeadingAtWheelSlip && wheelSlipDetector.SlipDetected())) {
				balanceController->headingReference = HeadingFromQuaternion(balanceController->q);
			}

			//velocityController.Step(balanceController->q, balanceController->dq, balanceController->dxy, balanceController->velocityReference, (balanceController->velocityReferenceFrame == lspc::ParameterTypes::HEADING_FRAME), balanceController->headingReference, balanceController->q_ref);
			/*velocityController.StepWithOmega(balanceController->q, balanceController->dq, balanceController->dxy, balanceController->velocityReference, (balanceController->velocityReferenceFrame == lspc::ParameterTypes::HEADING_FRAME), balanceController->headingReference, balanceController->q_ref, balanceController->omega_ref_body);
			velocityController.GetIntegral(q_tilt_integral);
			velocityController.GetFilteredVelocityReference_Inertial(balanceController->velocityReference_inertial); // store velocity reference with the filtered one used by the velocity controller  (for logging purposes)
			*/
			velocityLQR.Step(balanceController->xy, balanceController->q, balanceController->dxy, balanceController->dq, balanceController->velocityReference, (balanceController->velocityReferenceFrame == lspc::ParameterTypes::HEADING_FRAME), balanceController->headingReference, balanceController->headingVelocityReference, balanceController->q_ref, balanceController->omega_ref_body);
			velocityLQR.GetFilteredVelocityReference_Inertial(balanceController->velocityReference_inertial);
		}
		else {
			// We are not in VELOCITY_CONTROL mode - se reset references related to the VELOCITY_CONTROL mode
			balanceController->velocityReferenceFrame = lspc::ParameterTypes::INERTIAL_FRAME;
			balanceController->velocityReference[0] = 0;
			balanceController->velocityReference[1] = 0;
			balanceController->headingVelocityReference = 0;
			balanceController->headingReference = HeadingFromQuaternion(balanceController->q);
		}

		/* Wheel slip detector and equivalent control + q_dot ramp (to reduce the robot jumping on the ball during wheel slip) */
		if (params.estimator.EnableWheelSlipDetector) {
			wheelSlipDetector.Step(EncoderAngle);
			if (wheelSlipDetector.SlipDetected()) {
				WheelSlipRampGain = 0;
			}
			else if (WheelSlipRampGain < 1) {
				WheelSlipRampGain += dt / params.estimator.WheelSlipIncreaseTime;
				if (WheelSlipRampGain > 1)
					WheelSlipRampGain = 1;

				if (params.estimator.ReduceTorqueAtWheelSlip)
					TorqueRampUpGain = WheelSlipRampGain;
			}

			if (params.estimator.ReduceEquivalentControlAtWheelSlip)
				EquivalentControlPct = WheelSlipRampGain;
			else
				EquivalentControlPct = 1.0;

			if (params.estimator.ReduceQdotAtWheelSlip) {
				balanceController->dq[0] *= WheelSlipRampGain;
				balanceController->dq[1] *= WheelSlipRampGain;
				balanceController->dq[2] *= WheelSlipRampGain;
				balanceController->dq[3] *= WheelSlipRampGain;
			}
		}

		/* Compute control output based on references */
		if (params.controller.type == lspc::ParameterTypes::LQR_CONTROLLER && params.controller.mode != lspc::ParameterTypes::OFF) {
			lqr.Step(balanceController->q, balanceController->dq, balanceController->xy, balanceController->dxy, balanceController->COM, balanceController->q_ref, balanceController->omega_ref_body, Torque);
		} else if (params.controller.type == lspc::ParameterTypes::SLIDING_MODE_CONTROLLER && params.controller.mode != lspc::ParameterTypes::OFF) {
			// OBS. When running the Sliding Mode controller, inertial angular velocity reference is needed
			sm.Step(balanceController->q, balanceController->dq, balanceController->xy, balanceController->dxy, balanceController->COM, balanceController->q_ref, balanceController->omega_ref_body, EquivalentControlPct, Torque, S);
		} else if(params.controller.type == lspc::ParameterTypes::FEEDBACK_LINEARIZATION_CONTROLLER && params.controller.mode != lspc::ParameterTypes::OFF) {
			fbl.Step(balanceController->q, balanceController->dq, balanceController->xy, balanceController->dxy, balanceController->COM, balanceController->q_ref, balanceController->omega_ref_body, Torque);
		}else {
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
		float saturationTorque = params.model.SaturationTorqueOfMaxOutputTorque * params.model.MaxOutputTorque;
		Torque[0] = fmaxf(fminf(Torque[0], saturationTorque), -saturationTorque);
		Torque[1] = fmaxf(fminf(Torque[1], saturationTorque), -saturationTorque);
		Torque[2] = fmaxf(fminf(Torque[2], saturationTorque), -saturationTorque);

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

			/* Measure delivered torque feedback from ESCON drivers */
			TorqueDelivered[0] = motor1.GetAppliedOutputTorque();
			TorqueDelivered[1] = motor2.GetAppliedOutputTorque();
			TorqueDelivered[2] = motor3.GetAppliedOutputTorque();

			/* Detect motor driver failure mode - and if so, reset motor driver */
			if (params.controller.MotorFailureDetection && !params.debug.DisableMotorOutput) { // only run motor failure detection is output is actually enabled
				for (int i = 0; i < 3; i++) {
					//if (fabsf(Torque[i]) > 0.1 && fabsf((Torque[i] / TorqueDelivered[i]) - 1.0) > params.controller.MotorFailureThreshold) {
					if (fabsf(Torque[i]) > params.controller.MotorFailureThreshold && fabsf(Torque[i] - TorqueDelivered[i]) > params.controller.MotorFailureThreshold) {
						MotorDriverFailureCounts[i]++;
					} else {
						MotorDriverFailureCounts[i] = 0;
					}
					if (MotorDriverFailureCounts[i] > (params.controller.SampleRate * params.controller.MotorFailureDetectionTime)) {
						// Motor/motor driver failure detected
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
		}
		if (params.controller.mode == lspc::ParameterTypes::OFF) {
			/* Delivered torque can not be measured when the motors are disabled */
			TorqueDelivered[0] = 0;
			TorqueDelivered[1] = 0;
			TorqueDelivered[2] = 0;

			/* Reset failure counts */
			MotorDriverFailureCounts[0] = 0;
			MotorDriverFailureCounts[1] = 0;
			MotorDriverFailureCounts[2] = 0;

			/* Reset wheel slip detector */
			wheelSlipDetector.Reset();
			WheelSlipRampGain = 1.0;
			EquivalentControlPct = 1.0;

			/* Reset reference variables */
			HeadingQuaternion(balanceController->q, balanceController->q_ref); // reset attitude reference to upright with current heading
			balanceController->q_ref_setpoint[0] = balanceController->q_ref[0];
			balanceController->q_ref_setpoint[1] = balanceController->q_ref[1];
			balanceController->q_ref_setpoint[2] = balanceController->q_ref[2];
			balanceController->q_ref_setpoint[3] = balanceController->q_ref[3];
			balanceController->omega_ref_inertial[0] = 0; // zero angular velocity reference
			balanceController->omega_ref_inertial[1] = 0;
			balanceController->omega_ref_inertial[2] = 0;
			balanceController->omega_ref_body[0] = 0; // zero angular velocity reference
			balanceController->omega_ref_body[1] = 0;
			balanceController->omega_ref_body[2] = 0;
			balanceController->omega_ref_setpoint_frame = lspc::ParameterTypes::UNKNOWN_FRAME;
			balanceController->omega_ref_setpoint[0] = 0;
			balanceController->omega_ref_setpoint[1] = 0;
			balanceController->omega_ref_setpoint[2] = 0;
			balanceController->integrate_omega_ref_into_q_ref = false;
			balanceController->headingReference = HeadingFromQuaternion(balanceController->q);
			balanceController->headingVelocityReference = 0;
			balanceController->velocityReferenceFrame = lspc::ParameterTypes::UNKNOWN_FRAME;
			balanceController->velocityReference[0] = 0;
			balanceController->velocityReference[1] = 0;
			balanceController->velocityReference_inertial[0] = 0;
			balanceController->velocityReference_inertial[1] = 0;
			balanceController->ReferenceGenerationStep = 0; // only used if test reference generation is enabled

			/* Reset controllers with internal states */
			velocityController.Reset();
			velocityLQR.Reset();
		}

		if (params.controller.mode != lspc::ParameterTypes::OFF && !params.debug.DisableMotorOutput) {
			/* Ensure that motor drivers are enabled */
			motor1.Enable();
			motor2.Enable();
			motor3.Enable();
		} else {
			/* Controller is disabled or DisableMotorOutput is enabled, so disable motor drivers */
			motor1.Disable();
			motor2.Disable();
			motor3.Disable();
		}

		/* Measure compute time */
		dt_compute = microsTimer.GetDeltaTime(prevTimerValue);
		dt_compute2 = HAL_toc(timerPrev);

		/* Send controller info package */
		balanceController->SendControllerInfo(params.controller.type, params.controller.mode, Torque, dt_compute, TorqueDelivered);
		balanceController->SendControllerDebug(q_tilt_integral, dxy_kinematics, Torque, S);
		//Debug::printf("Balance controller compute time: %9.7f s\n", dt_compute);
		//Debug::printf("Applied torque: %4.2f\t%4.2f\t%4.2f\n", TorqueApplied[0], TorqueApplied[1], TorqueApplied[2]);

		/* Get values from Xsens IMU (if available) for logging */
		if (balanceController->mti) {
			MTImeas = balanceController->mti->GetLastMeasurement();
			balanceController->mti->GetEstimates(MTIest);
		}

		if (params.debug.EnableDumpMessages) {
			/* Send mixed data for logging through MathDump channel */
			float mathDumpArray[] = {timestamp,
									 imuFiltered.Accelerometer[0],
									 imuFiltered.Accelerometer[1],
									 imuFiltered.Accelerometer[2],
									 imuFiltered.Gyroscope[0],
									 imuFiltered.Gyroscope[1],
									 imuFiltered.Gyroscope[2],
									 imuFiltered.Magnetometer[0],
									 imuFiltered.Magnetometer[1],
									 imuFiltered.Magnetometer[2],
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
									 1.0f*wheelSlipDetector.SlipDetected(0) + 10.0f*wheelSlipDetector.SlipDetected(1) + 100.0f*wheelSlipDetector.SlipDetected(2),
									 balanceController->q_ref[0],
									 balanceController->q_ref[1],
									 balanceController->q_ref[2],
									 balanceController->q_ref[3],
									 balanceController->omega_ref_body[0],
									 balanceController->omega_ref_body[1],
									 balanceController->omega_ref_body[2],
									 balanceController->velocityReference_inertial[0],
									 balanceController->velocityReference_inertial[1],
									 q_tilt_integral[0],
									 q_tilt_integral[1],
									 q_tilt_integral[2],
									 q_tilt_integral[3],
									 WheelSlipRampGain,
									 S[0],
									 S[1],
									 S[2],
									 dt_compute,
									 Torque[0],
									 Torque[1],
									 Torque[2]
								};
			com.TransmitAsync(lspc::MessageTypesToPC::MathDump, (uint8_t *)&mathDumpArray, sizeof(mathDumpArray));

			/* Sensor dump */
			float sensorDumpArray[] = { timestamp,
										imuRaw.Accelerometer[0],
										imuRaw.Accelerometer[1],
										imuRaw.Accelerometer[2],
										imuRaw.Gyroscope[0],
										imuRaw.Gyroscope[1],
										imuRaw.Gyroscope[2],
										imuRaw.Magnetometer[0],
										imuRaw.Magnetometer[1],
										imuRaw.Magnetometer[2],
										EncoderAngle[0],
										EncoderAngle[1],
										EncoderAngle[2],
										TorqueDelivered[0],
										TorqueDelivered[1],
										TorqueDelivered[2],
										MTIest.q[0],
										MTIest.q[1],
										MTIest.q[2],
										MTIest.q[3],
										MTIest.dq[0],
										MTIest.dq[1],
										MTIest.dq[2],
										MTIest.dq[3],
										MTImeas.Accelerometer[0],
										MTImeas.Accelerometer[1],
										MTImeas.Accelerometer[2],
										MTImeas.Gyroscope[0],
										MTImeas.Gyroscope[1],
										MTImeas.Gyroscope[2],
										MTImeas.Magnetometer[0],
										MTImeas.Magnetometer[1],
										MTImeas.Magnetometer[2]
									};
			com.TransmitAsync(lspc::MessageTypesToPC::SensorDump, (uint8_t *)&sensorDumpArray, sizeof(sensorDumpArray));

			/* Covariance dump */
			float covarianceDumpArray[1 +
									  sizeof(Cov_q)/sizeof(float) +
									  sizeof(Cov_dq)/sizeof(float) +
									  sizeof(Cov_bias)/sizeof(float) +
									  sizeof(Cov_dxy)/sizeof(float) +
									  sizeof(Cov_COM)/sizeof(float)];
			covarianceDumpArray[0] = timestamp;
			uint8_t * writePtr = (uint8_t *)&covarianceDumpArray[1];
			memcpy(writePtr, Cov_q, sizeof(Cov_q)); writePtr += sizeof(Cov_q);
			memcpy(writePtr, Cov_dq, sizeof(Cov_dq)); writePtr += sizeof(Cov_dq);
			memcpy(writePtr, Cov_bias, sizeof(Cov_bias)); writePtr += sizeof(Cov_bias);
			memcpy(writePtr, Cov_dxy, sizeof(Cov_dxy)); writePtr += sizeof(Cov_dxy);
			memcpy(writePtr, Cov_COM, sizeof(Cov_COM)); writePtr += sizeof(Cov_COM);
			com.TransmitAsync(lspc::MessageTypesToPC::CovarianceDump, (uint8_t *)&covarianceDumpArray, sizeof(covarianceDumpArray));
		}
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
	delete(&fbl);
	delete(&velocityController);
	delete(&velocityLQR);
	delete(&qEKF);
	delete(&madgwick);
	delete(&velocityEKF);
	delete(&comEKF);
	delete(&kinematics);
	delete(&wheelSlipDetector);
	delete(&Motor1_LPF);
	delete(&Motor2_LPF);
	delete(&Motor3_LPF);

	/* Stop and delete task */
	balanceController->isRunning_ = false;
	balanceController->TaskHandle_ = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

/* Initialize/stabilize estimators for certain stabilization time */
void BalanceController::StabilizeFilters(Parameters& params, IMU& imu, QEKF& qEKF, Madgwick& madgwick, VelocityEKF& velocityEKF, TickType_t loopWaitTicks, float stabilizationTime)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t finishTick = xLastWakeTime + configTICK_RATE_HZ * stabilizationTime;

	IMU::Measurement_t imuMeas;
	int32_t EncoderTicks[3];
	float Cov_q[4*4]; // set diagonal to something small (if Madgwick is used)
	Cov_q[4*0 + 0] = 3*1E-7;
	Cov_q[4*1 + 1] = 3*1E-7;
	Cov_q[4*2 + 2] = 3*1E-7;
	Cov_q[4*3 + 3] = 3*1E-7;

	while (xLastWakeTime < finishTick) {
		// Wait until time has been reached to make control loop periodic
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);

		/* Get measurements (sample) */
		imu.Get(imuMeas);
		/* Adjust the measurements according to the calibration */
		imu.CorrectMeasurement(imuMeas, !params.estimator.UseXsensIMU, !params.estimator.UseXsensIMU, !params.estimator.UseXsensIMU); // do not correct gyroscope bias if Xsens IMU is used (since this is corrected internally by the Xsens Kalman filter)

		/* Compute quaternion estimate */
		if (params.estimator.UseMadgwick) {
			madgwick.updateIMU(imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2], imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2], 0.1); // use larger beta to make filter converge to current angle by trusting the accelerometer more
		} else {
			qEKF.Step(imuMeas.Accelerometer, imuMeas.Gyroscope, false); // do not estimate bias while stabilizing the filter
			qEKF.GetQuaternionCovariance(Cov_q);
		}

		/* Compute velocity estimate */
		if (params.estimator.UseVelocityEstimator) {
			EncoderTicks[0] = motor1.GetEncoderRaw();
			EncoderTicks[1] = motor2.GetEncoderRaw();
			EncoderTicks[2] = motor3.GetEncoderRaw();
			velocityEKF.Step(EncoderTicks, imuMeas.Accelerometer, q, Cov_q, dq); // velocity estimator can estimate either CoR velocity or ball velocity
		}
	}
}

/* Reference generation based on selected test */
void BalanceController::ReferenceGeneration(Parameters& params)
{
	if (params.behavioural.StepTestEnabled) {
		float quaternion_reference[4];
		ReferenceGenerationStep++;
		if (ReferenceGenerationStep >= 20*params.controller.SampleRate) // reset after 20 seconds
			ReferenceGenerationStep = 0;

		if (ReferenceGenerationStep < 2*params.controller.SampleRate) { // from 0-2 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(0), quaternion_reference); // rpy = 0,0,0
		}
		else if (ReferenceGenerationStep < 4*params.controller.SampleRate) { // from 2-4 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(5), quaternion_reference); // rpy = 3,0,0
		}
		else if (ReferenceGenerationStep < 6*params.controller.SampleRate) { // from 4-6 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(0), quaternion_reference); // rpy = 0,0,0
		}
		else if (ReferenceGenerationStep < 8*params.controller.SampleRate) { // from 6-8 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(5), deg2rad(0), quaternion_reference); // rpy = 0,3,0
		}
		else if (ReferenceGenerationStep < 10*params.controller.SampleRate) { // from 8-10 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(0), quaternion_reference); // rpy = 0,0,0
		}
		else if (ReferenceGenerationStep < 12*params.controller.SampleRate) { // from 10-12 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(5), deg2rad(-5), quaternion_reference); // rpy = -3,3,0
		}
		else if (ReferenceGenerationStep < 14*params.controller.SampleRate) { // from 12-14 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(0), quaternion_reference); // rpy = 0,0,0
		}
		else if (ReferenceGenerationStep < 18*params.controller.SampleRate) { // from 14-18 seconds
			Quaternion_eul2quat_zyx(deg2rad(45), deg2rad(0), deg2rad(0), quaternion_reference); // rpy = 0,0,45
		}
		else if (ReferenceGenerationStep < 20*params.controller.SampleRate) { // from 18-20 seconds
			Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(0), quaternion_reference); // rpy = 0,0,0
		}

		if (xSemaphoreTake( BalanceReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for updating
			BalanceReference.time = microsTimer.GetTime();
			BalanceReference.omega[0] = 0;
			BalanceReference.omega[1] = 0;
			BalanceReference.omega[2] = 0;
			BalanceReference.q[0] = quaternion_reference[0];
			BalanceReference.q[1] = quaternion_reference[1];
			BalanceReference.q[2] = quaternion_reference[2];
			BalanceReference.q[3] = quaternion_reference[3];
			BalanceReference.frame = lspc::ParameterTypes::BODY_FRAME;
			BalanceReference.angularVelocityOnly = false;
			xSemaphoreGive( BalanceReference.semaphore ); // give semaphore back
		}
	}
	else if (params.behavioural.SineTestEnabled) { // sine test with increasing frequency
			float quaternion_reference[4];
			float Amplitude = deg2rad(2);
			const float BaseFrequency = 0.5; // hz
			const float freqRate = 0.02; // hz pr. second
			float t = (float)ReferenceGenerationStep / params.controller.SampleRate;
			float freq = (BaseFrequency + freqRate*t);
			float g = 2*M_PI*freq*t;
			float f = Amplitude * sin(g);
			float dgdt = 2*M_PI* (freqRate*t + freq);
			float dfdt = Amplitude * cos(g) * dgdt;
			// Roll only
			Quaternion_eul2quat_zyx(0, 0, f, quaternion_reference);
			// Combined roll (sine wave) and pitch (cosine wave)
			//Quaternion_eul2quat_zyx(0, deg2rad(1) * cosf(2*M_PI *  (float)ReferenceGenerationStep * SineFrequency / params.controller.SampleRate), deg2rad(3) * sinf(2*M_PI *  (float)ReferenceGenerationStep * SineFrequency / params.controller.SampleRate), quaternion_reference);
			ReferenceGenerationStep++;


			if (xSemaphoreTake( BalanceReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for updating
				BalanceReference.time = microsTimer.GetTime();
				BalanceReference.omega[0] = dfdt; // deg2rad(3) * 2*M_PI*SineFrequency * cosf(2*M_PI *  (float)ReferenceGenerationStep * SineFrequency / params.controller.SampleRate);
				BalanceReference.omega[1] = 0; //deg2rad(0) * 2*M_PI*SineFrequency * -sinf(2*M_PI *  (float)ReferenceGenerationStep * SineFrequency / params.controller.SampleRate);
				BalanceReference.omega[2] = 0;
				BalanceReference.q[0] = quaternion_reference[0];
				BalanceReference.q[1] = quaternion_reference[1];
				BalanceReference.q[2] = quaternion_reference[2];
				BalanceReference.q[3] = quaternion_reference[3];
				BalanceReference.frame = lspc::ParameterTypes::BODY_FRAME;
				BalanceReference.angularVelocityOnly = false;
				xSemaphoreGive( BalanceReference.semaphore ); // give semaphore back
			}
	}
	else if (params.behavioural.CircleTestEnabled) { // constant inclination rotating in a circle around inertial z-axis with an increasing angular velocity
			float quaternion_reference_tmp[4];
			float Amplitude = deg2rad(3);
			float rotationIncreaseRate = 2*M_PI * 0.02; // rad/s^2
			float t = (float)ReferenceGenerationStep / params.controller.SampleRate;
			float psi = 0.5 * rotationIncreaseRate * t * t;
			float psi_dot = rotationIncreaseRate * t;
			quaternion_reference_tmp[0]	= cos(Amplitude/2);
			quaternion_reference_tmp[1]	= sin(Amplitude/2);
			quaternion_reference_tmp[2]	= 0;
			quaternion_reference_tmp[3]	= 0;

			float quaternion_reference[4];
			quaternion_reference[0] = quaternion_reference_tmp[0];
			quaternion_reference[1] = cosf(psi)*quaternion_reference_tmp[1] - sinf(psi)*quaternion_reference_tmp[2];
			quaternion_reference[2] = sinf(psi)*quaternion_reference_tmp[1] + cosf(psi)*quaternion_reference_tmp[2];
			quaternion_reference[3] = 0;

			float qdot_ref[4];
			qdot_ref[0] = 0;
			qdot_ref[1] = psi_dot * (-sinf(psi)*quaternion_reference_tmp[1] - cosf(psi)*quaternion_reference_tmp[2]);
			qdot_ref[2] = psi_dot * (cosf(psi)*quaternion_reference_tmp[1] - sinf(psi)*quaternion_reference_tmp[2]);
			qdot_ref[3] = 0;

			float omega_ref_body[3];
			Quaternion_GetAngularVelocity_Body(quaternion_reference, qdot_ref, omega_ref_body);

			ReferenceGenerationStep++;

			if (xSemaphoreTake( BalanceReference.semaphore, ( TickType_t ) 1) == pdTRUE) { // lock for updating
				BalanceReference.time = microsTimer.GetTime();
				BalanceReference.omega[0] = omega_ref_body[0];
				BalanceReference.omega[1] = omega_ref_body[1];
				BalanceReference.omega[2] = omega_ref_body[2];
				BalanceReference.q[0] = quaternion_reference[0];
				BalanceReference.q[1] = quaternion_reference[1];
				BalanceReference.q[2] = quaternion_reference[2];
				BalanceReference.q[3] = quaternion_reference[3];
				BalanceReference.frame = lspc::ParameterTypes::BODY_FRAME;
				BalanceReference.angularVelocityOnly = false;
				xSemaphoreGive( BalanceReference.semaphore ); // give semaphore back
			}
	}
	else {
		ReferenceGenerationStep = 0; // reset reference generation step such that we are ready when enabled
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

	if (params.estimator.UseXsensIMU) {
		memcpy(imu_msg.accelerometer.cov, params.estimator.cov_acc_mti, sizeof(params.estimator.cov_acc_mti));
		memcpy(imu_msg.gyroscope.cov, params.estimator.cov_gyro_mti, sizeof(params.estimator.cov_gyro_mti));
	} else {
		memcpy(imu_msg.accelerometer.cov, params.estimator.cov_acc_mpu, sizeof(params.estimator.cov_acc_mpu));
		memcpy(imu_msg.gyroscope.cov, params.estimator.cov_gyro_mpu, sizeof(params.estimator.cov_gyro_mpu));
	}
	imu_msg.gyroscope.x = imuMeas.Gyroscope[0];
	imu_msg.gyroscope.y = imuMeas.Gyroscope[1];
	imu_msg.gyroscope.z = imuMeas.Gyroscope[2];

	imu_msg.magnetometer.x = imuMeas.Magnetometer[0];
	imu_msg.magnetometer.y = imuMeas.Magnetometer[1];
	imu_msg.magnetometer.z = imuMeas.Magnetometer[2];

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

void BalanceController::SendControllerDebug(const float q_integral[4], const float velocity_kinematics[2], const float Torque[3], const float S[3])
{
	lspc::MessageTypesToPC::ControllerDebug_t msg;
	float YPR[3];

	msg.time = microsTimer.GetTime();

	Quaternion_quat2eul_zyx(q, YPR);
	msg.orient.yaw = rad2deg(YPR[0]);
	msg.orient.pitch = rad2deg(YPR[1]);
	msg.orient.roll = rad2deg(YPR[2]);

	Quaternion_quat2eul_zyx(q_ref, YPR);
	msg.orient_ref.yaw = rad2deg(YPR[0]);
	msg.orient_ref.pitch = rad2deg(YPR[1]);
	msg.orient_ref.roll = rad2deg(YPR[2]);

	Quaternion_quat2eul_zyx(q_integral, YPR);
	msg.orient_integral.yaw = rad2deg(YPR[0]);
	msg.orient_integral.pitch = rad2deg(YPR[1]);
	msg.orient_integral.roll = rad2deg(YPR[2]);

	msg.omega.x = omega_body[0];
	msg.omega.y = omega_body[1];
	msg.omega.z = omega_body[2];
	msg.omega_ref.x = omega_ref_body[0];
	msg.omega_ref.y = omega_ref_body[1];
	msg.omega_ref.z = omega_ref_body[2];
	msg.vel.x = dxy[0];
	msg.vel.y = dxy[1];
	msg.vel_kinematics.x = velocity_kinematics[0];
	msg.vel_kinematics.y = velocity_kinematics[1];
	msg.vel_ref.x = velocityReference[0];
	msg.vel_ref.y = velocityReference[1];
	msg.torque[0] = Torque[0];
	msg.torque[1] = Torque[1];
	msg.torque[2] = Torque[2];
	msg.S[0] = S[0];
	msg.S[1] = S[1];
	msg.S[2] = S[2];

	com.TransmitAsync(lspc::MessageTypesToPC::ControllerDebug, (uint8_t *)&msg, sizeof(msg));
}


void BalanceController::CalibrateIMU(bool calibrateAccelerometer)
{
	bool restartAfterCalibration = false;
	if (isRunning_) {
		Stop();
		restartAfterCalibration = true;
	}

	if (calibrateAccelerometer)
		imu.CalibrateAccelerometer(true);
	else
		imu.Calibrate(true); // calibrate only gyro bias and alignment


	if (restartAfterCalibration) {
		/*Debug::print("Restarting controller in 5 seconds...\n");
		osDelay(5000);*/
		Start();
	}
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

	/* Send acknowledge message back to PC */
	msgAck.acknowledged = true;
	balanceController->com.TransmitAsync(lspc::MessageTypesToPC::CalibrateIMUAck, (uint8_t *)&msgAck, sizeof(msgAck));

	balanceController->CalibrateIMU(msg.calibrate_accelerometer);
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

void BalanceController::QuaternionReference_Callback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;
	if (!balanceController->BalanceReference.semaphore) return;

	volatile lspc::MessageTypesFromPC::QuaternionReference_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	xSemaphoreTake( balanceController->BalanceReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	/* Update references with input values from message */
	balanceController->BalanceReference.time = balanceController->microsTimer.GetTime();
	balanceController->BalanceReference.frame = lspc::ParameterTypes::BODY_FRAME;
	balanceController->BalanceReference.q[0] = msg.q.w;
	balanceController->BalanceReference.q[1] = msg.q.x;
	balanceController->BalanceReference.q[2] = msg.q.y;
	balanceController->BalanceReference.q[3] = msg.q.z;
	balanceController->BalanceReference.omega[0] = 0; // only quaternion reference is specified in this package
	balanceController->BalanceReference.omega[1] = 0;
	balanceController->BalanceReference.omega[2] = 0;
	balanceController->BalanceReference.angularVelocityOnly = false;

	xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back
}

void BalanceController::AngularVelocityReference_Callback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;
	if (!balanceController->BalanceReference.semaphore) return;

	volatile lspc::MessageTypesFromPC::AngularVelocityReference_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	xSemaphoreTake( balanceController->BalanceReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	/* Update references with input values from message */
	balanceController->BalanceReference.time = balanceController->microsTimer.GetTime();
	balanceController->BalanceReference.frame = msg.frame;
	balanceController->BalanceReference.q[0] = 1; // only angular velocity reference is specified in this package
	balanceController->BalanceReference.q[1] = 0;
	balanceController->BalanceReference.q[2] = 0;
	balanceController->BalanceReference.q[3] = 0;
	balanceController->BalanceReference.omega[0] = msg.omega.x;
	balanceController->BalanceReference.omega[1] = msg.omega.y;
	balanceController->BalanceReference.omega[2] = msg.omega.z;
	balanceController->BalanceReference.angularVelocityOnly = true;

	xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back
}

void BalanceController::BalanceControllerReference_Callback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;
	if (!balanceController->BalanceReference.semaphore) return;

	volatile lspc::MessageTypesFromPC::BalanceControllerReference_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	xSemaphoreTake( balanceController->BalanceReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	/* Update references with input values from message */
	balanceController->BalanceReference.time = balanceController->microsTimer.GetTime();
	balanceController->BalanceReference.frame = msg.frame;
	balanceController->BalanceReference.q[0] = msg.q.w;
	balanceController->BalanceReference.q[1] = msg.q.x;
	balanceController->BalanceReference.q[2] = msg.q.y;
	balanceController->BalanceReference.q[3] = msg.q.z;
	balanceController->BalanceReference.omega[0] = msg.omega.x;
	balanceController->BalanceReference.omega[1] = msg.omega.y;
	balanceController->BalanceReference.omega[2] = msg.omega.z;
	balanceController->BalanceReference.angularVelocityOnly = false;

	xSemaphoreGive( balanceController->BalanceReference.semaphore ); // give semaphore back
}

void BalanceController::VelocityReference_Callback(void * param, const std::vector<uint8_t>& payload)
{
	BalanceController * balanceController = (BalanceController *)param;
	if (!balanceController) return;
	if (!balanceController->VelocityReference.semaphore) return;

	volatile lspc::MessageTypesFromPC::VelocityReference_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating

	/* Update references with input values from message */
	balanceController->VelocityReference.time = balanceController->microsTimer.GetTime();
	balanceController->VelocityReference.frame = msg.frame;
	balanceController->VelocityReference.dx = msg.vel.x;
	balanceController->VelocityReference.dy = msg.vel.y;
	balanceController->VelocityReference.dyaw = msg.vel.yaw;

	xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back
}
