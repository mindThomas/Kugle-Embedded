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
	VelocityReference.semaphore = xSemaphoreCreateBinary();
	if (VelocityReference.semaphore == NULL) {
		ERROR("Could not create Velocity reference semaphore");
		return;
	}
	vQueueAddToRegistry(VelocityReference.semaphore, "Velocity reference");
	xSemaphoreGive( VelocityReference.semaphore ); // give the semaphore the first time

	/* Register message type callbacks */
	com.registerCallback(lspc::MessageTypesFromPC::CalibrateIMU, CalibrateIMUCallback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::VelocityReference_Heading, VelocityReference_Heading_Callback, (void *)this);
	com.registerCallback(lspc::MessageTypesFromPC::VelocityReference_Inertial, VelocityReference_Inertial_Callback, (void *)this);

	Start();
}

BalanceController::~BalanceController()
{
	shouldStop_ = true;
	while (isRunning_) osDelay(10);

	/* Unregister message callbacks */
	com.unregisterCallback(lspc::MessageTypesFromPC::CalibrateIMU);
	com.unregisterCallback(lspc::MessageTypesFromPC::VelocityReference_Heading);
	com.unregisterCallback(lspc::MessageTypesFromPC::VelocityReference_Inertial);

	/* Delete semaphores */
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
	QuaternionVelocityControl& velocityController = *(new QuaternionVelocityControl(params, &balanceController->microsTimer, 1.0f / params.controller.SampleRate));
	QEKF& qEKF = *(new QEKF(params, &balanceController->microsTimer));
	Madgwick& madgwick = *(new Madgwick(params.controller.SampleRate, params.estimator.MadgwickBeta));
	VelocityEKF& velocityEKF = *(new VelocityEKF(params, &balanceController->microsTimer));
	COMEKF& comEKF = *(new COMEKF(params, &balanceController->microsTimer));
	Kinematics& kinematics = *(new Kinematics(params));
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> accel_x_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> accel_y_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> accel_z_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> gyro_x_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> gyro_y_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);
	IIR<sizeof(params.estimator.SoftwareLPFcoeffs_a)/sizeof(float)-1> gyro_z_filt(params.estimator.SoftwareLPFcoeffs_a, params.estimator.SoftwareLPFcoeffs_b);

	/* Measurement variables */
	IMU::Measurement_t imuMeas;
	int32_t EncoderTicks[3];
	float EncoderAngle[3];

	/* Estimate covariance variables */
	float Cov_q[4*4];
	float Cov_dxy[2*2];

	/* Control output variables */
	float Torque[3];
	float TorqueRampUpGain = 0;
	bool TorqueRampUpFinished = false;

	/* Control output filtering objects */
	FirstOrderLPF& Motor1_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));
	FirstOrderLPF& Motor2_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));
	FirstOrderLPF& Motor3_LPF = *(new FirstOrderLPF(1.0f/params.controller.SampleRate, params.controller.TorqueLPFtau));

	if (!lqr.UnitTest()) {
		ERROR("LQR Unit test failed!");
	}

	if (!sm.UnitTest()) {
		ERROR("Sliding Mode Unit test failed!");
	}

	if (!qEKF.UnitTest()) {
		ERROR("qEKF Unit test failed!");
	}

	/* Reset estimators */
	imu.Get(imuMeas);
	EncoderTicks[0] = motor1.GetEncoderRaw();
	EncoderTicks[1] = motor2.GetEncoderRaw();
	EncoderTicks[2] = motor3.GetEncoderRaw();
	EncoderAngle[0] = motor1.GetAngle();
	EncoderAngle[1] = motor2.GetAngle();
	EncoderAngle[2] = motor3.GetAngle();
	qEKF.Reset(imuMeas.Accelerometer); // reset attitude estimator to current attitude, based on IMU
	madgwick.Reset(imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2]);
	velocityEKF.Reset(EncoderTicks);
	comEKF.Reset();
	kinematics.Reset(EncoderAngle);

	balanceController->StabilizeFilters(params, imu, qEKF, madgwick, loopWaitTicks, 1.0f); // stabilize estimators for 1 second

	/* Reset COM estimate */
	balanceController->COM[0] = 0;
	balanceController->COM[1] = 0;
	balanceController->COM[2] = params.model.l; // initialize COM directly above center of ball at height L

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
	balanceController->PropagateQuaternionReference = false;
	balanceController->headingReference = 0; // consider to replace this with current heading (based on estimate of stabilized QEKF filter)
	balanceController->ReferenceGenerationStep = 0; // only used if test reference generation is enabled
	balanceController->prevTimerValue = microsTimer.Get();

	/* Reset reference inputs */
	xSemaphoreTake( balanceController->VelocityReference.semaphore, ( TickType_t ) portMAX_DELAY); // lock for updating
	balanceController->VelocityReference.dx = 0;
	balanceController->VelocityReference.dy = 0;
	balanceController->VelocityReference.dyaw = 0;
	xSemaphoreGive( balanceController->VelocityReference.semaphore ); // give semaphore back

	/* Enable motor outputs with 0 torque */
	motor1.SetTorque(0);
	motor2.SetTorque(0);
	motor3.SetTorque(0);
	motor1.Enable();
	motor2.Enable();
	motor3.Enable();

	float volatile dt_meas, dt_meas2;

/*#pragma GCC push_options
#pragma GCC optimize("O0")
// Code here
#pragma GCC pop_options*/
/* For functions the following post attribute to the function declaration (in C/C++ file) can be made to disable optimization
__attribute__((optimize("O0")))
*/

	/* Main control loop */
	xLastWakeTime = xTaskGetTickCount();
	while (!balanceController->shouldStop_) {
		/* Wait until time has been reached to make control loop periodic */
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);
		params.Refresh(); // load current parameters from global parameter object

	    uint32_t prevTimer = microsTimer.Get();
		uint32_t timerPrev = HAL_tic();

		/* Get measurements (sample) */
		imu.Get(imuMeas);
	    /* Adjust the measurements according to the calibration */
		imu.CorrectMeasurement(imuMeas);

		if (params.estimator.EnableSoftwareLPFfilters) {
			imuMeas.Accelerometer[0] = accel_x_filt.Filter(imuMeas.Accelerometer[0]);
			imuMeas.Accelerometer[1] = accel_y_filt.Filter(imuMeas.Accelerometer[1]);
			imuMeas.Accelerometer[2] = accel_z_filt.Filter(imuMeas.Accelerometer[2]);
			imuMeas.Gyroscope[0] = gyro_x_filt.Filter(imuMeas.Gyroscope[0]);
			imuMeas.Gyroscope[1] = gyro_y_filt.Filter(imuMeas.Gyroscope[1]);
			imuMeas.Gyroscope[2] = gyro_z_filt.Filter(imuMeas.Gyroscope[2]);
		}

		EncoderTicks[0] = motor1.GetEncoderRaw();
		EncoderTicks[1] = motor2.GetEncoderRaw();
		EncoderTicks[2] = motor3.GetEncoderRaw();
		EncoderAngle[0] = motor1.GetAngle();
		EncoderAngle[1] = motor2.GetAngle();
		EncoderAngle[2] = motor3.GetAngle();

		if (params.debug.EnableRawSensorOutput) {
			balanceController->SendRawSensors(imuMeas, EncoderAngle);
		}

		/* Attitude estimation */
		if (params.estimator.UseMadgwick) {
			madgwick.updateIMU(imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2], imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2]);
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
			qEKF.Step(imuMeas.Accelerometer, imuMeas.Gyroscope, params.estimator.EstimateBias);
			qEKF.GetQuaternion(balanceController->q);
			qEKF.GetQuaternionDerivative(balanceController->dq);
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

	    /* Velocity estimation using kinematics */
	    if (!params.estimator.UseVelocityEstimator) {
	    	// compute velocity from encoder-based motor velocities and forward kinematics
	    	kinematics.EstimateMotorVelocity(EncoderAngle);
			if (params.estimator.Use2Lvelocity) {
				float dxy_ball[2];
				kinematics.ForwardKinematics(balanceController->q, balanceController->dq, dxy_ball);
				kinematics.ConvertBallTo2Lvelocity(dxy_ball, balanceController->q, balanceController->dq, balanceController->dxy);            // put 2L velocity into dxy
			} else {
				kinematics.ForwardKinematics(balanceController->q, balanceController->dq, balanceController->dxy);       // put ball velocity into dxy
			}
	    }

	    /* Velocity estimation using velocity EKF */
	    if (params.estimator.UseVelocityEstimator) {
	    	velocityEKF.Step(EncoderTicks, balanceController->q, Cov_q, balanceController->dq, balanceController->COM); // velocity estimator estimates 2L velocity
	    	velocityEKF.GetVelocity(balanceController->dxy);
	    	velocityEKF.GetVelocityCovariance(Cov_dxy);

	    	// OBS. dxy was in the original design supposed to be ball velocity, but the velocity estimator estimates the 2L velocity which gives indirect "stabilization"
	    	// In the Sliding Mode controller this velocity is (only) used to calculated "feedforward" torque to counteract friction
	        if (!params.estimator.Use2Lvelocity) { // however if the 2L velocity is not desired, it is here converted back to ball velocity
	        	kinematics.Convert2LtoBallVelocity(balanceController->dxy, balanceController->q, balanceController->dq, balanceController->dxy);
	        }
	    }

	    /* Center of Mass estimation */
	    if (params.estimator.EstimateCOM && params.estimator.UseVelocityEstimator) { // can only estimate COM if velocity is also estimated (due to need of velocity estimate covariance)
	    	comEKF.Step(balanceController->dxy, Cov_dxy, balanceController->q, Cov_q, balanceController->dq);
	    	comEKF.GetCOM(balanceController->COM);
	    }

	    /* Disable dq to avoid noisy control outputs resulting from noisy dq estimates */
	    if (params.controller.DisableQdot) { // q_dot removed because it is VERY noisy - this causes oscillations on yaw, if yaw reference is included
	    	balanceController->dq[0] = 0.0f;
	    	balanceController->dq[1] = 0.0f;
	    	balanceController->dq[2] = 0.0f;
	    	balanceController->dq[3] = 0.0f;
	    }

		/* Send State Estimates message */
		balanceController->SendEstimates();

		/*Debug::printf("qEKF = [%.3f, %.3f, %.3f, %.3f]\n", balanceController->q[0], balanceController->q[1], balanceController->q[2], balanceController->q[3]);
		float YPR[3];
		Quaternion_quat2eul_zyx(balanceController->q, YPR);
		Debug::printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", rad2deg(YPR[2]), rad2deg(YPR[1]), rad2deg(YPR[0]));
		//Debug::printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", microsTimer.GetTime(), imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2], imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2]);*/


	    /* Reference generation - get references */
	    balanceController->ReferenceGeneration(params, velocityController); // this function updates q_ref and omega_ref

	    /* Compute internal q_ref, omega_ref_body and omega_ref_inertial based on mode and setpoints */

	    /* Compute control output based on references */
	    if (params.controller.Type == Parameters::LQR_CONTROLLER) {
	    	lqr.Step(balanceController->q, balanceController->dq, balanceController->q_ref, balanceController->omega_ref_body, Torque);
		} else if (params.controller.Type == Parameters::SLIDING_MODE_CONTROLLER) {
			// OBS. When running the Sliding Mode controller, inertial angular velocity reference is needed
			float S[3];
	    	sm.Step(balanceController->q, balanceController->dq, balanceController->xy, balanceController->dxy, balanceController->q_ref, balanceController->omega_ref_inertial, Torque, S);
		} else {
			// Undefined controller mode - set torque output to 0
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

	    /* Clamp the torque outputs between configured limits */
	    if (params.controller.EnableTorqueSaturation) {
	    	Torque[0] = fmax(fmin(Torque[0], params.controller.TorqueMax), -params.controller.TorqueMax);
	    	Torque[1] = fmax(fmin(Torque[1], params.controller.TorqueMax), -params.controller.TorqueMax);
	    	Torque[2] = fmax(fmin(Torque[2], params.controller.TorqueMax), -params.controller.TorqueMax);
	    }

	    /* Initial Torque ramp up */
	    if (params.controller.TorqueRampUp && !TorqueRampUpFinished) {
	    	Torque[0] *= TorqueRampUpGain;
	    	Torque[1] *= TorqueRampUpGain;
	    	Torque[2] *= TorqueRampUpGain;

	    	TorqueRampUpGain += 1.0f / (params.controller.TorqueRampUpTime * params.controller.SampleRate); // ramp up rate
	    	if (TorqueRampUpGain >= 1.0) {
	    		TorqueRampUpFinished = true;
	    	}
	    }

	    /* Torque output LPF filtering */
	    if (params.controller.EnableTorqueLPF) {
			Torque[0] = Motor1_LPF.Filter(Torque[0]);
			Torque[1] = Motor2_LPF.Filter(Torque[1]);
			Torque[2] = Motor3_LPF.Filter(Torque[2]);
	    }

		/* Set control output */
		motor1.SetTorque(Torque[0]);
		motor2.SetTorque(Torque[1]);
		motor3.SetTorque(Torque[2]);

		dt_meas = microsTimer.GetDeltaTime(prevTimer);
		dt_meas2 = HAL_toc(timerPrev);

		//Debug::printf("Balance controller compute time: %9.7f s\n", dt_meas);

		balanceController->SendControllerInfo(params.controller.Type, params.controller.Mode, Torque, dt_meas);
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

	/* Stop and delete task */
	balanceController->isRunning_ = false;
	balanceController->TaskHandle_ = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

/*
void BalanceController::SetReference(const float q_ref_[4], const float omega_ref_[3])
{
	q_ref[0] = q_ref_[0];
	q_ref[1] = q_ref_[1];
	q_ref[2] = q_ref_[2];
	q_ref[3] = q_ref_[3];
	omega_ref[0] = omega_ref_[0];
	omega_ref[1] = omega_ref_[1];
	omega_ref[2] = omega_ref_[2];
	PropagateQuaternionReference = false;
}

void BalanceController::SetReference(const float omega_ref_[3])
{
	omega_ref[0] = omega_ref_[0];
	omega_ref[1] = omega_ref_[1];
	omega_ref[2] = omega_ref_[2];
	PropagateQuaternionReference = true;
}
*/


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
void BalanceController::ReferenceGeneration(Parameters& params, QuaternionVelocityControl& velocityController)
{
	float dt;
	dt = microsTimer.GetDeltaTime(prevTimerValue);
	prevTimerValue = microsTimer.Get();

    if (params.behavioural.JoystickVelocityControl) {
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

		// Compute velocity control based on joystick reference
		float VelRef[2] = {ref_dx, ref_dy};
		headingReference += ref_dyaw * dt;
		omega_ref_inertial[2] = ref_dyaw; // set omega_ref_z as joystick yawdot reference based on assumption of "close to upright" position
		omega_ref_body[2] = ref_dyaw; // set omega_ref_z as joystick yawdot reference based on assumption of "close to upright" position
		velocityController.Step(q, dq, dxy, VelRef, true, headingReference, q_ref);
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
    else if (params.behavioural.StepTestEnabled) {
    	ReferenceGenerationStep++;
		if (ReferenceGenerationStep > 8*params.controller.SampleRate) // reset after 8 seconds
			ReferenceGenerationStep = 0;

		if (ReferenceGenerationStep < 2*params.controller.SampleRate) { // from 0-2 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(0), q_ref);
		}
		else if (ReferenceGenerationStep < 4*params.controller.SampleRate) { // from 2-4 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(5), q_ref);
		}
		else if (ReferenceGenerationStep < 6*params.controller.SampleRate) { // from 4-6 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(0), q_ref);
		}
		else if (ReferenceGenerationStep < 8*params.controller.SampleRate) { // from 6-8 seconds
			Quaternion_eul2quat_zyx(0, 0, deg2rad(-5), q_ref);
		}
    }

    if (params.behavioural.IndependentHeading) {
    	HeadingIndependentReferenceManual(q_ref, q, q_ref);
    }

    PropagateQuaternionReference = false;
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

void BalanceController::SendRawSensors(const IMU::Measurement_t& imuMeas, const float EncoderAngle[3])
{
	lspc::MessageTypesToPC::RawSensor_IMU_MPU9250_t imu_msg;
	lspc::MessageTypesToPC::RawSensor_Encoders_t encoders_msg;

	imu_msg.time = microsTimer.GetTime();
	imu_msg.accelerometer.x = imuMeas.Accelerometer[0];
	imu_msg.accelerometer.y = imuMeas.Accelerometer[1];
	imu_msg.accelerometer.z = imuMeas.Accelerometer[2];
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

void BalanceController::SendControllerInfo(const Parameters::controllerType_t Type, const Parameters::controllerMode_t Mode, const float Torque[3], const float ComputeTime)
{
	lspc::MessageTypesToPC::ControllerInfo_t msg;

	msg.time = microsTimer.GetTime();
	msg.type = (uint8_t)Type;
	msg.mode = (uint8_t)Mode;
	msg.torque1 = Torque[0];
	msg.torque2 = Torque[0];
	msg.torque3 = Torque[0];
	msg.compute_time = ComputeTime;

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
	}

	/* Check current control mode as a calibration can not be performed if the robot is balancing */
	Parameters * params = new Parameters;
	Parameters::controllerMode_t mode = params->controller.Mode;
	delete(params);
	if (mode != Parameters::OFF) {
		// Send acknowledge back to PC
		msgAck.acknowledged = false;
		balanceController->com.TransmitAsync(lspc::MessageTypesToPC::CalibrateIMUAck, (uint8_t *)&msgAck, sizeof(msgAck));
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
