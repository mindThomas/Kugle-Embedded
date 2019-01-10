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
 
#include "AttitudeController.h"
#include "cmsis_os.h"

#include "Debug.h"
#include "LQR.h"
#include "QuaternionVelocityControl.h"
#include "SlidingMode.h"
#include "QEKF.h"
#include "MadgwickAHRS.h"
#include "COMEKF.h"
#include "VelocityEKF.h"
#include "Kinematics.h"
#include "Quaternion.h"

void StabilizeFilters(Parameters& params, IMU& imu, QEKF& qEKF, Madgwick& madgwick, TickType_t loopWaitTicks, float stabilizationTime);

AttitudeController::AttitudeController(Parameters& params_, IMU& imu_, ESCON& motor1_, ESCON& motor2_, ESCON& motor3_, LSPC& com_, Timer& microsTimer_) : _TaskHandle(0), _isRunning(false), _shouldStop(false), params(params_), imu(imu_), motor1(motor1_), motor2(motor2_), motor3(motor3_), com(com_), microsTimer(microsTimer_)
{
	Start();
}

AttitudeController::~AttitudeController()
{
	_shouldStop = true;
	while (_isRunning) osDelay(10);
}

int AttitudeController::Start()
{
	if (_isRunning) return 0; // task already running
	return xTaskCreate( AttitudeController::Thread, (char *)"Attitude Controller", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);
}

int AttitudeController::Stop(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running

	_shouldStop = true;
	osDelay(timeout);
	if (_isRunning) return -1; // timeout trying to stop task
	return 1;
}

int AttitudeController::Restart(uint32_t timeout)
{
	if (!_isRunning) return 0; // task not running
	int errCode = Stop(timeout);
	if (errCode != 1) return errCode;
	return Start();
}

void AttitudeController::Thread(void * pvParameters)
{
	AttitudeController * task = (AttitudeController *)pvParameters;
	TickType_t xLastWakeTime;
	task->_isRunning = true;

	/* Load initialized objects */
	Parameters& params = task->params;
	IMU& imu = task->imu;
	ESCON& motor1 = task->motor1;
	ESCON& motor2 = task->motor2;
	ESCON& motor3 = task->motor3;
	LSPC& com = task->com;
	Timer& microsTimer = task->microsTimer;

	/* Controller loop time / sample rate */
	TickType_t loopWaitTicks = configTICK_RATE_HZ / params.controller.SampleRate;

	/* Create and initialize controller and estimator objects */
	LQR& lqr = *(new LQR(task->params));
	SlidingMode& sm = *(new SlidingMode(task->params));
	QuaternionVelocityControl& velocityController = *(new QuaternionVelocityControl(task->params, &task->microsTimer, 1.0f / task->params.controller.SampleRate));
	QEKF& qEKF = *(new QEKF(task->params, &task->microsTimer));
	Madgwick& madgwick = *(new Madgwick(task->params.controller.SampleRate, task->params.estimator.MadgwickBeta));
	VelocityEKF& velocityEKF = *(new VelocityEKF(task->params, &task->microsTimer));
	COMEKF& comEKF = *(new COMEKF(task->params, &task->microsTimer));
	Kinematics& kinematics = *(new Kinematics(task->params));

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

	StabilizeFilters(params, imu, qEKF, madgwick, loopWaitTicks, 1.0f); // stabilize estimators for 1 second

	/* Reset COM estimate */
	task->COM[0] = 0;
	task->COM[1] = 0;
	task->COM[2] = params.model.l; // initialize COM directly above center of ball at height L

	/* Reset position estimate */
	task->xy[0] = 0;
	task->xy[1] = 0;

	/* Reset reference variables */
	task->q_ref[0] = 1; // attitude reference = just upright
	task->q_ref[1] = 0;
	task->q_ref[2] = 0;
	task->q_ref[3] = 0;
	task->omega_ref[0] = 0; // zero angular velocity reference
	task->omega_ref[1] = 0;
	task->omega_ref[2] = 0;
	task->PropagateQuaternionReference = false;
	task->headingReference = 0; // consider to replace this with current heading (based on estimate of stabilized QEKF filter)
	task->ReferenceGenerationStep = 0; // only used if test reference generation is enabled
	task->prevTimerValue = microsTimer.Get();

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
	while (!task->_shouldStop) {
		/* Wait until time has been reached to make control loop periodic */
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);

		/* Get measurements (sample) */
		imu.Get(imuMeas);
		EncoderTicks[0] = motor1.GetEncoderRaw();
		EncoderTicks[1] = motor2.GetEncoderRaw();
		EncoderTicks[2] = motor3.GetEncoderRaw();
		EncoderAngle[0] = motor1.GetAngle();
		EncoderAngle[1] = motor2.GetAngle();
		EncoderAngle[2] = motor3.GetAngle();

		/* Attitude estimation */
		if (params.estimator.UseMadgwick) {
			madgwick.updateIMU(imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2], imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2]);
			madgwick.getQuaternion(task->q);
			madgwick.getQuaternionDerivative(task->dq);
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
			qEKF.GetQuaternion(task->q);
			qEKF.GetQuaternionDerivative(task->dq);
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
	      dq_tmp[0] = task->dq[0];
	      dq_tmp[1] = task->dq[1];
	      dq_tmp[2] = task->dq[2];
	      dq_tmp[3] = task->dq[3];
	      HeadingIndependentQdot(dq_tmp, task->q, task->dq);
	    }

	    /* Velocity estimation using kinematics */
	    if (!params.estimator.UseVelocityEstimator) {
	    	// compute velocity from encoder-based motor velocities and forward kinematics
	    	kinematics.EstimateMotorVelocity(EncoderAngle);
			if (params.estimator.Use2Lvelocity) {
				float dxy_ball[2];
				kinematics.ForwardKinematics(task->q, task->dq, dxy_ball);
				kinematics.ConvertBallTo2Lvelocity(dxy_ball, task->q, task->dq, task->dxy);            // put 2L velocity into dxy
			} else {
				kinematics.ForwardKinematics(task->q, task->dq, task->dxy);       // put ball velocity into dxy
			}
	    }

	    /* Velocity estimation using velocity EKF */
	    if (params.estimator.UseVelocityEstimator) {
	    	velocityEKF.Step(EncoderTicks, task->q, Cov_q, task->dq, task->COM); // velocity estimator estimates 2L velocity
	    	velocityEKF.GetVelocity(task->dxy);
	    	velocityEKF.GetVelocityCovariance(Cov_dxy);

	    	// OBS. dxy was in the original design supposed to be ball velocity, but the velocity estimator estimates the 2L velocity which gives indirect "stabilization"
	    	// In the Sliding Mode controller this velocity is (only) used to calculated "feedforward" torque to counteract friction
	        if (!params.estimator.Use2Lvelocity) { // however if the 2L velocity is not desired, it is here converted back to ball velocity
	        	kinematics.Convert2LtoBallVelocity(task->dxy, task->q, task->dq, task->dxy);
	        }
	    }

	    /* Center of Mass estimation */
	    if (params.estimator.EstimateCOM && params.estimator.UseVelocityEstimator) { // can only estimate COM if velocity is also estimated (due to need of velocity estimate covariance)
	    	comEKF.Step(task->dxy, Cov_dxy, task->q, Cov_q, task->dq);
	    	comEKF.GetCOM(task->COM);
	    }

	    /* Disable dq to avoid noisy control outputs resulting from noisy dq estimates */
	    if (params.controller.DisableQdot) { // q_dot removed because it is VERY noisy - this causes oscillations on yaw, if yaw reference is included
	    	task->dq[0] = 0.0f;
	    	task->dq[1] = 0.0f;
	    	task->dq[2] = 0.0f;
	    	task->dq[3] = 0.0f;
	    }

	    /* Reference generation - get references */
	    task->ReferenceGeneration(velocityController); // this function updates q_ref and omega_ref

	    uint32_t prevTimer = microsTimer.Get();
		uint32_t timerPrev = HAL_tic();

		/* Compute control output based on references */
	    lqr.Step(task->q, task->dq, task->q_ref, task->omega_ref, Torque);
	    float S[3];
	    sm.Step(task->q, task->dq, task->xy, task->dxy, task->q_ref, task->omega_ref, Torque, S);

		dt_meas = microsTimer.GetDeltaTime(prevTimer);
		dt_meas2 = HAL_toc(timerPrev);

		Debug::printf("dt: %9.7f \n", dt_meas);

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
	}
	/* End of control loop */

	/* Disable motor outputs */
	motor1.Disable();
	motor2.Disable();
	motor3.Disable();

	/* Clear controller and estimator objects */
	delete(&lqr);
	delete(&velocityController);
	delete(&qEKF);
	delete(&madgwick);
	delete(&velocityEKF);
	delete(&comEKF);
	delete(&kinematics);

	/* Stop and delete task */
	task->_isRunning = false;
	task->_TaskHandle = 0;
	vTaskDelete(NULL); // delete/stop this current task
}

void AttitudeController::SetReference(const float q_ref_[4], const float omega_ref_[3])
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

void AttitudeController::SetReference(const float omega_ref_[3])
{
	omega_ref[0] = omega_ref_[0];
	omega_ref[1] = omega_ref_[1];
	omega_ref[2] = omega_ref_[2];
	PropagateQuaternionReference = true;
}


/* Initialize/stabilize estimators for certain stabilization time */
void StabilizeFilters(Parameters& params, IMU& imu, QEKF& qEKF, Madgwick& madgwick, TickType_t loopWaitTicks, float stabilizationTime)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t finishTick = xLastWakeTime + configTICK_RATE_HZ * stabilizationTime;

	IMU::Measurement_t imuMeas;

	while (xLastWakeTime < finishTick) {
		// Wait until time has been reached to make control loop periodic
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);

		// Get IMU measurement
		imu.Get(imuMeas);

		// Compute attitude estimate
		if (params.estimator.UseMadgwick) {
			madgwick.updateIMU(imuMeas.Gyroscope[0], imuMeas.Gyroscope[1], imuMeas.Gyroscope[2], imuMeas.Accelerometer[0], imuMeas.Accelerometer[1], imuMeas.Accelerometer[2], 0.1); // use larger beta to make filter converge to current angle by trusting the accelerometer more
		} else {
			qEKF.Step(imuMeas.Accelerometer, imuMeas.Gyroscope, false); // do not estimate bias while stabilizing the filter
		}
	}
}

/* Reference generation based on selected test */
void AttitudeController::ReferenceGeneration(QuaternionVelocityControl& velocityController)
{
	float dt;
	dt = microsTimer.GetDeltaTime(prevTimerValue);
	prevTimerValue = microsTimer.Get();

    if (params.behavioural.JoystickVelocityControl) {
		// Get velocity references from joystick
		float ref_xdot = 0.0f;
		float ref_ydot = 0.0f;
		float ref_yawdot = deg2rad(10);

		// Compute velocity control based on joystick reference
		float VelRef[2] = {ref_xdot, ref_ydot};
		headingReference += ref_yawdot * dt;
		omega_ref[2] = ref_yawdot; // set omega_ref_z as joystick yawdot reference based on assumption of "close to upright" position
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
