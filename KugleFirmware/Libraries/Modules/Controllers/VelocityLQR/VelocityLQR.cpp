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
 
#include "VelocityLQR.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>
#include <cmath> // for fminf, fmaxf, fabsf

#include "Quaternion.h"
#include "MathLib.h"
#include "Matrix.h"
#include "Parameters.h"

#include "mass.h"
#include "coriolis.h"
#include "gravity.h"
#include "friction.h"
#include "input_forces.h"

VelocityLQR::VelocityLQR(Parameters& params, Timer * microsTimer, float SamplePeriod) : _params(params), _microsTimer(microsTimer), _omega_x_ref_filt(SamplePeriod, params.controller.VelocityController_OmegaLPFtau), _omega_y_ref_filt(SamplePeriod, params.controller.VelocityController_OmegaLPFtau)
{
	Reset();
}

VelocityLQR::VelocityLQR(Parameters& params, float SamplePeriod) : _params(params), _microsTimer(0), _omega_x_ref_filt(SamplePeriod, params.controller.VelocityController_OmegaLPFtau), _omega_y_ref_filt(SamplePeriod, params.controller.VelocityController_OmegaLPFtau)
{
	Reset();
}

VelocityLQR::~VelocityLQR()
{
}

void VelocityLQR::Reset()
{
	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;

	// Reset roll and pitch reference variables (used to integrate the computed angular velocity reference)
	roll_ref = 0;
	pitch_ref = 0;

	position_control_enabled = false;

	// Reset position reference used for station keeping
	position_reference[0] = 0;
	position_reference[1] = 0;

	// Reset velocity error integral
	velocity_error_integral[0] = 0;
	velocity_error_integral[1] = 0;

	VelocityIntegralAfterPowerup = true;
	VelocityIntegralInitializationTime = _params.controller.VelocityLQR_IntegratorPowerupStabilizeTime; // wait 5 seconds in the beginning for integrator to settle (and before allowing manual movement)
}


void VelocityLQR::Step(const float xy[2], const float q[4], const float dxy[2], const float dq[4], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float omega_heading, float q_ref_out[4], float omega_body_ref_out[3])
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	Step(xy, q, dxy, dq, velocityRef, velocityRefGivenInHeadingFrame, headingRef, omega_heading, _params.controller.VelocityLQR_VelocityClamp, _params.controller.VelocityLQR_AngularVelocityClamp, _params.controller.VelocityControl_AccelerationLimit, _params.controller.VelocityController_MaxTilt, _params.controller.VelocityLQR_IntegralEnabled,  _params.controller.VelocityLQR_PositionControlAtZeroVelocityReference, _params.controller.VelocityLQR_PositionControlAtZeroVelocityReference_MaximumKickinVelocity, _params.controller.VelocityController_StabilizationDetectionVelocity, _params.controller.VelocityController_AngleLPFtau, _params.controller.VelocityController_OmegaLPFtau, !_params.controller.VelocityControl_UseOmegaRef, _params.controller.VelocityLQR_K, dt, q_ref_out, omega_body_ref_out);
}


/**
 * @brief 	Compute quaternion and angular velocity reference given a desired velocity reference
 * @param	q[4]      	   Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	   Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	   Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	   Input: current ball (center) velocity defined in inertial frame
 * @param	velocityRef[2] Input: desired velocity defined in either inertial frame or heading frame  (specified with the flag 'velocityRefGivenInHeadingFrame')   [m/s]
 * @param	velocityRefGivenInHeadingFrame    Input: specified which frame 'velocityRef' is defined in
 * @param	headingRef     Input: desired heading [rad]
 * @param	omega_heading_ref  Input: desired angular velocity around z-axis (yaw/heading)   [rad/s]
 */
void VelocityLQR::Step(const float xy[2], const float q[4], const float dxy[2], const float dq[4], const float velocityRef[2], const bool velocityRefGivenInHeadingFrame, const float headingRef, const float omega_heading, const float velocity_error_clamp, const float angular_velocity_clamp, const float acceleration_limit, const float MaxTilt, const bool VelocityIntegralEnabled, const bool PositionControlAtZeroVelocityReference, const float PositionControlAtZeroVelocityReference_MaximumKickinVelocity, const float StabilizationDetectionVelocity, const float angle_lpf_tau, const float omega_lpf_tau, const bool DoNotSetOmegaRef, const float * gainMatrix, const float dt, float q_ref_out[4], float omega_body_ref_out[3])
{
	// See ARM-CMSIS DSP library for matrix operations: https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html

	/* Form error state vector */
	// The error state is linearized in the heading frame. Hence both velocity error, quaternions etc. has to be transformed into heading frame (rotated with heading)
	// X_err = [dx_err_integral, dy_err_integral, q2, q3, dx_err, dy_err, dq2, dq3, q2_ref_prev, q3_ref_prev]
	float X_err[10]; arm_matrix_instance_f32 X_err_; arm_mat_init_f32(&X_err_, 10, 1, (float32_t *)X_err);
	float * pos_err_heading = &X_err[0];
	float * vel_err_integral_heading = &X_err[0];
	float * q_tilt_xy = &X_err[2];
	float * vel_err_heading = &X_err[4];
	float * dq_tilt_xy = &X_err[6];
	float * q_ref_tilt_xy = &X_err[8];

	/* Extract current heading */
	float heading = HeadingFromQuaternion(q);
	float q_heading[4];
	HeadingQuaternion(q, q_heading);

	/* Convert quaternion parts into heading frame */
	/* Compute previously set reference quaternion */
	float q_ref_tilt[4];
	Quaternion_eul2quat_zyx(0, pitch_ref, roll_ref, q_ref_tilt);

	/* Extract tilt quaternion */
	float q_tilt[4]; // current tilt
	/* q_tilt = q_heading* o q; */
	Quaternion_PhiT(q_heading, q, q_tilt);

	/* Extract body angular velocity */
	float omega_body[3];
	Quaternion_GetAngularVelocity_Body(q, dq, omega_body);

	/* Compute quaternion derivative in heading frame */
	float dq_tilt[4];
	/* dq_tilt = 1/2 * Phi(q_tilt) * [0;omega_body]; */
	Quaternion_GetDQ_FromBody(q_tilt, omega_body, dq_tilt);

	/* Set state vector with quaternion states */
	q_ref_tilt_xy[0] = q_ref_tilt[1];
	q_ref_tilt_xy[1] = q_ref_tilt[2];
	q_tilt_xy[0] = q_tilt[1];
	q_tilt_xy[1] = q_tilt[2];
	dq_tilt_xy[0] = dq_tilt[1];
	dq_tilt_xy[1] = dq_tilt[2];

	/* Update position reference if we are supposed to be moving */
	if (!position_control_enabled) {
		position_reference[0] = xy[0];
		position_reference[1] = xy[1];
	}

	/* Enable position control (station keeping) if velocity reference is zero */
	if (!VelocityIntegralAfterPowerup && PositionControlAtZeroVelocityReference && velocityRef[0] == 0 && velocityRef[1] == 0 && (dxy[0]*dxy[0] + dxy[1]*dxy[1] < PositionControlAtZeroVelocityReference_MaximumKickinVelocity*PositionControlAtZeroVelocityReference_MaximumKickinVelocity)) {
		position_control_enabled = true;
	} else {
		position_control_enabled = false;
	}

	if (VelocityIntegralInitializationTime > 0) {
		VelocityIntegralInitializationTime -= dt;
		if (VelocityIntegralInitializationTime < 0) VelocityIntegralInitializationTime = 0;

		if (velocityRef[0] != 0 || velocityRef[1] != 0) { // velocity reference demands movement
			VelocityIntegralInitializationTime = 0;
			VelocityIntegralAfterPowerup = false;
		}
	}
	else if (VelocityIntegralAfterPowerup) {
		if (velocityRef[0] != 0 || velocityRef[1] != 0 ||
			(dxy[0]*dxy[0] + dxy[1]*dxy[1] > StabilizationDetectionVelocity*StabilizationDetectionVelocity)) { // manual movement of velocity reference requires the initialization integral to be disabled
			VelocityIntegralInitializationTime = 0;
			VelocityIntegralAfterPowerup = false;
		}
	}

	/* Compute position error */
	float pos_err[2];
	pos_err[0] = xy[0] - position_reference[0];
	pos_err[1] = xy[1] - position_reference[1];

	/* Rotate position error from inertial frame into heading frame */
	Math_Rotate2D(pos_err, -heading, pos_err_heading);

	/* Compute velocity error */
	float vel_err[2];
	vel_err[0] = dxy[0];
	vel_err[1] = dxy[1];

	if (!velocityRefGivenInHeadingFrame) {
		vel_err[0] -= velocityRef[0];
		vel_err[1] -= velocityRef[1];
	}

	/* Rotation velocity error from inertial frame into heading frame */
	Math_Rotate2D(vel_err, -heading, vel_err_heading);

	if (velocityRefGivenInHeadingFrame) {
		vel_err_heading[0] -= velocityRef[0];
		vel_err_heading[1] -= velocityRef[1];
	}

	/* Compute reference in different frames for debugging */
	if (velocityRefGivenInHeadingFrame) {
		Velocity_Reference_Filtered_Heading[0] = velocityRef[0];
		Velocity_Reference_Filtered_Heading[1] = velocityRef[1];
		Math_Rotate2D(velocityRef, heading, Velocity_Reference_Filtered_Inertial);
	} else {
		Velocity_Reference_Filtered_Inertial[0] = velocityRef[0];
		Velocity_Reference_Filtered_Inertial[1] = velocityRef[1];
		Math_Rotate2D(velocityRef, -heading, Velocity_Reference_Filtered_Heading);
	}

	/* Clamp velocity error */
	vel_err_heading[0] = fminf(fmaxf(vel_err_heading[0], -velocity_error_clamp), velocity_error_clamp);
	vel_err_heading[1] = fminf(fmaxf(vel_err_heading[1], -velocity_error_clamp), velocity_error_clamp);

	/* Include integral term as current integral value */
	vel_err_integral_heading[0] += velocity_error_integral[0];
	vel_err_integral_heading[1] += velocity_error_integral[1];

	/* Update integral */
	if (VelocityIntegralEnabled || VelocityIntegralAfterPowerup) {
		// Rate limit the integration
		velocity_error_integral[0] += dt * fmaxf(fminf(vel_err_heading[0], 0.05), -0.05);
		velocity_error_integral[1] += dt * fmaxf(fminf(vel_err_heading[1], 0.05), -0.05);
	}

	/* Compute angular velocity reference by matrix multiplication with LQR gain */
	arm_matrix_instance_f32 LQR_K_; arm_mat_init_f32(&LQR_K_, 2, 10, (float32_t *)gainMatrix);
	float omega_ref_control[3]; arm_matrix_instance_f32 omega_ref_control_; arm_mat_init_f32(&omega_ref_control_, 2, 1, (float32_t *)omega_ref_control);
	arm_mat_mult_f32(&LQR_K_, &X_err_, &omega_ref_control_); // omega_body_ref_xy = -K * X_err
	arm_negate_f32(omega_ref_control, omega_ref_control, 2);

	/* Saturate angular velocity output */
	omega_ref_control[0] = fminf(fmaxf(omega_ref_control[0], -angular_velocity_clamp), angular_velocity_clamp);
	omega_ref_control[1] = fminf(fmaxf(omega_ref_control[1], -angular_velocity_clamp), angular_velocity_clamp);

	/* Filter angular velocity before setting as output */
	_omega_x_ref_filt.ChangeTimeconstant(omega_lpf_tau); // ensure that the LPF has the latest time constant (if parameter has been changed)
	_omega_y_ref_filt.ChangeTimeconstant(omega_lpf_tau);
	omega_ref_control[0] = _omega_x_ref_filt.Filter(omega_ref_control[0]);
	omega_ref_control[1] = _omega_y_ref_filt.Filter(omega_ref_control[1]);

	/* Set the output angular velocity */
	omega_body_ref_out[0] = omega_ref_control[0];
	omega_body_ref_out[1] = omega_ref_control[1];
	omega_body_ref_out[2] = omega_heading;

	if (DoNotSetOmegaRef) {
		omega_body_ref_out[0] = 0;
		omega_body_ref_out[1] = 0;
	}

	/* Integrate angle reference */
	roll_ref += dt * omega_ref_control[0];
	pitch_ref += dt * omega_ref_control[1];

	/* Saturate roll and pitch references */
	roll_ref = fminf(fmaxf(roll_ref, -MaxTilt), MaxTilt);
	pitch_ref = fminf(fmaxf(pitch_ref, -MaxTilt), MaxTilt);

	/* Compute and set output quaternion */
	Quaternion_eul2quat_zyx(headingRef, pitch_ref, roll_ref, q_ref_out);
}

void VelocityLQR::GetFilteredVelocityReference_Inertial(float velocity_reference_inertial[2])
{
	velocity_reference_inertial[0] = Velocity_Reference_Filtered_Inertial[0];
	velocity_reference_inertial[1] = Velocity_Reference_Filtered_Inertial[1];
}

void VelocityLQR::GetFilteredVelocityReference_Heading(float velocity_reference_heading[2])
{
	velocity_reference_heading[0] = Velocity_Reference_Filtered_Heading[0];
	velocity_reference_heading[1] = Velocity_Reference_Filtered_Heading[1];
}

bool VelocityLQR::UnitTest(void)
{
	// Unit test not implemented
	return false;
}
