/* Copyright (C) 2018 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */
 
#include "LQR.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

#include "Quaternion.h"
#include "Matrix.h"
#include "Parameters.h"

/*#include "mass.h"
#include "coriolis.h"
#include "gravity.h"
#include "friction.h"
#include "input_forces.h"*/

LQR::LQR(Parameters& params) : _params(params)
{
}

LQR::~LQR()
{
}


/**
 * @brief 	Compute control output with LQR controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in body frame   (OBS. Notice this is body frame)
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void LQR::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	Step(q, dq, q_ref, omega_ref, tau);
}

/**
 * @brief 	Compute control output with LQR controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in body frame   (OBS. Notice this is body frame)
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void LQR::Step(const float q[4], const float dq[4], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	// See ARM-CMSIS DSP library for matrix operations: https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html

	/* For equivalent torque computation the following code can be used
	float M[6*6]; arm_matrix_instance_f32 M_; arm_mat_init_f32(&M_, 6, 6, M);
	float C[6*6]; arm_matrix_instance_f32 C_; arm_mat_init_f32(&C_, 6, 6, C);
	float G[6]; arm_matrix_instance_f32 G_; arm_mat_init_f32(&G_, 6, 1, G);
	float D[6]; arm_matrix_instance_f32 D_; arm_mat_init_f32(&D_, 6, 1, D);
	float Q[6*3]; arm_matrix_instance_f32 Q_; arm_mat_init_f32(&Q_, 6, 3, Q);

	mass(_COM_X, _COM_Y, _COM_Z, _Jbx, _Jby, _Jbz, _Jk, _Jw, _Mb, _Mk, *q0, *q1, *q2, *q3, _rk, _rw, M);
	coriolis(_COM_X, _COM_Y, _COM_Z, _Jbx, _Jby, _Jbz, _Jw, _Mb, 0.0f, *dq0, *dq1, *dq2, *dq3, *dx, *dy, *q0, *q1, *q2, *q3, _rk, _rw, C); // beta = 0
	gravity(_COM_X, _COM_Y, _COM_Z, _Mb, 0.0f, _g, *q0, *q1, *q2, *q3, G); // beta = 0
	friction(_Bvb, _Bvk, _Bvm, 0.0f, *dq0, *dq1, *dq2, *dq3, *dx, *dy, *q0, *q1, *q2, *q3, _rk, _rw, D);
	input_forces(*q0, *q1, *q2, *q3, _rk, _rw, Q);*/

	/* Form error state vector */
	// Since the LQR is designed on error dynamics only for attitude control,
	// we will have to convert the full state estimate vector into a reduced error state vector for the attitude
	float X_err[7]; arm_matrix_instance_f32 X_err_; arm_mat_init_f32(&X_err_, 7, 1, (float32_t *)X_err);
	float * q_err = &X_err[0]; arm_matrix_instance_f32 q_err_; arm_mat_init_f32(&q_err_, 4, 1, (float32_t *)q_err);
	float * omega_err = &X_err[4]; arm_matrix_instance_f32 omega_err_; arm_mat_init_f32(&omega_err_, 3, 1, (float32_t *)omega_err);

	/* Quaternion error in Body frame */
	// q_err = Phi(q_ref)^T * q
	Quaternion_PhiT(q_ref, q, q_err);
	/* Clamp "yaw" error - this is possibly a crude way of doing it */
	q_err[4] = fmax(fmin(q_err[4], 1/2*deg2rad(_params.controller.LQR_MaxYawError)), -1/2*deg2rad(_params.controller.LQR_MaxYawError));

	/* Body angular velocity */
	// omeg_err = 2*devec*Phi(q)^T*dq - omega_ref
	Quaternion_devecPhiT(q, dq, omega_err); // devec*Phi(q)^T*dq
	arm_scale_f32(omega_err, 2.0f, omega_err, 3); // 2*devec*Phi(q)^T*dq
	arm_sub_f32(omega_err, (float*)omega_ref, omega_err, 3); // 2*devec*Phi(q)^T*dq - omega_ref

	/* Compute equillibrium/linearized torque */
	// Initially we will just set the equillibrium/linearized torque to 0, since we have linearized around upright and with zero velocity
	float tau0[3] = {0, 0, 0};

	/* Compute control torque by matrix multiplication with LQR gain */
	arm_matrix_instance_f32 LQR_K_; arm_mat_init_f32(&LQR_K_, 3, 6, (float32_t *)_params.controller.LQR_K);
	arm_matrix_instance_f32 X_err_reduced_; arm_mat_init_f32(&X_err_reduced_, 6, 1, (float32_t *)&X_err[1]); // removes q0 (first element)
	float tau_control[3]; arm_matrix_instance_f32 tau_control_; arm_mat_init_f32(&tau_control_, 3, 1, (float32_t *)tau_control);
	arm_mat_mult_f32(&LQR_K_, &X_err_reduced_, &tau_control_); // tau = -K * X_err_reduced
	arm_negate_f32(tau_control, tau_control, 3);

	/* Add linearized torque and computed torque and set output */
	arm_add_f32(tau0, tau_control, tau, 3); // tau = tau0 + tau_control
}
