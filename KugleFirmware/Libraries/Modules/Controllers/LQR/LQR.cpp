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
 
#include "LQR.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

#include "Quaternion.h"
#include "Matrix.h"
#include "Parameters.h"

#include "mass.h"
#include "coriolis.h"
#include "gravity.h"
#include "friction.h"
#include "input_forces.h"

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
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in body frame   (OBS. Notice this is body frame)
 * @param	UseSteadyStateTorque   Input: boolean to control whether to add steady state torque calculated at the reference state using the nominal model
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void LQR::Step(const float q[4], const float dq[4], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	const float xy[2] = { 0, 0 };
	const float dxy[2] = { 0, 0 };
	const float COM[3] = { _params.model.COM_X,_params.model.COM_Y, _params.model.COM_Z };
	Step(q, dq, xy, dxy, COM, q_ref, omega_ref, (float *)_params.controller.BalanceLQR_K, _params.controller.EquivalentControl, tau);
}

/**
 * @brief 	Compute control output with LQR controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in body frame   (OBS. Notice this is body frame)
 * @param	UseSteadyStateTorque   Input: boolean to control whether to add steady state torque calculated at the reference state using the nominal model
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void LQR::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	const float COM[3] = { _params.model.COM_X,_params.model.COM_Y, _params.model.COM_Z };
	Step(q, dq, xy, dxy, COM, q_ref, omega_ref, (float *)_params.controller.BalanceLQR_K, _params.controller.EquivalentControl, tau);
}

/**
 * @brief 	Compute control output with LQR controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	COM[3]    	  Input: current estimate of COM
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in body frame   (OBS. Notice this is body frame)
 * @param	UseSteadyStateTorque   Input: boolean to control whether to add steady state torque calculated at the reference state using the nominal model
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void LQR::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float COM[3], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	Step(q, dq, xy, dxy, COM, q_ref, omega_ref, (float *)_params.controller.BalanceLQR_K, _params.controller.EquivalentControl, tau);
}

/**
 * @brief 	Compute control output with LQR controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	COM[3]    	  Input: current estimate of COM
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in body frame   (OBS. Notice this is body frame)
 * @param   gainMatrix    Input: LQR gain matrix array
 * @param	UseSteadyStateTorque   Input: boolean to control whether to add steady state torque calculated at the reference state using the nominal model
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void LQR::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float COM[3], const float q_ref[4], const float omega_ref[3], const float * gainMatrix, const bool UseSteadyStateTorque, float tau[3])
{
	// See ARM-CMSIS DSP library for matrix operations: https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html

	/* Form error state vector */
	// Since the LQR is designed on error dynamics only for attitude control,
	// we will have to convert the full state estimate vector into a reduced error state vector for the attitude
	// X_err = [q2_err, q3_err, q4_err, omega_body_x_err, omega_body_y_err, omega_body_z_err]
	float X_err[6]; arm_matrix_instance_f32 X_err_; arm_mat_init_f32(&X_err_, 6, 1, (float32_t *)X_err);
	float q_err[4];
	float omega_err[3];

	/* Quaternion error in Body frame */
	// q_err = Phi(q_ref)^T * q
	Quaternion_PhiT(q_ref, q, q_err);

    /* Invert quaternion if the scalar is negative to get the shortest path */
	if (q_err[0] < 0)
      arm_negate_f32(q_err, q_err, 4);

	/* Clamp "yaw" error - this is possibly a crude way of doing it */
	// ToDo: Consider to clamp the yaw error by converting the quaternion error into roll, pitch and yaw components, then clamp the yaw component and reassemble into quaternion error
	q_err[3] = fmaxf(fminf(q_err[3], 0.5f*deg2rad(_params.controller.BalanceLQR_MaxYawError)), -0.5f*deg2rad(_params.controller.BalanceLQR_MaxYawError));

	/* Copy quaternion error into error state vector */
	X_err[0] = q_err[1];
	X_err[1] = q_err[2];
	X_err[2] = q_err[3];

	/* Body angular velocity */
	// omeg_err = 2*devec*Phi(q)^T*dq - omega_ref
	Quaternion_devecPhiT(q, dq, omega_err); // devec*Phi(q)^T*dq
	arm_scale_f32(omega_err, 2.0f, omega_err, 3); // 2*devec*Phi(q)^T*dq
	arm_sub_f32(omega_err, (float*)omega_ref, omega_err, 3); // 2*devec*Phi(q)^T*dq - omega_ref

	/* Copy angular velocity error into error state vector */
	X_err[3] = omega_err[0];
	X_err[4] = omega_err[1];
	X_err[5] = omega_err[2];

	/* Compute equillibrium/linearized/steady state torque */
	float tau_ss[3] = {0, 0, 0}; // steady state torque
	if (UseSteadyStateTorque) {
		/* Body angular velocity */
		/* omega = 2*devec*Phi(q)'*dq */
		/* omega = 2*devec*  q* o dq */
		/* dq = 1/2*  q o vec*omega */
		/* dq_ref = 1/2 * Phi(q_ref) * [0;omega_ref]; */
		float dq_ref[4]; arm_matrix_instance_f32 dq_ref_; arm_mat_init_f32(&dq_ref_, 4, 1, (float32_t *)dq_ref);
		float omega_ref_q[4] = {0, omega_ref[0], omega_ref[1], omega_ref[2]};
		Quaternion_Phi(q_ref, omega_ref_q, dq_ref); // Phi(q_ref) * [0;omega_ref]
		arm_scale_f32(dq_ref, 0.5f, dq_ref, 4);

		float chi_ss[6] = {xy[0], xy[1], q_ref[0], q_ref[1], q_ref[2], q_ref[3]}; arm_matrix_instance_f32 chi_ss_; arm_mat_init_f32(&chi_ss_, 6, 1, (float32_t *)chi_ss);
		float dchi_ss[6] = {dxy[0], dxy[1], dq_ref[0], dq_ref[1], dq_ref[2], dq_ref[3]}; arm_matrix_instance_f32 dchi_ss_; arm_mat_init_f32(&dchi_ss_, 6, 1, (float32_t *)dchi_ss);

		float M[6*6]; arm_matrix_instance_f32 M_; arm_mat_init_f32(&M_, 6, 6, M);
		float C[6*6]; arm_matrix_instance_f32 C_; arm_mat_init_f32(&C_, 6, 6, C);
		float G[6]; arm_matrix_instance_f32 G_; arm_mat_init_f32(&G_, 6, 1, G);
		float D[6]; arm_matrix_instance_f32 D_; arm_mat_init_f32(&D_, 6, 1, D);
		float Q[6*3]; arm_matrix_instance_f32 Q_; arm_mat_init_f32(&Q_, 6, 3, Q);

		mass( COM[0], COM[1], COM[2], _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jk, _params.model.Jw, _params.model.Mb, _params.model.Mk, q_ref[0], q_ref[1], q_ref[2], q_ref[3], _params.model.rk, _params.model.rw, M);
		coriolis( COM[0], COM[1], COM[2], _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.Mb, 0.0f, dq_ref[0], dq_ref[1], dq_ref[2], dq_ref[3], dxy[0], dxy[1], q_ref[0], q_ref[1], q_ref[2], q_ref[3], _params.model.rk, _params.model.rw, C); // beta = 0
		gravity( COM[0], COM[1], COM[2], _params.model.Mb, 0.0f, _params.model.g, q_ref[0], q_ref[1], q_ref[2], q_ref[3], G); // beta = 0
		friction(_params.model.Bvb, _params.model.Bvk, _params.model.Bvm, 0.0f, dq_ref[0], dq_ref[1], dq_ref[2], dq_ref[3], dxy[0], dxy[1], q_ref[0], q_ref[1], q_ref[2], q_ref[3], _params.model.rk, _params.model.rw, D);
		input_forces(q[0], q[1], q[2], q[3], _params.model.rk, _params.model.rw, Q);

		float Minv[6*6]; arm_matrix_instance_f32 Minv_; arm_mat_init_f32(&Minv_, 6, 6, Minv);
		inv6x6(M, Minv);

		/* f_ss = Minv * (-C*dchi - G - D) */
		/* g_ss = Minv * Q */
		float f_ss[6]; arm_matrix_instance_f32 f_ss_; arm_mat_init_f32(&f_ss_, 6, 1, f_ss);
		float g_ss[6*3]; arm_matrix_instance_f32 g_ss_; arm_mat_init_f32(&g_ss_, 6, 3, g_ss);
		float tmp6[6]; arm_matrix_instance_f32 tmp6_; arm_mat_init_f32(&tmp6_, 6, 1, tmp6);
		arm_mat_mult_f32(&C_, &dchi_ss_, &tmp6_);
		arm_add_f32(tmp6, G, tmp6, 6);
		arm_add_f32(tmp6, D, tmp6, 6);
		arm_negate_f32(tmp6, tmp6, 6);
		arm_mat_mult_f32(&Minv_, &tmp6_, &f_ss_);
		arm_mat_mult_f32(&Minv_, &Q_, &g_ss_);

		/* Extract the quaternion elements of the steady state ODE */
		arm_matrix_instance_f32 fq_ss_; arm_mat_init_f32(&fq_ss_, 4, 1, &f_ss[2]); // extract the quaternion elements of f_ss
		float gq_ss[4*3]; arm_matrix_instance_f32 gq_ss_; arm_mat_init_f32(&gq_ss_, 4, 3, gq_ss); // extract the quaternion elements (last 4 rows) of g_ss
		Matrix_Extract(g_ss, 6, 3, 2, 0, 4, 3, gq_ss);

		/* tau_ss = inv(2*devec*Phi(q)'*gq_ss) * (-2*devec*Phi(dq_ref)'*dq_ref - 2*devec*Phi(q)'*fq_ss) */
		/* Note that the current quaternion is used here */
		float devecPhiQ_T[3*4]; arm_matrix_instance_f32 devecPhiQ_T_; arm_mat_init_f32(&devecPhiQ_T_, 3, 4, devecPhiQ_T);
		Quaternion_mat_devecPhiT(q, devecPhiQ_T);
		float devecPhiDQ_ref_T[3*4]; arm_matrix_instance_f32 devecPhiDQ_ref_T_; arm_mat_init_f32(&devecPhiDQ_ref_T_, 3, 4, devecPhiDQ_ref_T);
		Quaternion_mat_devecPhiT(dq_ref, devecPhiDQ_ref_T);

		float sum[3]; arm_matrix_instance_f32 sum_; arm_mat_init_f32(&sum_, 3, 1, sum);
		float tmp3[3]; arm_matrix_instance_f32 tmp3_; arm_mat_init_f32(&tmp3_, 3, 1, tmp3);
		arm_mat_mult_f32(&devecPhiQ_T_, &fq_ss_, &tmp3_); // devec*Phi(q)'*fq_ss
		arm_scale_f32(tmp3, 2.f, sum, 3); // sum = 2*devec*Phi(q)'*fq_ss

		arm_mat_mult_f32(&devecPhiDQ_ref_T_, &dq_ref_, &tmp3_); // devec*Phi(dq_ref)'*dq_ref
		arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Phi(dq_ref)'*dq_ref
		arm_add_f32(tmp3, sum, sum, 3); // sum += 2*devec*Phi(dq_ref)'*dq_ref

		arm_negate_f32(sum, sum, 3); // negate the sum to get minus in front of all parts

		/* inv(2*devec*Phi(q)'*gq_ss) */
		float Input[3*3]; arm_matrix_instance_f32 Input_; arm_mat_init_f32(&Input_, 3, 3, Input);
		float InputInv[3*3]; arm_matrix_instance_f32 InputInv_; arm_mat_init_f32(&InputInv_, 3, 3, InputInv);
		arm_mat_mult_f32(&devecPhiQ_T_, &gq_ss_, &Input_);
		arm_scale_f32(Input, 2.f, Input, 3*3);
		inv3x3(Input, InputInv);

		/* Final computation: tau_ss = inv(2*devec*Phi(q)'*gq_ss) * (-2*devec*Phi(dq_ref)'*dq_ref - 2*devec*Phi(q)'*fq_ss) */
		arm_matrix_instance_f32 tau_ss_; arm_mat_init_f32(&tau_ss_, 3, 1, tau_ss);
		arm_mat_mult_f32(&InputInv_, &sum_, &tau_ss_); // tau_ss = inv(2*devec*Phi(q)'*gq_ss) * (-2*devec*Phi(dq_ref)'*dq_ref - 2*devec*Phi(q)'*fq_ss)
	}

	/* Compute control torque by matrix multiplication with LQR gain */
	arm_matrix_instance_f32 LQR_K_; arm_mat_init_f32(&LQR_K_, 3, 6, (float32_t *)gainMatrix);
	float tau_control[3]; arm_matrix_instance_f32 tau_control_; arm_mat_init_f32(&tau_control_, 3, 1, (float32_t *)tau_control);
	arm_mat_mult_f32(&LQR_K_, &X_err_, &tau_control_); // tau = -K * X_err_reduced
	arm_negate_f32(tau_control, tau_control, 3);

	/* Add linearized torque and computed torque and set output */
	arm_add_f32(tau_ss, tau_control, tau, 3); // tau = tau_ss + tau_control
}

bool LQR::UnitTest(void)
{
	// Perform a unit test to verify if it passes
	const float UnitTestGain[3*6] = {22.0217540086999,	1.1248573671829e-14,	-0.182574185835055,	2.71774528131163,	1.44298962102302e-15,	-0.101363312648735,
									-11.01087700435,	19.0713984074259,	-0.182574185835056,	-1.35887264065581,	2.35481794807652,	-0.101363312648735,
									-11.01087700435,	-19.0713984074259,	-0.182574185835055,	-1.35887264065582,	-2.35481794807652,	-0.101363312648735};

	float q[4];
	Quaternion_eul2quat_zyx(deg2rad(10), deg2rad(-2), deg2rad(5), q);
	float omega_b_vec[4] = {0, 0.5, 0.2, 0.1};

	float dq[4];
	Quaternion_Phi(q, omega_b_vec, dq); // Phi(q)*[0;omega_b]
	arm_scale_f32(dq, 1.0f/2.0f, dq, 4); // 1/2 * Phi(q)*[0;omega_b]

	float xy[2] = { 0, 0 };
	float dxy[2] = { 0, 0 };
	float COM[3] = { 0, 0, _params.model.l };

	float q_ref[4];
	Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(10), q_ref);

	float omega_b_ref[3] = {0, 0, 0};

	float Torque[3];
	Step(q, dq, xy, dxy, COM, q_ref, omega_b_ref, (float *)UnitTestGain, false, Torque);

	float Torque_Expected[3] = {-0.4092, -0.1150, 0.6033};
	if (Math_Round(Torque[0], 4) == Math_Round(Torque_Expected[0], 4) &&
		Math_Round(Torque[1], 4) == Math_Round(Torque_Expected[1], 4) &&
		Math_Round(Torque[2], 4) == Math_Round(Torque_Expected[2], 4))
		return true;
	else
		return false;
}
