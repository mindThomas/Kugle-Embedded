/* Copyright (C) 2020 Victor Borja. All rights reserved.
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
 * Victor Borja
 * e-mail   :  v.borja1991@gmail.com
 * ------------------------------------------
 */
 
#include "FeedbackLinearization.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

#include "Quaternion.h"
#include "MathLib.h"
#include "Matrix.h"
#include "Parameters.h"
#include "Debug.h"

#include "mass.h"
#include "coriolis.h"
#include "gravity.h"
#include "friction.h"
#include "input_forces.h"


FeedbackLinearization::FeedbackLinearization(Parameters& params) : _params(params)
{
	// TODO adjust
	_frameType = 0; //"Q_DOT_BODY_MANIFOLD" -> "BODY_FRAME"
}

FeedbackLinearization::~FeedbackLinearization()
{
}

/**
 * @brief 	Compute control output with Feedback Linearization controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]          Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]         Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]        Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in inertial frame
 * @param	tau[3]        Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void FeedbackLinearization::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	Step(q, dq, xy, dxy, q_ref, omega_ref, _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g, _params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z, _params.controller.Kf, tau);
}

/**
 * @brief 	Compute control output with Feedback Linearization controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]          Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]         Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]        Input: current ball (center) velocity defined in inertial frame
 * @param	COM[3]        Input: current estimate of COM
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in inertial frame
 * @param	tau[3]        Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 * @param	S[3]          Output: sliding manifold values for the three surfaces used for the attitude control
 */
void FeedbackLinearization::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float COM[3], const float q_ref[4], const float omega_ref[3], float tau[3])
{
	Step(q, dq, xy, dxy, q_ref, omega_ref, _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g, COM[0], COM[1], COM[2], _params.controller.Kf, tau);
}

/**
 * @brief 	Compute control output with Feedback Linearization controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]          Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]         Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]        Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in either inertial frame or body frame, depending on the BodyFrame flag
 * @param	tau[3]        Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 */
void FeedbackLinearization::Step(const float q[4], const float dq_in[4], const float xy[2], const float dxy[2], const float q_ref_in[4], const float omega_ref_body[3], const float Jk, const float Mk, const float rk, const float Mb, const float Jbx, const float Jby, const float Jbz, const float Jw, const float rw, const float Bvk, const float Bvm, const float Bvb, const float l, const float g_const, const float COM_X, const float COM_Y, const float COM_Z, const float Kf[6], float tau[3])
{
	// See ARM-CMSIS DSP library for matrix operations: https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
	arm_matrix_instance_f32 q_; arm_mat_init_f32(&q_, 4, 1, (float32_t *)q);
	float dq[4] = {dq_in[0], dq_in[1], dq_in[2], dq_in[3]}; arm_matrix_instance_f32 dq_; arm_mat_init_f32(&dq_, 4, 1, (float32_t *)dq);
	float chi[6] = {xy[0], xy[1], q[0], q[1], q[2], q[3]}; arm_matrix_instance_f32 chi_; arm_mat_init_f32(&chi_, 6, 1, (float32_t *)chi);
	float dchi[6] = {dxy[0], dxy[1], dq[0], dq[1], dq[2], dq[3]}; arm_matrix_instance_f32 dchi_; arm_mat_init_f32(&dchi_, 6, 1, (float32_t *)dchi);

	float M[6*6]; arm_matrix_instance_f32 M_; arm_mat_init_f32(&M_, 6, 6, M);
	float C[6*6]; arm_matrix_instance_f32 C_; arm_mat_init_f32(&C_, 6, 6, C);
	float G[6]; arm_matrix_instance_f32 G_; arm_mat_init_f32(&G_, 6, 1, G);
	float D[6]; arm_matrix_instance_f32 D_; arm_mat_init_f32(&D_, 6, 1, D);
	float Q[6*3]; arm_matrix_instance_f32 Q_; arm_mat_init_f32(&Q_, 6, 3, Q);

	mass(COM_X, COM_Y, COM_Z, Jbx, Jby, Jbz, Jk, Jw, Mb, Mk, q[0], q[1], q[2], q[3], rk, rw, M);
	coriolis(COM_X, COM_Y, COM_Z, Jbx, Jby, Jbz, Jw, Mb, 0.0f, dq[0], dq[1], dq[2], dq[3], dxy[0], dxy[1], q[0], q[1], q[2], q[3], rk, rw, C); // beta = 0
	gravity(COM_X, COM_Y, COM_Z, Mb, 0.0f, g_const, q[0], q[1], q[2], q[3], G); // beta = 0
	friction(Bvb, Bvk, Bvm, 0.0f, dq[0], dq[1], dq[2], dq[3], dxy[0], dxy[1], q[0], q[1], q[2], q[3], rk, rw, D);
	input_forces(q[0], q[1], q[2], q[3], rk, rw, Q);

 	float Minv[6*6]; arm_matrix_instance_f32 Minv_; arm_mat_init_f32(&Minv_, 6, 6, Minv);
	inv6x6(M, Minv);

	/* Computation of f and g canonical form parts */
	/* f = Minv * (-C*dchi - G - D) */
	/* g = Minv * Q */
	float f[6]; arm_matrix_instance_f32 f_; arm_mat_init_f32(&f_, 6, 1, f);
	float g[6*3]; arm_matrix_instance_f32 g_; arm_mat_init_f32(&g_, 6, 3, g);
	float tmp6[6]; arm_matrix_instance_f32 tmp6_; arm_mat_init_f32(&tmp6_, 6, 1, tmp6);
	arm_mat_mult_f32(&C_, &dchi_, &tmp6_);
	arm_add_f32(tmp6, G, tmp6, 6);
	arm_add_f32(tmp6, D, tmp6, 6);
	arm_negate_f32(tmp6, tmp6, 6);
	arm_mat_mult_f32(&Minv_, &tmp6_, &f_);
	arm_mat_mult_f32(&Minv_, &Q_, &g_);

	/* Extract of fq and gq - Focus for the quaternion part of the functions */
	float fq[4]; arm_matrix_instance_f32 fq_; arm_mat_init_f32(&fq_, 4, 1, fq);
	Matrix_Extract(f, 6,1, 2, 0, 4, 1, fq);
	float gq[4*3]; arm_matrix_instance_f32 gq_; arm_mat_init_f32(&gq_, 4, 3, gq);
	Matrix_Extract(g, 6,3,  2,0,  4,3,  gq);

	/* Compute quaternion error in required frame */
	float q_err[4];
	if (_frameType == 0) {
		/* Quaternion error in Body frame */
		// q_err = Phi(q_ref)' * q
		Quaternion_PhiT(q_ref_in, q, q_err);
	} else {
		/* Quaternion error in Inertial frame */
		// q_err = Gamma(q_ref)' * q
		Quaternion_GammaT(q_ref_in, q, q_err);
	}

	/* Invert quaternion if the scalar is negative to get the shortest path */
	if (q_err[0] < 0)
	  arm_negate_f32(q_err, q_err, 4);

	/* Recompute q_ref based on q_err  (makes a difference if q_err was negated) */
	float q_ref[4];
	if (_frameType == 0 ) {
		/* q_err = q_ref* o q
	   	   q_ref o q_err = q
	   	   q_ref = q o q_err*
	   	   q_ref = Gamma(q_err)' * q
		 */
		Quaternion_GammaT(q_err, q, q_ref);
	} else {
		/* q_err = q o q_ref*
	   	   q_err o q_ref = q
	   	   q_ref = q_err* o q
	   	   q_ref = Phi(q_err)' * q
		 */
		Quaternion_PhiT(q_err, q, q_ref);
	}

	/* Compute omega_ref in required frame */
	float omega_ref[3];
	if (_frameType == 1) {
		// Convert omega_ref_body (input) into omega_ref_inertial to be used by Feedback Linearization controller
		Quaternion_RotateVector_Body2Inertial(q_ref, omega_ref_body, omega_ref);
	} else {
		// Take omega_ref_body (input) directly, since the Feedback Linearization controller will use angular velocity reference in body frame
		omega_ref[0] = omega_ref_body[0];
		omega_ref[1] = omega_ref_body[1];
		omega_ref[2] = omega_ref_body[2];
	}

	/* Compute quaternion derivative reference */
	float dq_ref[4];
	float omega_ref_q[4] = {0, omega_ref[0], omega_ref[1], omega_ref[2]};
	if (_frameType == 0) {
		/* Body angular velocity */
		/* dq_ref = 1/2 * Phi(q_ref) * [0;omega_ref]; */
		Quaternion_Phi(q_ref, omega_ref_q, dq_ref); // Phi(q_ref) * [0;omega_ref]
		arm_scale_f32(dq_ref, 0.5f, dq_ref, 4);
	} else {
		/* Inertial angular velocity */
		/* dq_ref = 1/2 * Gamma(q_ref) * [0;omega_ref]; */
		Quaternion_Gamma(q_ref, omega_ref_q, dq_ref); // Gamma(q_ref) * [0;omega_ref]
		arm_scale_f32(dq_ref, 0.5f, dq_ref, 4);
	}

	/* Compute quaternion derivate error */
	float dq_err[4];
	float dq_err1[4];
	float dq_err2[4];
	if (_frameType == 0) {
		/* Quaternion error in Body frame */
		// dq_err = Phi(dq_ref)'*q + Phi(q_ref)'*dq
		Quaternion_PhiT(dq_ref, q, q_err1);
		Quaternion_PhiT(q_ref,dq,q_err2);
		for (int i=0; i<4;i++)
			dq_err[i]= dq_err1[i]+dq_err2[i];
	} else {
		/* Quaternion error in Inertial frame */
		// dq_err = Gamma(dq_ref)'*q + Gamma(q_ref)'*dq
		Quaternion_GammaT(dq_ref, q, q_err1);
		Quaternion_GammaT(q_ref, dq, q_err2);
		for (int i=0; i<4;i++)
			dq_err[i]= dq_err1[i]+dq_err2[i];
	}


	/* Prepare E and D matrices to get motors inputs.
	 * u = E\(v-D)
	 * E = [gq(2,1), gq(2,1), gq(2,1);
	 * 		gq(2,1), gq(2,1), gq(2,1);
	 * 		gq(2,1), gq(2,1), gq(2,1)];
	 * D = [fq(2), fq(3), fq(4)];
	 * v = [-k(1)*q2-k(4)*dq2;
     *   	-k(2)*q3-k(5)*dq3;
     *  	-k(3)*q4-k(6)*dq4]; Where K represent the gains of the controller -> kf
	 */

	float E[3*3];
	arm_matrix_instance_f32 E_;
	arm_mat_init_f32(&E_, 3, 3, E);
	Matrix_Extract(gq, 4,3, 1,0, 3,3, E);

	float Einv[3*3]; arm_matrix_instance_f32 Einv_; arm_mat_init_f32(&Einv_, 3, 3, Einv);arm_mat_inverse_f32(&E_, &Einv_);

	float Df[3]; arm_matrix_instance_f32 Df_; arm_mat_init_f32(&Df_, 3, 1, Df);
	Matrix_Extract(fq, 4,1, 1,0, 3,1, Df);

	// TODO try to improve performance of this
	float v[3] = {-Kf[0]*q_ref[1]-Kf[3]*dq_ref[1],-Kf[1]*q_ref[2]-Kf[4]*dq_ref[2],-Kf[2]*q_ref[3]-Kf[5]*dq_ref[2]};arm_matrix_instance_f32 v_; arm_mat_init_f32(&v_, 3, 1, v);

	float u[3]; arm_matrix_instance_f32 u_; arm_mat_init_f32(&u_, 3, 1, u);

	float auxFeed1[3]; arm_matrix_instance_f32 auxFeed1_; arm_mat_init_f32(&auxFeed1_, 3, 1, auxFeed1);

	// u = E\(v-D)
	arm_mat_sub_f32(&v_, &Df_, &auxFeed1_);
	arm_mat_mult_f32(&Einv_, &auxFeed1_, &u_);

	tau[0] = u[0];
	tau[1] = u[1];
	tau[2] = u[2];
}

void FeedbackLinearization::Saturation(const float * in, const int size, float * out)
{
	for (int i = 0; i < size; i++) {
		out[i] = fminf(fmaxf(out[i], -1), 1);
	}
}

void FeedbackLinearization::Sign(float * in, int size, float * out)
{
	for (int i = 0; i < size; i++)
	  out[i] = copysignf(1.0f, in[i]);
}

bool FeedbackLinearization::UnitTest(void)
{
	const float l = 0.35f;

	const float COM_X = 0.0f;
	const float COM_Y = 0.0f;
	const float COM_Z = l;

	const float g = 9.82f;

	const float rk = 0.129f;
	const float Mk = 1.46f;
	const float Jk = ((2.f * Mk * rk*rk) / 3.f);

	const float rw = 0.05f;
	const float Mw = 0.270f;
	const float i_gear = 4.3;
	const float Jow = 9.f * 0.0001f;
	const float Jm = 1.21f * 0.0001f;
	const float Jw = (Jow + i_gear*i_gear*Jm);

	const float Mb = (8.205f + 5.856f);

	const float Jbx = (0.958f + Mb * l*l);
	const float Jby = (0.961f + Mb * l*l);
	const float Jbz = 0.31f;

	const float Bvk = 0*0.001f;
	const float Bvm = 0*0.001f;
	const float Bvb = 0*0.001f;

	const float Kf[6] = {100, 100, 100, 50, 50, 50};

	float q[4];
	Quaternion_eul2quat_zyx(deg2rad(10), deg2rad(-2), deg2rad(5), q);
	float omega_b_vec[4] = {0, 0.15, 0.2, 0.1};

	float dq[4];
	Quaternion_Phi(q, omega_b_vec, dq); // Phi(q)*[0;omega_b]
	arm_scale_f32(dq, 1.0f/2.0f, dq, 4); // 1/2 * Phi(q)*[0;omega_b]

	float xy[2] = {0, 0};
	float dxy[2] = {0.5, -0.2};

	float q_ref[4];
	Quaternion_eul2quat_zyx(deg2rad(0), deg2rad(0), deg2rad(10), q_ref);

	float omega_i_ref[3] = {0, 0, 0};

	float Torque[3];
	Step(q, dq, xy, dxy, q_ref, omega_i_ref, Jk, Mk, rk, Mb, Jbx, Jby, Jbz, Jw, rw, Bvk, Bvm, Bvb, l, g, COM_X, COM_Y, COM_Z, Kf, Torque);

	float Torque_Expected[3] = {-1.0488, -0.2760, 2.0310};
	if (Math_Round(Torque[0], 4) == Math_Round(Torque_Expected[0], 4) &&
		Math_Round(Torque[1], 4) == Math_Round(Torque_Expected[1], 4) &&
		Math_Round(Torque[2], 4) == Math_Round(Torque_Expected[2], 4))
		return true;
	else
		return false;
}
