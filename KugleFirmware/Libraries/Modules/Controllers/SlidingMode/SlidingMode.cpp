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
 
#include "SlidingMode.h"
 
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


SlidingMode::SlidingMode(Parameters& params) : _params(params)
{
}

SlidingMode::~SlidingMode()
{
}

/**
 * @brief 	Compute control output with Sliding mode controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in inertial frame
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 * @param	S[3]      	  Output: sliding manifold values for the three surfaces used for the attitude control
 */
void SlidingMode::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], float tau[3], float S[3])
{
	Step(q, dq, xy, dxy, q_ref, omega_ref, _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g, _params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z, _params.controller.K, _params.controller.eta, _params.controller.epsilon, _params.controller.ContinousSwitching, _params.controller.EquivalentControl, _params.controller.DisableQdotInEquivalentControl, _params.controller.ManifoldType, tau, S);
}

/**
 * @brief 	Compute control output with Sliding mode controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	COM[3]    	  Input: current estimate of COM
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in inertial frame
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 * @param	S[3]      	  Output: sliding manifold values for the three surfaces used for the attitude control
 */
void SlidingMode::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float COM[3], const float q_ref[4], const float omega_ref[3], float tau[3], float S[3])
{
	Step(q, dq, xy, dxy, q_ref, omega_ref, _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g, COM[0], COM[1], COM[2], _params.controller.K, _params.controller.eta, _params.controller.epsilon, _params.controller.ContinousSwitching, _params.controller.EquivalentControl, _params.controller.DisableQdotInEquivalentControl, _params.controller.ManifoldType, tau, S);
}

/**
 * @brief 	Compute control output with Sliding mode controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in either inertial frame or body frame, depending on the BodyFrame flag
 * @param   model params       Input: Different mixed model constants
 * @param   controller params  Input: Different tunable Sliding mode controller parameters
 * @param   manifoldType  Input: defines which specific manifold type/definition to use, including the choice of quaternion error and angular velocity reference frame
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 * @param	S[3]      	  Output: sliding manifold values for the three surfaces used for the attitude control
 */
void SlidingMode::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref_in[4], const float omega_ref_body[3], const float Jk, const float Mk, const float rk, const float Mb, const float Jbx, const float Jby, const float Jbz, const float Jw, const float rw, const float Bvk, const float Bvm, const float Bvb, const float l, const float g_const, const float COM_X, const float COM_Y, const float COM_Z, const float K[3], const float eta[3], const float epsilon[3], const bool continuousSwitching, const bool IncludeEquivalentControl, const bool DisableQdotInEquivalentControl, const lspc::ParameterTypes::slidingManifoldType_t manifoldType, float tau[3], float S[3])
{
    // See ARM-CMSIS DSP library for matrix operations: https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
    arm_matrix_instance_f32 q_; arm_mat_init_f32(&q_, 4, 1, (float32_t *)q);
    arm_matrix_instance_f32 dq_; arm_mat_init_f32(&dq_, 4, 1, (float32_t *)dq);
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

    arm_matrix_instance_f32 fq_; arm_mat_init_f32(&fq_, 4, 1, &f[2]);
    float gq[4*3]; arm_matrix_instance_f32 gq_; arm_mat_init_f32(&gq_, 4, 3, gq);
    Matrix_Extract(g, 6,3,  2,0,  4,3,  gq);

    /* Compute omega_ref in required frame */
    float omega_ref[3];
    if (manifoldType == lspc::ParameterTypes::Q_DOT_INERTIAL_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_INERTIAL_MANIFOLD) {
    	// Convert omega_ref_body (input) into omega_ref_inertial to be used by sliding mode controller
    	Quaternion_RotateVector_Body2Inertial(q, omega_ref_body, omega_ref);
    } else {
    	// Take omega_ref_body (input) directly, since the sliding mode controller will use angular velocity reference in body frame
    	omega_ref[0] = omega_ref_body[0];
    	omega_ref[1] = omega_ref_body[1];
    	omega_ref[2] = omega_ref_body[2];
    }

    /* Compute quaternion error in required frame */
    float q_err[4];
    if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
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
	if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
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

	/* Compute quaternion derivative reference */
    float dq_ref[4];
    float omega_ref_q[4] = {0, omega_ref[0], omega_ref[1], omega_ref[2]};
    if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
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

    /* Prepare matrices used in computations */
    float devecPhiQ_T[3*4]; arm_matrix_instance_f32 devecPhiQ_T_; arm_mat_init_f32(&devecPhiQ_T_, 3, 4, devecPhiQ_T);
    Quaternion_mat_devecPhiT(q, devecPhiQ_T);

	float devecPhiQref_T[3*4]; arm_matrix_instance_f32 devecPhiQref_T_; arm_mat_init_f32(&devecPhiQref_T_, 3, 4, devecPhiQref_T);
	Quaternion_mat_devecPhiT(q_ref, devecPhiQref_T); // devec*Phi(q_ref)'

	float devecPhiDQ_T[3*4]; arm_matrix_instance_f32 devecPhiDQ_T_; arm_mat_init_f32(&devecPhiDQ_T_, 3, 4, devecPhiDQ_T);
	Quaternion_mat_devecPhiT(dq, devecPhiDQ_T); // devec*Phi(dq)'

	float devecPhiDQref_T[3*4]; arm_matrix_instance_f32 devecPhiDQref_T_; arm_mat_init_f32(&devecPhiDQref_T_, 3, 4, devecPhiDQref_T);
	Quaternion_mat_devecPhiT(dq_ref, devecPhiDQref_T); // devec*Phi(dq_ref)'

    float devecGammaQ_T[3*4]; arm_matrix_instance_f32 devecGammaQ_T_; arm_mat_init_f32(&devecGammaQ_T_, 3, 4, devecGammaQ_T);
    Quaternion_mat_devecGammaT(q, devecGammaQ_T);

	float devecGammaQref_T[3*4]; arm_matrix_instance_f32 devecGammaQref_T_; arm_mat_init_f32(&devecGammaQref_T_, 3, 4, devecGammaQref_T);
	Quaternion_mat_devecGammaT(q_ref, devecGammaQref_T); // devec*Gamma(q_ref)'

	float devecGammaDQ_T[3*4]; arm_matrix_instance_f32 devecGammaDQ_T_; arm_mat_init_f32(&devecGammaDQ_T_, 3, 4, devecGammaDQ_T);
	Quaternion_mat_devecGammaT(dq, devecGammaDQ_T); // devec*Gamma(dq)'

	float devecGammaDQref_T[3*4]; arm_matrix_instance_f32 devecGammaDQref_T_; arm_mat_init_f32(&devecGammaDQref_T_, 3, 4, devecGammaDQref_T);
	Quaternion_mat_devecGammaT(dq_ref, devecGammaDQref_T); // devec*Gamma(dq_ref)'


	/* Compute InputInv matrix that maps from switching law to control output */
	float Input[3*3]; arm_matrix_instance_f32 Input_; arm_mat_init_f32(&Input_, 3, 3, Input);
	float InputInv[3*3]; arm_matrix_instance_f32 InputInv_; arm_mat_init_f32(&InputInv_, 3, 3, InputInv);

    if (manifoldType == lspc::ParameterTypes::Q_DOT_INERTIAL_MANIFOLD) {
        /* Inertial angular velocity */
        /* InputInv = inv(devec*Gamma(q_ref)' * gq) */
        arm_mat_mult_f32(&devecGammaQref_T_, &gq_, &Input_);
        arm_scale_f32(Input, 2.f, Input, 3*3);
    }
    if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD) {
        /* Body angular velocity */
        /* InputInv = inv(devec*Phi(q_ref)' * gq) */
        arm_mat_mult_f32(&devecPhiQref_T_, &gq_, &Input_);
        arm_scale_f32(Input, 2.f, Input, 3*3);
    }
    else if (manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
        /* Body angular velocity */
        /* InputInv = inv(2 * devec*Phi(q)' * gq) */
        Quaternion_mat_devecPhiT(q, devecPhiQ_T);
        arm_mat_mult_f32(&devecPhiQ_T_, &gq_, &Input_);
        arm_scale_f32(Input, 2.f, Input, 3*3);
    }
    else if (manifoldType == lspc::ParameterTypes::OMEGA_INERTIAL_MANIFOLD) {
        /* Inertial angular velocity */
        /* InputInv = inv(2 * devec*Gamma(q)' * gq) */
        Quaternion_mat_devecGammaT(q, devecGammaQ_T);
        arm_mat_mult_f32(&devecGammaQ_T_, &gq_, &Input_);
        arm_scale_f32(Input, 2.f, Input, 3*3);
    }
    inv3x3(Input, InputInv);


    /* Compute equivalent control */
    float sum3[3]; arm_matrix_instance_f32 sum3_; arm_mat_init_f32(&sum3_, 3, 1, sum3);
    float tmp3[3]; arm_matrix_instance_f32 tmp3_; arm_mat_init_f32(&tmp3_, 3, 1, tmp3);
    float tmp4[4]; arm_matrix_instance_f32 tmp4_; arm_mat_init_f32(&tmp4_, 4, 1, tmp4);

    if (manifoldType == lspc::ParameterTypes::Q_DOT_INERTIAL_MANIFOLD) {
    	/* tau_eq = InputInv * ( -devec*Gamma(q_ref)'*fq - 2*devec*Gamma(dq_ref)'*dq - K*devec*Gamma(dq_ref)'*q - K*devec*Gamma(q_ref)'*dq ); */
    	/* Here ddq_ref has been left out since we assume omega_ref to be slowly varying */
		arm_mat_mult_f32(&devecGammaQref_T_, &fq_, &sum3_); // sum = devec*Gamma(q_ref)'*fq

		arm_mat_mult_f32(&devecGammaDQref_T_, &q_, &tmp3_); // devec*Gamma(dq_ref)'*q
		arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Gamma(q_ref)'*dq
		arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Gamma(q_ref)'*dq

		if (!DisableQdotInEquivalentControl) {
			arm_mat_mult_f32(&devecGammaQref_T_, &dq_, &tmp3_); // devec*Gamma(q_ref)'*dq
			arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Gamma(q_ref)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Gamma(q_ref)'*dq

			arm_mat_mult_f32(&devecGammaDQref_T_, &dq_, &tmp3_); // devec*Gamma(dq_ref)'*dq
			arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Gamma(dq_ref)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += 2*devec*Gamma(dq_ref)'*dq
		}

		arm_negate_f32(sum3, sum3, 3); // negate the sum to get minus in front of all parts
    }
    if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD) {
    	/* tau_eq = InputInv * ( -devec*Phi(q_ref)'*fq - 2*devec*Phi(dq_ref)'*dq - K*devec*Phi(dq_ref)'*q - K*devec*Phi(q_ref)'*dq ); */
    	/* Here ddq_ref has been left out since we assume omega_ref to be slowly varying */
    	arm_mat_mult_f32(&devecPhiQref_T_, &fq_, &sum3_); // devec*Phi(q_ref)'*fq

		arm_mat_mult_f32(&devecPhiDQref_T_, &q_, &tmp3_); // devec*Phi(dq_ref)'*q
		arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Phi(q_ref)'*dq
		arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Phi(q_ref)'*dq

		if (!DisableQdotInEquivalentControl) {
			arm_mat_mult_f32(&devecPhiQref_T_, &dq_, &tmp3_); // devec*Phi(q_ref)'*dq
			arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Phi(q_ref)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Phi(q_ref)'*dq

			arm_mat_mult_f32(&devecPhiDQref_T_, &dq_, &tmp3_); // devec*Phi(dq_ref)'*dq
			arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Phi(dq_ref)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += 2*devec*Phi(dq_ref)'*dq
		}

		arm_negate_f32(sum3, sum3, 3); // negate the sum to get minus in front of all parts
    }
    else if (manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
    	/* tau_eq = InputInv * (-2*devec*Phi(dq)'*dq - 2*devec*Phi(q)'*fq + domega_ref - K*devec*Phi(dq_ref)'*q - K*devec*Phi(q_ref)'*dq); */
    	/* But we assume domega_ref = 0    (omega_ref is slowly varying) */
		arm_mat_mult_f32(&devecPhiQ_T_, &fq_, &sum3_); // devec*Phi(q)'*fq
		arm_scale_f32(sum3, 2.f, sum3, 3); // sum = 2*devec*Phi(q)'*fq

		arm_mat_mult_f32(&devecPhiDQref_T_, &q_, &tmp3_); // devec*Phi(dq_ref)'*q
		arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Phi(q_ref)'*dq
		arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Phi(q_ref)'*dq

		if (!DisableQdotInEquivalentControl) {
			arm_mat_mult_f32(&devecPhiQref_T_, &dq_, &tmp3_); // devec*Phi(q_ref)'*dq
			arm_mult_f32((float*)K, tmp3, sum3, 3); // K*devec*Phi(q_ref)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Phi(q_ref)'*dq

			arm_mat_mult_f32(&devecPhiDQ_T_, &dq_, &tmp3_); // devec*Phi(dq)'*dq
			arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Phi(dq)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += 2*devec*Phi(dq)'*dq
		}

		arm_negate_f32(sum3, sum3, 3); // negate the sum to get minus in front of all parts
    }
    else if (manifoldType == lspc::ParameterTypes::OMEGA_INERTIAL_MANIFOLD) {
    	/* tau_eq = InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq + domega_ref - K*devec*Gamma(dq_ref)'*q - K*devec*Gamma(q_ref)'*dq); */
    	/* But we assume domega_ref = 0    (omega_ref is slowly varying) */
		arm_mat_mult_f32(&devecGammaQ_T_, &fq_, &sum3_); // devec*Gamma(q)'*fq
		arm_scale_f32(sum3, 2.f, sum3, 3); // sum = 2*devec*Gamma(q)'*fq

		arm_mat_mult_f32(&devecGammaDQref_T_, &q_, &tmp3_); // devec*Gamma(dq_ref)'*q
		arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Gamma(q_ref)'*dq
		arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Gamma(q_ref)'*dq

		if (!DisableQdotInEquivalentControl) {
			arm_mat_mult_f32(&devecGammaQref_T_, &dq_, &tmp3_); // devec*Gamma(q_ref)'*dq
			arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Gamma(q_ref)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += K*devec*Gamma(q_ref)'*dq

			arm_mat_mult_f32(&devecGammaDQ_T_, &dq_, &tmp3_); // devec*Gamma(dq)'*dq
			arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Gamma(dq)'*dq
			arm_add_f32(tmp3, sum3, sum3, 3); // sum += 2*devec*Gamma(dq)'*dq
		}

		arm_negate_f32(sum3, sum3, 3); // negate the sum to get minus in front of all parts
    }

    /* Combine computed parts into equivalent control */
    float tau_eq[3]; arm_matrix_instance_f32 tau_eq_; arm_mat_init_f32(&tau_eq_, 3, 1, tau_eq);
    arm_mat_mult_f32(&InputInv_, &sum3_, &tau_eq_);


    /* Compute angular velocity */
    float omega[3]; arm_matrix_instance_f32 omega_; arm_mat_init_f32(&omega_, 3, 1, omega);

    if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
    	/* Body angular velocity */
    	/* omega = 2*devec*Phi(q)'*dq; */
    	arm_mat_mult_f32(&devecPhiQ_T_, &dq_, &omega_); // devec*Phi(q)'*dq
    	arm_scale_f32(omega, 2.f, omega, 3); // 2*devec*Phi(q)'*dq
    } else {
    	/* Inertial angular velocity */
    	/* omega = 2*devec*Gamma(q)'*dq; */
    	arm_mat_mult_f32(&devecGammaQ_T_, &dq_, &omega_); // devec*Gamma(q)'*dq
    	arm_scale_f32(omega, 2.f, omega, 3); // 2*devec*Gamma(q)'*dq
    }


    /* Compute quaternion derivative error */
    float devec_dq_err[3]; arm_matrix_instance_f32 devec_dq_err_; arm_mat_init_f32(&devec_dq_err_, 3, 1, devec_dq_err);

    if (manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) {
    	/* Error defined in body frame */
    	/* devec*dq_err = devec*Phi(dq_ref)'*q + devec*Phi(q_ref)'*dq */
    	arm_mat_mult_f32(&devecPhiDQref_T_, &q_, &devec_dq_err_); // devec*Phi(dq_ref)'*q
    	arm_mat_mult_f32(&devecPhiQref_T_, &dq_, &tmp3_); // devec*Phi(q_ref)'*dq
    	arm_add_f32(tmp3, devec_dq_err, devec_dq_err, 3); // devec*Phi(dq_ref)'*q + devec*Phi(q_ref)'*dq
    } else {
    	/* Error defined in inertial frame */
    	/* devec*dq_err = devec*Gamma(dq_ref)'*q + devec*Gamma(q_ref)'*dq */
    	arm_mat_mult_f32(&devecGammaDQref_T_, &q_, &devec_dq_err_); // devec*Gamma(dq_ref)'*q
    	arm_mat_mult_f32(&devecGammaQref_T_, &dq_, &tmp3_); // devec*Gamma(q_ref)'*dq
    	arm_add_f32(tmp3, devec_dq_err, devec_dq_err, 3); // devec*Gamma(dq_ref)'*q + devec*Gamma(q_ref)'*dq
    }

    /* Compute sliding variable */
    if (manifoldType == lspc::ParameterTypes::Q_DOT_INERTIAL_MANIFOLD || manifoldType == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD) {
    	/* S = devec*dq_err + K*devec*q_err */
    	arm_mult_f32(devec(q_err), (float*)K, S, 3); // S = K*devec*q_err
    	arm_add_f32(devec_dq_err, S, S, 3); // S += devec*dq_err  -->  S = devec*dq_err + K*devec*q_err
	}
	else if (manifoldType == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD || manifoldType == lspc::ParameterTypes::OMEGA_INERTIAL_MANIFOLD) {
	    /* S = omega - omega_ref + K*devec*q_err */
	    arm_mult_f32(devec(q_err), (float*)K, S, 3); // S = K*devec*q_err
	    arm_add_f32(omega, S, S, 3); // S += omega  -->  S = omega + K*devec*q_err
	    arm_sub_f32(S, (float*)omega_ref, S, 3); // S -= omega_ref  -->  S = omega - omega_ref + K*devec*q_err
	}

    /* Compute switching law */
    float u[3]; arm_matrix_instance_f32 u_; arm_mat_init_f32(&u_, 3, 1, u);
    if (continuousSwitching) { // continous switching law
      /* satS = sat(S/epsilon); */
      float satS[3];
      Saturation(S, 3, epsilon, satS);

      arm_mult_f32(satS, (float*)eta, u, 3); // eta * satS
      arm_negate_f32(u, u, 3); // negate to get  u = -eta * satS;
    } else { // discontinous switching law
      /* sgnS = sign(S); */
      float sgnS[3];
      Sign(S, 3, sgnS);

      arm_mult_f32(sgnS, (float*)eta, u, 3); // eta * satS
      arm_negate_f32(u, u, 3); // negate to get  u = -eta * satS;
    }

    /* tau_switching = InputInv * u; */
    float tau_switching[3]; arm_matrix_instance_f32 tau_switching_; arm_mat_init_f32(&tau_switching_, 3, 1, tau_switching);
    arm_mat_mult_f32(&InputInv_, &u_, &tau_switching_);

    /* Set output torque */
    if (IncludeEquivalentControl) {
    	/* tau = tau_eq + tau_switching; */
    	arm_add_f32(tau_eq, tau_switching, tau, 3);
    } else {
    	/* tau = tau_switching */
    	tau[0] = tau_switching[0];
    	tau[1] = tau_switching[1];
    	tau[2] = tau_switching[2];
    }
}

void SlidingMode::Saturation(const float * in, const int size, const float * epsilon, float * out)
{
	//arm_scale_f32(in, 1.f/epsilon, out, size);
    for (int i = 0; i < size; i++) {
    	out[i] = in[i] / epsilon[i];
    	out[i] = fmin(fmax(out[i], -1), 1);
    }
}

void SlidingMode::Sign(float * in, int size, float * out)
{
    for (int i = 0; i < size; i++)
      out[i] = copysignf(1.0f, in[i]);
}

bool SlidingMode::UnitTest(void)
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

	const float K[3] = {50, 50, 30};
	const float eta[3] = {2.5, 2.5, 2.5};
	const float epsilon[3] = {2.5, 2.5, 2.5};
	const bool continuousSwitching = true;

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
	float S[3];
	Step(q, dq, xy, dxy, q_ref, omega_i_ref, Jk, Mk, rk, Mb, Jbx, Jby, Jbz, Jw, rw, Bvk, Bvm, Bvb, l, g, COM_X, COM_Y, COM_Z, K, eta, epsilon, continuousSwitching, true, false, lspc::ParameterTypes::OMEGA_BODY_MANIFOLD, Torque, S);

	float Torque_Expected[3] = {-1.0488, -0.2760, 2.0310};
	if (Math_Round(Torque[0], 4) == Math_Round(Torque_Expected[0], 4) &&
		Math_Round(Torque[1], 4) == Math_Round(Torque_Expected[1], 4) &&
		Math_Round(Torque[2], 4) == Math_Round(Torque_Expected[2], 4))
		return true;
	else
		return false;
}
