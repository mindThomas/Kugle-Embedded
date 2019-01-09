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
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "SlidingMode.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

#include "Quaternion.h"
#include "Math.h"
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
	Step(q, dq, xy, dxy, q_ref, omega_ref, _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g, _params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z, _params.controller.K, _params.controller.eta, _params.controller.epsilon, _params.controller.ContinousSwitching, tau, S);
}

/**
 * @brief 	Compute control output with Sliding mode controller given a quaternion attitude reference and angular velocity reference
 * @param	q[4]      	  Input: current quaternion state estimate defined in inertial frame
 * @param	dq[4]     	  Input: current quaternion derivative estimate defined in inertial frame
 * @param	xy[2]	  	  Input: current ball (center) position defined in inertial frame
 * @param	dxy[2]    	  Input: current ball (center) velocity defined in inertial frame
 * @param	q_ref[4]  	  Input: desired/reference quaternion defined in inertial frame
 * @param	omega_ref[3]  Input: desired/reference angular velocity defined in inertial frame
 * @param   model params       Input: Different mixed model constants
 * @param   controller params  Input: Different tunable Sliding mode controller parameters
 * @param	tau[3]    	  Output: motor torque outputs [Nm] where tau[0] is the motor placed along the x-axis of the robot-centric frame
 * @param	S[3]      	  Output: sliding manifold values for the three surfaces used for the attitude control
 */
void SlidingMode::Step(const float q[4], const float dq[4], const float xy[2], const float dxy[2], const float q_ref[4], const float omega_ref[3], const float Jk, const float Mk, const float rk, const float Mb, const float Jbx, const float Jby, const float Jbz, const float Jw, const float rw, const float Bvk, const float Bvm, const float Bvb, const float l, const float g_const, const float COM_X, const float COM_Y, const float COM_Z, const float K[3], const float eta, const float epsilon, const bool continuousSwitching, float tau[3], float S[3])
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

    #if DEBUG
    tic();
    #endif
    mass(COM_X, COM_Y, COM_Z, Jbx, Jby, Jbz, Jk, Jw, Mb, Mk, q[0], q[1], q[2], q[3], rk, rw, M);
    coriolis(COM_X, COM_Y, COM_Z, Jbx, Jby, Jbz, Jw, Mb, 0.0f, dq[0], dq[1], dq[2], dq[3], dxy[0], dxy[1], q[0], q[1], q[2], q[3], rk, rw, C); // beta = 0
    gravity(COM_X, COM_Y, COM_Z, Mb, 0.0f, g_const, q[0], q[1], q[2], q[3], G); // beta = 0
    friction(Bvb, Bvk, Bvm, 0.0f, dq[0], dq[1], dq[2], dq[3], dxy[0], dxy[1], q[0], q[1], q[2], q[3], rk, rw, D);
    input_forces(q[0], q[1], q[2], q[3], rk, rw, Q);
    #if DEBUG
    toc();
    #endif

    float Minv[6*6]; arm_matrix_instance_f32 Minv_; arm_mat_init_f32(&Minv_, 6, 6, Minv);
    #if DEBUG
    tic();
    #endif
    inv6x6(M, Minv);
    #if DEBUG
    toc();

    Serial.println("M = ");
    Matrix_Print(M, 6, 6);

    Serial.println("C = ");
    Matrix_Print(C, 6, 6);

    Serial.println("G = ");
    Matrix_Print(G, 6, 1);

    Serial.println("D = ");
    Matrix_Print(D, 6, 1);

    Serial.println("Q = ");
    Matrix_Print(Q, 6, 3);
    #endif

    /* f = Minv * (-C*dchi - G - D) */
    /* g = Minv * Q */
    #if DEBUG
    tic();
    #endif
    float f[6]; arm_matrix_instance_f32 f_; arm_mat_init_f32(&f_, 6, 1, f);
    float g[6*3]; arm_matrix_instance_f32 g_; arm_mat_init_f32(&g_, 6, 3, g);
    float tmp6[6]; arm_matrix_instance_f32 tmp6_; arm_mat_init_f32(&tmp6_, 6, 1, tmp6);
    arm_mat_mult_f32(&C_, &dchi_, &tmp6_);
    arm_add_f32(tmp6, G, tmp6, 6);
    arm_add_f32(tmp6, D, tmp6, 6);
    arm_negate_f32(tmp6, tmp6, 6);
    arm_mat_mult_f32(&Minv_, &tmp6_, &f_);
    arm_mat_mult_f32(&Minv_, &Q_, &g_);
    #if DEBUG
    toc();

    Serial.println("f = ");
    Matrix_Print(f, 6, 1);

    Serial.println("g = ");
    Matrix_Print(g, 6, 3);
    #endif

    arm_matrix_instance_f32 fq_; arm_mat_init_f32(&fq_, 4, 1, &f[2]);
    float gq[4*3]; arm_matrix_instance_f32 gq_; arm_mat_init_f32(&gq_, 4, 3, gq);
    Matrix_Extract(g, 6,3,  2,0,  4,3,  gq);

    #if DEBUG
    Serial.println("gq = ");
    Matrix_Print(gq, 4, 3);
    #endif

    /* Quaternion error in Inertial frame */
    // q_err = Gamma(q_ref)' * q
    float q_err[4];
    Quaternion_GammaT(q_ref, q, q_err);
    if (q_err[0] < 0)
      arm_negate_f32(q_err, q_err, 4); // invert the quaternion to get the shortest path
      //Quaternion_Conjugate(q_err);  // flip the sign of the vector part to ensure that we take the shortest path

    #if DEBUG
    Serial.println("q =");
    Quaternion_Print(q);

    Serial.println("q_ref =");
    Quaternion_Print(q_ref);

    Serial.println("q_err =");
    Quaternion_Print(q_err);
    #endif

    /* Inertial angular velocity */
    /* dq_ref = 1/2 * Gamma(q_ref) * [0;omega_ref]; */
    float dq_ref[4];
    float omega_ref_q[4] = {0, omega_ref[0], omega_ref[1], omega_ref[2]};
    Quaternion_Gamma(q_ref, omega_ref_q, dq_ref); // Gamma(q_ref) * [0;omega_ref]
    arm_scale_f32(dq_ref, 0.5f, dq_ref, 4);

    /* Inertial angular velocity */
    /* InputInv = inv(2 * devec*Gamma(q)' * gq) */
    #if DEBUG
    tic();
    #endif
    float Input[3*3]; arm_matrix_instance_f32 Input_; arm_mat_init_f32(&Input_, 3, 3, Input);
    float InputInv[3*3]; arm_matrix_instance_f32 InputInv_; arm_mat_init_f32(&InputInv_, 3, 3, InputInv);
    float devecGammaQ_T[3*4]; arm_matrix_instance_f32 devecGammaQ_T_; arm_mat_init_f32(&devecGammaQ_T_, 3, 4, devecGammaQ_T);
    Quaternion_mat_devecGammaT(q, devecGammaQ_T);
    arm_mat_mult_f32(&devecGammaQ_T_, &gq_, &Input_);
    arm_scale_f32(Input, 2.f, Input, 3*3);
    inv3x3(Input, InputInv);
    #if DEBUG
    toc();

    Debug::print("devecGammaQ_T_ = \n");
    Matrix_Print(devecGammaQ_T, 3, 4);

    Debug::print("Input = \n");
    Matrix_Print(Input, 3, 3);

    Debug::print("InputInv = \n");
    Matrix_Print(InputInv, 3, 3);
    #endif

    /* tau_eq = InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq - K*devec*Gamma(dq_ref)'*q - K*devec*Gamma(q_ref)'*dq); */
    float sum[3]; arm_matrix_instance_f32 sum_; arm_mat_init_f32(&sum_, 3, 1, sum);
    float tmp3[3]; arm_matrix_instance_f32 tmp3_; arm_mat_init_f32(&tmp3_, 3, 1, tmp3);
    float tmp4[4]; arm_matrix_instance_f32 tmp4_; arm_mat_init_f32(&tmp4_, 4, 1, tmp4);

    //float K[3] = {_params.controller.K[0], _params.controller.K[1], _params.controller.K[2]};

    float devecGammaQref_T[3*4]; arm_matrix_instance_f32 devecGammaQref_T_; arm_mat_init_f32(&devecGammaQref_T_, 3, 4, devecGammaQref_T);
    Quaternion_mat_devecGammaT(q_ref, devecGammaQref_T); // devec*Gamma(q_ref)'
    arm_mat_mult_f32(&devecGammaQref_T_, &dq_, &tmp3_); // devec*Gamma(q_ref)'*dq
    arm_mult_f32((float*)K, tmp3, sum, 3); // sum = K*devec*Gamma(q_ref)'*dq

    float devecGammaDQref_T[3*4]; arm_matrix_instance_f32 devecGammaDQref_T_; arm_mat_init_f32(&devecGammaDQref_T_, 3, 4, devecGammaDQref_T);
    Quaternion_mat_devecGammaT(dq_ref, devecGammaDQref_T); // devec*Gamma(dq_ref)'
    arm_mat_mult_f32(&devecGammaDQref_T_, &q_, &tmp3_); // devec*Gamma(dq_ref)'*q
    arm_mult_f32((float*)K, tmp3, tmp3, 3); // K*devec*Gamma(q_ref)'*dq
    arm_add_f32(tmp3, sum, sum, 3); // sum += K*devec*Gamma(q_ref)'*dq

    arm_mat_mult_f32(&devecGammaQ_T_, &fq_, &tmp3_); // devec*Gamma(q)'*fq
    arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Gamma(q)'*fq
    arm_add_f32(tmp3, sum, sum, 3); // sum += 2*devec*Gamma(q)'*fq

    float devecGammaDQ_T[3*4]; arm_matrix_instance_f32 devecGammaDQ_T_; arm_mat_init_f32(&devecGammaDQ_T_, 3, 4, devecGammaDQ_T);
    Quaternion_mat_devecGammaT(dq, devecGammaDQ_T); // devec*Gamma(dq)'
    arm_mat_mult_f32(&devecGammaDQ_T_, &dq_, &tmp3_); // devec*Gamma(dq)'*dq
    arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Gamma(dq)'*dq
    arm_add_f32(tmp3, sum, sum, 3); // sum += 2*devec*Gamma(dq)'*dq

    arm_negate_f32(sum, sum, 3); // negate the sum to get minus in front of all parts

    #if DEBUG
    Serial.println("sum = ");
    Matrix_Print(sum, 3, 1);
    #endif

    float tau_eq[3]; arm_matrix_instance_f32 tau_eq_; arm_mat_init_f32(&tau_eq_, 3, 1, tau_eq);
    arm_mat_mult_f32(&InputInv_, &sum_, &tau_eq_); // InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq - K*devec*Gamma(dq_ref)'*q - K*devec*Gamma(q_ref)'*dq)

    #if DEBUG
    Serial.println("tau_eq = ");
    Matrix_Print(tau_eq, 3, 1);
    #endif

    /* Inertial angular velocity */
    /* omega = 2*devec*Gamma(q)'*dq; */
    float omega[3]; arm_matrix_instance_f32 omega_; arm_mat_init_f32(&omega_, 3, 1, omega);
    arm_mat_mult_f32(&devecGammaQ_T_, &dq_, &omega_); // devec*Gamma(q)'*dq
    arm_scale_f32(omega, 2.f, omega, 3); // 2*devec*Gamma(q)'*dq

    #if DEBUG
    Serial.println("omega = ");
    Matrix_Print(omega, 3, 1);
    #endif

    /* S = omega - omega_ref + K*devec*q_err */
    arm_mult_f32(devec(q_err), (float*)K, S, 3); // S = K*devec*q_err
    arm_add_f32(omega, S, S, 3); // S += omega  -->  S = omega + K*devec*q_err
    arm_sub_f32(S, (float*)omega_ref, S, 3); // S -= omega_ref  -->  S = omega - omega_ref + K*devec*q_err

    #if DEBUG
    Serial.println("S = ");
    Matrix_Print(S, 3, 1);
    #endif

    float u[3]; arm_matrix_instance_f32 u_; arm_mat_init_f32(&u_, 3, 1, u);
    if (continuousSwitching) { // continous switching law
      /* satS = sat(S/epsilon); */
      float satS[3];
      Saturation(S, 3, epsilon, satS);

      arm_scale_f32(satS, -eta, u, 3);  // u = -eta * satS;
    } else { // discontinous switching law
      /* sgnS = sign(S); */
      float sgnS[3];
      Sign(S, 3, sgnS);

      arm_scale_f32(sgnS, -eta, u, 3);    // u = -eta * sgnS;
    }

    #if DEBUG
    Serial.println("u = ");
    Matrix_Print(u, 3, 1);
    #endif

    /* tau_switching = InputInv * u; */
    float tau_switching[3]; arm_matrix_instance_f32 tau_switching_; arm_mat_init_f32(&tau_switching_, 3, 1, tau_switching);
    arm_mat_mult_f32(&InputInv_, &u_, &tau_switching_);

    #if DEBUG
    Serial.println("tau_switching = ");
    Matrix_Print(tau_switching, 3, 1);
    #endif

    /* tau = tau_eq + tau_switching; */
    arm_add_f32(tau_eq, tau_switching, tau, 3);

    #if DEBUG
    Serial.println("tau = ");
    Matrix_Print(tau, 3, 1);
    #endif
}

void SlidingMode::Saturation(float * in, int size, float epsilon, float * out)
{
    arm_scale_f32(in, 1.f/epsilon, out, size);
    for (int i = 0; i < size; i++)
      out[i] = fmin(fmax(out[i], -1), 1);
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
	const float eta = 2.5;
	const float epsilon = 2.5;
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
	Step(q, dq, xy, dxy, q_ref, omega_i_ref, Jk, Mk, rk, Mb, Jbx, Jby, Jbz, Jw, rw, Bvk, Bvm, Bvb, l, g, COM_X, COM_Y, COM_Z, K, eta, epsilon, continuousSwitching, Torque, S);

	float Torque_Expected[3] = {-1.0488, -0.2760, 2.0310};
	if (Math_Round(Torque[0], 4) == Math_Round(Torque_Expected[0], 4) &&
		Math_Round(Torque[1], 4) == Math_Round(Torque_Expected[1], 4) &&
		Math_Round(Torque[2], 4) == Math_Round(Torque_Expected[2], 4))
		return true;
	else
		return false;
}
