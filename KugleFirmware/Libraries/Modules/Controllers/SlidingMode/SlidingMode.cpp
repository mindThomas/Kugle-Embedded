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
 
#include "SlidingMode.h"
 
#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

#include "Quaternion.h"
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

void SlidingMode::Step(float X[12], float q_ref[4], float tau[3], float S[3])
{
    float * x = &X[0];
    float * y = &X[1];
    float * q0 = &X[2];
    float * q1 = &X[3];
    float * q2 = &X[4];
    float * q3 = &X[5];
    float * dx = &X[6];
    float * dy = &X[7];
    float * dq0 = &X[8];
    float * dq1 = &X[9];
    float * dq2 = &X[10];
    float * dq3 = &X[11];

    // See ARM-CMSIS DSP library for matrix operations: https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
    float q[4] = {*q0, *q1, *q2, *q3}; arm_matrix_instance_f32 q_; arm_mat_init_f32(&q_, 4, 1, (float32_t *)q);
    float dq[4] = {*dq0, *dq1, *dq2, *dq3}; arm_matrix_instance_f32 dq_; arm_mat_init_f32(&dq_, 4, 1, (float32_t *)dq);
    float chi[6] = {*x, *y, *q0, *q1, *q2, *q3}; arm_matrix_instance_f32 chi_; arm_mat_init_f32(&chi_, 6, 1, (float32_t *)chi);
    float dchi[6] = {*dx, *dy, *dq0, *dq1, *dq2, *dq3}; arm_matrix_instance_f32 dchi_; arm_mat_init_f32(&dchi_, 6, 1, (float32_t *)dchi);

    float M[6*6]; arm_matrix_instance_f32 M_; arm_mat_init_f32(&M_, 6, 6, M);
    float C[6*6]; arm_matrix_instance_f32 C_; arm_mat_init_f32(&C_, 6, 6, C);
    float G[6]; arm_matrix_instance_f32 G_; arm_mat_init_f32(&G_, 6, 1, G);
    float D[6]; arm_matrix_instance_f32 D_; arm_mat_init_f32(&D_, 6, 1, D);
    float Q[6*3]; arm_matrix_instance_f32 Q_; arm_mat_init_f32(&Q_, 6, 3, Q);

    #if DEBUG
    tic();
    #endif
    mass(_params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jk, _params.model.Jw, _params.model.Mb, _params.model.Mk, *q0, *q1, *q2, *q3, _params.model.rk, _params.model.rw, M);
    coriolis(_params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.Mb, 0.0f, *dq0, *dq1, *dq2, *dq3, *dx, *dy, *q0, *q1, *q2, *q3, _params.model.rk, _params.model.rw, C); // beta = 0
    gravity(_params.model.COM_X, _params.model.COM_Y, _params.model.COM_Z, _params.model.Mb, 0.0f, _params.model.g, *q0, *q1, *q2, *q3, G); // beta = 0
    friction(_params.model.Bvb, _params.model.Bvk, _params.model.Bvm, 0.0f, *dq0, *dq1, *dq2, *dq3, *dx, *dy, *q0, *q1, *q2, *q3, _params.model.rk, _params.model.rw, D);
    input_forces(*q0, *q1, *q2, *q3, _params.model.rk, _params.model.rw, Q);
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

    float q_err[4];
    // q_err = Gamma(q_ref)' * q
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

    Serial.println("devecGammaQ_T_ = ");
    Matrix_Print(devecGammaQ_T, 3, 4);

    Serial.println("Input = ");
    Matrix_Print(Input, 3, 3);

    Serial.println("InputInv = ");
    Matrix_Print(InputInv, 3, 3);
    #endif

    /* tau_eq = InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq - K*devec*Gamma(q_ref)'*dq); */
    float sum[3]; arm_matrix_instance_f32 sum_; arm_mat_init_f32(&sum_, 3, 1, sum);
    float tmp3[3]; arm_matrix_instance_f32 tmp3_; arm_mat_init_f32(&tmp3_, 3, 1, tmp3);
    float tmp4[4]; arm_matrix_instance_f32 tmp4_; arm_mat_init_f32(&tmp4_, 4, 1, tmp4);

    float K[3] = {_params.controller.K[0], _params.controller.K[1], _params.controller.K[2]};
    float devecGammaQref_T[3*4]; arm_matrix_instance_f32 devecGammaQref_T_; arm_mat_init_f32(&devecGammaQref_T_, 3, 4, devecGammaQref_T);
    Quaternion_mat_devecGammaT(q_ref, devecGammaQ_T);
    arm_mat_mult_f32(&devecGammaQref_T_, &dq_, &tmp3_); // devec*Gamma(q_ref)'*dq
    arm_mult_f32(K, tmp3, sum, 3); // K*devec*Gamma(q_ref)'*dq
    //arm_mult_f32(devec(dq), K, sum, 3); // K*devec*dq

    arm_mat_mult_f32(&devecGammaQ_T_, &fq_, &tmp3_); // devec*Gamma(q)'*fq
    arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Gamma(q)'*fq
    arm_add_f32(tmp3, sum, sum, 3);

    float devecGammaDQ_T[3*4]; arm_matrix_instance_f32 devecGammaDQ_T_; arm_mat_init_f32(&devecGammaDQ_T_, 3, 4, devecGammaDQ_T);
    Quaternion_mat_devecGammaT(dq, devecGammaDQ_T);

    arm_mat_mult_f32(&devecGammaDQ_T_, &dq_, &tmp3_); // devec*Gamma(dq)'*dq
    arm_scale_f32(tmp3, 2.f, tmp3, 3); // 2*devec*Gamma(dq)'*dq
    arm_add_f32(tmp3, sum, sum, 3);

    arm_negate_f32(sum, sum, 3);

    #if DEBUG
    Serial.println("sum = ");
    Matrix_Print(sum, 3, 1);
    #endif

    float tau_eq[3]; arm_matrix_instance_f32 tau_eq_; arm_mat_init_f32(&tau_eq_, 3, 1, tau_eq);
    arm_mat_mult_f32(&InputInv_, &sum_, &tau_eq_); // InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq - K*devec*Gamma(q_ref)'*dq)

    #if DEBUG
    Serial.println("tau_eq = ");
    Matrix_Print(tau_eq, 3, 1);
    #endif

    /* omeg = 2*devec*Gamma(q)'*dq; % inertial angular velocity */
    float omeg[3]; arm_matrix_instance_f32 omeg_; arm_mat_init_f32(&omeg_, 3, 1, omeg);
    arm_mat_mult_f32(&devecGammaQ_T_, &dq_, &omeg_);
    arm_scale_f32(omeg, 2.f, omeg, 3);

    #if DEBUG
    Serial.println("omeg = ");
    Matrix_Print(omeg, 3, 1);
    #endif

    /* S = omeg + K*devec*q_err */
    arm_mult_f32(devec(q_err), K, S, 3);
    arm_add_f32(omeg, S, S, 3);

    #if DEBUG
    Serial.println("S = ");
    Matrix_Print(S, 3, 1);
    #endif

    float u[3]; arm_matrix_instance_f32 u_; arm_mat_init_f32(&u_, 3, 1, u);
    if (_params.controller.ContinousSwitching) { // continous switching law
      /* satS = sat(S/epsilon); */
      float satS[3];
      Saturation(S, 3, _params.controller.epsilon, satS);

      arm_scale_f32(satS, -_params.controller.eta, u, 3);  // u = -eta * satS;
    } else { // discontinous switching law
      /* sgnS = sign(S); */
      float sgnS[3];
      Sign(S, 3, sgnS);

      arm_scale_f32(sgnS, -_params.controller.eta, u, 3);    // u = -eta * sgnS;
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

void SlidingMode::HeadingIndependentReferenceManual(const float q_ref[4], const float q[4], float q_ref_out[4])
{
  /* Derive tilt and heading from combined quaternion */
  // Z unit vector of Body in Inertial frame
  // I_e_Z = devec * Phi(q) * Gamma(q)' * [0;0;0;1];
  // Extract direction in which this Z vector is pointing (project down to XY-plane)
  // direction = I_e_Z(1:2);  % direction = [eye(2), zeros(2,1)] * I_e_Z;
  float direction[2];
  direction[0] = 2*q[0]*q[2] + 2*q[1]*q[3];
  direction[1] = 2*q[2]*q[3] - 2*q[0]*q[1];

  // Tilt amount corresponds to sin^-1 of the length of this vector
  float normDirection = sqrtf(direction[0]*direction[0] + direction[1]*direction[1]);
  float tilt = asinf(normDirection);

  // normalize direction vector before forming tilt quaternion
  if (normDirection != 0) {
    direction[0] = direction[0] / normDirection;
    direction[1] = direction[1] / normDirection;
  } else {
    direction[0] = 0;
    direction[1] = 0;
  }

  // Tilt quaternion describes the current (heading independent) tilt of the robot
  float q_tilt[4];
  q_tilt[0] = cosf(tilt/2);
  q_tilt[1] = sinf(tilt/2) * -direction[1];
  q_tilt[2] = sinf(tilt/2) * direction[0];
  q_tilt[3] = 0;

  // Remove the tilt from the current quaternion to extract the heading part of the quaternion
  float q_heading[4];
  Quaternion_PhiT(q_tilt, q, q_heading); // q_heading = Phi(q_tilt)' * q;

  /* Derive tilt from quaternion reference */
  // Z unit vector of Body in Inertial frame
  // I_e_Z = devec * Phi(q_ref) * Gamma(q_ref)' * [0;0;0;1];
  // Extract direction in which this Z vector is pointing (project down to XY-plane)
  // direction = I_e_Z(1:2); //direction = [eye(2), zeros(2,1)] * I_e_Z;
  float direction_ref[2];
  direction_ref[0] = 2*q_ref[0]*q_ref[2] + 2*q_ref[1]*q_ref[3];
  direction_ref[1] = 2*q_ref[2]*q_ref[3] - 2*q_ref[0]*q_ref[1];
  // Tilt amount corresponds to sin^-1 of the length of this vector
  float normDirectionRef = sqrtf(direction_ref[0]*direction_ref[0] + direction_ref[1]*direction_ref[1]);
  float tilt_ref = asinf(normDirectionRef);

  // normalize direction vector before forming tilt quaternion
  if (normDirectionRef != 0) {
    direction_ref[0] = direction_ref[0] / normDirectionRef;
    direction_ref[1] = direction_ref[1] / normDirectionRef;
  } else {
    direction_ref[0] = 0;
    direction_ref[1] = 0;
  }

  // Tilt quaternion describes the current (heading independent) tilt of the robot
  float q_tilt_ref[4];
  q_tilt_ref[0] = cosf(tilt_ref/2);
  q_tilt_ref[1] = sinf(tilt_ref/2) * -direction_ref[1];
  q_tilt_ref[2] = sinf(tilt_ref/2) * direction_ref[0];
  q_tilt_ref[3] = 0;

  // Remove the tilt from the current quaternion to extract the heading part of the quaternion
  float q_heading_ref[4];
  Quaternion_PhiT(q_tilt_ref, q_ref, q_heading_ref); // q_heading_ref = Phi(q_tilt_ref)' * q_ref;

  /* Calculate reference quaternion by multiplying with the desired reference */
  // We multiply on the right side since the heading quaternion is given
  // around the Z-axis in the body frame - thus in the frame of the desired tilt angle
  //Quaternion_Phi(q_tilt_ref, q_heading, q_ref_out); // q_ref_out = Phi(q_tilt_ref) * q_heading;     % if desired tilt reference is given in inertial heading frame
  Quaternion_Phi(q_heading, q_tilt_ref, q_ref_out); // q_ref_out = Phi(q_heading) * q_tilt_ref;      % if desired tilt reference is given in body heading frame
}

void SlidingMode::HeadingIndependentQdot(const float dq[4], const float q[4], float q_dot_out[4])
{
  /* omeg = 2*Phi(q)'*dq    % body
     removeYaw = [eye(3), zeros(3,1); zeros(1,3), 0];
     dq_withoutYaw = SimplifyWithQuatConstraint(1/2 * Phi(q) * removeYaw * 2*Phi(q)' * dq, q)
  */
  /*q_dot_out[0] = dq[0]*q[0]*q[0] + dq[3]*q[3]*q[0] + dq[0]*q[1]*q[1] - dq[2]*q[3]*q[1] + dq[0]*q[2]*q[2] + dq[1]*q[3]*q[2];
  q_dot_out[1] = dq[1]*q[0]*q[0] - dq[3]*q[2]*q[0] + dq[1]*q[1]*q[1] + dq[2]*q[2]*q[1] + dq[1]*q[3]*q[3] + dq[0]*q[2]*q[3];
  q_dot_out[2] = dq[2]*q[0]*q[0] + dq[3]*q[1]*q[0] + dq[2]*q[2]*q[2] + dq[1]*q[1]*q[2] + dq[2]*q[3]*q[3] - dq[0]*q[1]*q[3];
  q_dot_out[3] = dq[3]*q[1]*q[1] + dq[2]*q[0]*q[1] + dq[3]*q[2]*q[2] - dq[1]*q[0]*q[2] + dq[3]*q[3]*q[3] + dq[0]*q[0]*q[3];*/

  /* No body angular velocity */
  /* omeg = devec*2*Phi(q)'*dq    % body
     omeg_noYaw = [omeg(1:2); 0]
     dq_noYaw = 1/2 * Phi(q) * vec*omeg_noYaw
  */
  q_dot_out[0] = dq[0]*q[1]*q[1] + dq[0]*q[2]*q[2] - dq[1]*q[0]*q[1] - dq[2]*q[0]*q[2] + dq[1]*q[2]*q[3] - dq[2]*q[1]*q[3];
  q_dot_out[1] = dq[1]*q[0]*q[0] + dq[1]*q[3]*q[3] - dq[0]*q[0]*q[1] + dq[0]*q[2]*q[3] - dq[3]*q[0]*q[2] - dq[3]*q[1]*q[3];
  q_dot_out[2] = dq[2]*q[0]*q[0] + dq[2]*q[3]*q[3] - dq[0]*q[0]*q[2] - dq[0]*q[1]*q[3] + dq[3]*q[0]*q[1] - dq[3]*q[2]*q[3];
  q_dot_out[3] = dq[3]*q[1]*q[1] + dq[3]*q[2]*q[2] - dq[1]*q[0]*q[2] + dq[2]*q[0]*q[1] - dq[1]*q[1]*q[3] - dq[2]*q[2]*q[3];

  // The second method is only slightly different from the first, in the sense that it forces the q0 component of omeg to be 0 (sort of a rectification)
}
