//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Nov-2018 11:57:59
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "VelocityEstimator.h"
#include "OffsetEstimator_dEncoders2L_dq.h"
#include "SteadyStateAcceleration.h"
#include "SteadyStateAcceleration_dCOM.h"
#include "SteadyStateAcceleration_dq.h"

// Function Definitions

//
// function [X_out, P_out] = VelocityEstimator(X, P_prev, EncoderDiffMeas, qQEKF, Cov_qQEKF, qdotQEKF, SamplePeriod, n_gear, n_ticksRev, Jk,Mk,rk,Mb,Jbx,Jby,Jbz,Jw,rw,Bvk,Bvm,Bvb,l,g, COM,  Var_COM, eta_qQEKF_velocity, eta_dqQEKF_encoder)
// for q o p = Phi(q) * p
// Arguments    : const float X[2]
//                const float P_prev[4]
//                const float EncoderDiffMeas[3]
//                const float qQEKF[4]
//                const float Cov_qQEKF[16]
//                const float qdotQEKF[4]
//                float SamplePeriod
//                float n_gear
//                float n_ticksRev
//                float Jk
//                float Mk
//                float rk
//                float Mb
//                float Jbx
//                float Jby
//                float Jbz
//                float Jw
//                float rw
//                float Bvk
//                float Bvm
//                float Bvb
//                float l
//                float g
//                const float COM[3]
//                float Var_COM
//                float eta_qQEKF_velocity
//                float eta_dqQEKF_encoder
//                float X_out[2]
//                float P_out[4]
// Return Type  : void
//
__attribute__((optimize("O3"))) void VelocityEstimator(const float X[2], const float P_prev[4], const float
  EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16], const
  float qdotQEKF[4], float SamplePeriod, float n_gear, float n_ticksRev, float
  Jk, float Mk, float rk, float Mb, float, float, float, float Jw, float rw,
  float, float, float, float l, float g, const float COM[3], float Var_COM,
  float eta_qQEKF_velocity, float eta_dqQEKF_encoder, float X_out[2], float
  P_out[4])
{
  float y;
  float maxval;
  float a21;
  int rtemp;
  float fv0[3];
  int k;
  float dAcceleration_dqB[4];
  static const float fv1[9] = { 0.0F, 1.0F, 0.0F, -0.866025388F, -0.5F, 0.0F,
    0.866025388F, -0.5F, 0.0F };

  static const signed char iv0[3] = { 1, 0, 0 };

  static const signed char iv1[12] = { 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  float dAcceleration_dCOM[4];
  static const signed char iv2[3] = { 0, 1, 0 };

  float fv2[4];
  static const signed char iv3[3] = { 0, 0, 1 };

  float b_dAcceleration_dCOM[4];
  float b_P_prev[4];
  static const float fv3[16] = { 0.0F, -0.707106769F, -0.0F, -0.707106769F,
    0.707106769F, 0.0F, 0.707106769F, -0.0F, 0.0F, -0.707106769F, 0.0F,
    0.707106769F, 0.707106769F, 0.0F, -0.707106769F, 0.0F };

  float b_qQEKF[16];
  static const float fv4[16] = { 0.0F, 0.353553385F, -0.612372458F,
    -0.707106769F, -0.353553385F, 0.0F, 0.707106769F, -0.612372458F,
    0.612372458F, -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, 0.612372458F,
    0.353553385F, 0.0F };

  float b_qdotQEKF[16];
  float c_qdotQEKF[16];
  float c_qQEKF[16];
  float b_Var_COM[4];
  float W[12];
  float d_qQEKF[16];
  float d_qdotQEKF[16];
  float e_qQEKF[12];
  int r1;
  float vel_2L_to_ball_correction[3];
  static const float fv5[16] = { 0.0F, 0.353553385F, 0.612372458F, -0.707106769F,
    -0.353553385F, 0.0F, 0.707106769F, 0.612372458F, -0.612372458F,
    -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, -0.612372458F,
    0.353553385F, 0.0F };

  float b_dAcceleration_dqB[8];
  float acceleration[2];
  float dx_2L_apriori;
  float dy_2L_apriori;
  float b_Cov_qQEKF[8];
  static const signed char iv4[4] = { 1, 0, 0, 1 };

  float P_apriori[4];
  float a;
  float dEncoder_dqQEKF[12];
  double H[6];
  float b_a[12];
  float f_qQEKF[12];
  static const signed char iv5[4] = { 0, 0, 1, 0 };

  static const signed char iv6[4] = { 0, -1, 0, 0 };

  float K[6];
  float b_H[9];
  int r2;
  int r3;
  float S[9];
  float fv6[9];
  float b_y[6];
  static const signed char iv7[9] = { 2, 0, 0, 0, 2, 0, 0, 0, 2 };

  float b_dx_2L_apriori[2];
  signed char I[4];

  // 'VelocityEstimator:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p 
  // 'VelocityEstimator:4'               q(2) q(1)  -q(4) q(3);
  // 'VelocityEstimator:5'               q(3) q(4)  q(1)  -q(2);
  // 'VelocityEstimator:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'VelocityEstimator:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q 
  // 'VelocityEstimator:8'                  p(2) p(1) p(4) -p(3);
  // 'VelocityEstimator:9'                  p(3) -p(4) p(1) p(2);
  // 'VelocityEstimator:10'                  p(4) p(3) -p(2) p(1)];
  // 'VelocityEstimator:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'VelocityEstimator:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'VelocityEstimator:14' I_conj = diag([1,-1,-1,-1]);
  // 'VelocityEstimator:16' dt = SamplePeriod;
  // 'VelocityEstimator:17' xCOM = COM(1);
  // 'VelocityEstimator:18' yCOM = COM(2);
  // 'VelocityEstimator:20' TicksPrRev = n_gear * n_ticksRev;
  //  motor mapping (inverse kinematics)
  // 'VelocityEstimator:23' alpha = deg2rad(45);
  // 'VelocityEstimator:24' gamma = deg2rad(120);
  // 'VelocityEstimator:26' e1 = [1,0,0]';
  // 'VelocityEstimator:27' e2 = [0,1,0]';
  // 'VelocityEstimator:28' e3 = [0,0,1]';
  // 'VelocityEstimator:29' R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1]; 
  // 'VelocityEstimator:30' R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0]; 
  // 'VelocityEstimator:32' W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1); 
  y = rk / rw;

  // 'VelocityEstimator:33' W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2); 
  maxval = rk / rw;

  // 'VelocityEstimator:34' W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3); 
  a21 = rk / rw;

  // 'VelocityEstimator:35' W = [W1;W2;W3];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      fv0[rtemp] += fv1[rtemp + 3 * k] * (y * (float)iv0[k]);
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    dAcceleration_dqB[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      dAcceleration_dqB[rtemp] += (float)iv1[rtemp + (k << 2)] * fv0[k];
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      fv0[rtemp] += fv1[rtemp + 3 * k] * (maxval * (float)iv2[k]);
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    dAcceleration_dCOM[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      dAcceleration_dCOM[rtemp] += (float)iv1[rtemp + (k << 2)] * fv0[k];
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      fv0[rtemp] += fv1[rtemp + 3 * k] * (a21 * (float)iv3[k]);
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    fv2[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      fv2[rtemp] += (float)iv1[rtemp + (k << 2)] * fv0[k];
    }

    b_dAcceleration_dCOM[rtemp] = 0.0F;
    b_P_prev[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      b_dAcceleration_dCOM[rtemp] += fv3[rtemp + (k << 2)] * dAcceleration_dqB[k];
      b_P_prev[rtemp] += fv4[rtemp + (k << 2)] * dAcceleration_dCOM[k];
    }
  }

  //  Split state vector, X[k-1], into individual variables
  // 'VelocityEstimator:38' dx_2L = X(1);
  // 'VelocityEstimator:39' dy_2L = X(2);
  // 'VelocityEstimator:41' vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]'; 
  dAcceleration_dqB[0] = 0.0F;
  dAcceleration_dqB[1] = 0.0F;
  dAcceleration_dqB[2] = 0.0F;
  dAcceleration_dqB[3] = 2.0F * l;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = qQEKF[3];
  b_qQEKF[13] = -qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = -qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = qQEKF[2];
  b_qQEKF[11] = -qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  b_qdotQEKF[0] = qdotQEKF[0];
  b_qdotQEKF[1] = -qdotQEKF[1];
  b_qdotQEKF[2] = -qdotQEKF[2];
  b_qdotQEKF[3] = -qdotQEKF[3];
  b_qdotQEKF[4] = qdotQEKF[1];
  b_qdotQEKF[5] = qdotQEKF[0];
  b_qdotQEKF[6] = -qdotQEKF[3];
  b_qdotQEKF[7] = qdotQEKF[2];
  b_qdotQEKF[8] = qdotQEKF[2];
  b_qdotQEKF[9] = qdotQEKF[3];
  b_qdotQEKF[10] = qdotQEKF[0];
  b_qdotQEKF[11] = -qdotQEKF[1];
  b_qdotQEKF[12] = qdotQEKF[3];
  b_qdotQEKF[13] = -qdotQEKF[2];
  b_qdotQEKF[14] = qdotQEKF[1];
  b_qdotQEKF[15] = qdotQEKF[0];
  c_qdotQEKF[0] = qdotQEKF[0];
  c_qdotQEKF[4] = -qdotQEKF[1];
  c_qdotQEKF[8] = -qdotQEKF[2];
  c_qdotQEKF[12] = -qdotQEKF[3];
  c_qdotQEKF[1] = qdotQEKF[1];
  c_qdotQEKF[5] = qdotQEKF[0];
  c_qdotQEKF[9] = qdotQEKF[3];
  c_qdotQEKF[13] = -qdotQEKF[2];
  c_qdotQEKF[2] = qdotQEKF[2];
  c_qdotQEKF[6] = -qdotQEKF[3];
  c_qdotQEKF[10] = qdotQEKF[0];
  c_qdotQEKF[14] = qdotQEKF[1];
  c_qdotQEKF[3] = qdotQEKF[3];
  c_qdotQEKF[7] = qdotQEKF[2];
  c_qdotQEKF[11] = -qdotQEKF[1];
  c_qdotQEKF[15] = qdotQEKF[0];
  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = -qQEKF[3];
  c_qQEKF[7] = qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = -qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = -qQEKF[2];
  c_qQEKF[14] = qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    b_Var_COM[rtemp] = 0.0F;
    W[rtemp] = b_dAcceleration_dCOM[rtemp];
    W[4 + rtemp] = b_P_prev[rtemp];
    for (k = 0; k < 4; k++) {
      d_qQEKF[rtemp + (k << 2)] = 0.0F;
      d_qdotQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        d_qQEKF[rtemp + (k << 2)] += b_qQEKF[rtemp + (r1 << 2)] * b_qdotQEKF[r1
          + (k << 2)];
        d_qdotQEKF[rtemp + (k << 2)] += c_qdotQEKF[rtemp + (r1 << 2)] *
          c_qQEKF[r1 + (k << 2)];
      }

      b_Var_COM[rtemp] += fv5[rtemp + (k << 2)] * fv2[k];
    }

    W[8 + rtemp] = b_Var_COM[rtemp];
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 4; k++) {
      b_qQEKF[k + (rtemp << 2)] = d_qQEKF[k + (rtemp << 2)] + d_qdotQEKF[k +
        (rtemp << 2)];
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      e_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        e_qQEKF[rtemp + (k << 2)] += b_qQEKF[rtemp + (r1 << 2)] * (float)iv1[r1
          + (k << 2)];
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      fv0[rtemp] += dAcceleration_dqB[k] * e_qQEKF[k + (rtemp << 2)];
    }

    vel_2L_to_ball_correction[rtemp] = fv0[rtemp];
  }

  // 'VelocityEstimator:42' dx_ball = dx_2L - vel_2L_to_ball_correction(1);
  // 'VelocityEstimator:43' dy_ball = dy_2L - vel_2L_to_ball_correction(2);
  //  Process covariances
  // dAcceleration_dqB = OffsetEstimator_Acceleration_dqB(Jk,Jw,Mb,Mk,g,l,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw,xCOM,yCOM);     
  // 'VelocityEstimator:47' dAcceleration_dqB = SteadyStateAcceleration_dq(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dq(COM[0], COM[1], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
    qQEKF[2], qQEKF[3], rk, b_dAcceleration_dqB);

  // dAcceleration_dqB = SteadyStateAcceleration_dq_Full(xCOM,yCOM,l,Jk,Jw,Mb,Mk,dx_ball,dy_ball,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'VelocityEstimator:49' dAcceleration_dCOM = SteadyStateAcceleration_dCOM(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dCOM(COM[0], COM[1], l, Jk, Mb, Mk, g, qQEKF[0],
    qQEKF[1], qQEKF[2], qQEKF[3], rk, b_dAcceleration_dCOM);

  // 'VelocityEstimator:51' cov_COM = Var_COM * eye(2);
  // 'VelocityEstimator:52' cov_velocity = dt^2 * dAcceleration_dqB * eta_qQEKF_velocity*Cov_qQEKF * dAcceleration_dqB' + dt^2 * dAcceleration_dCOM * cov_COM * dAcceleration_dCOM'; 
  maxval = SamplePeriod * SamplePeriod;
  a21 = SamplePeriod * SamplePeriod;

  //  Setup covariance matrices
  // 'VelocityEstimator:55' Q = [cov_velocity];
  //  Measurement covariances						
  // 'VelocityEstimator:58' cov_quantization = 0.5 * eye(3);
  //     %% Prediction step
  // 'VelocityEstimator:62' X_apriori = zeros(2,1);
  // 'VelocityEstimator:64' dpsi1 = (2*pi)/(dt*TicksPrRev) * EncoderDiffMeas(1); 
  // 'VelocityEstimator:65' dpsi2 = (2*pi)/(dt*TicksPrRev) * EncoderDiffMeas(2); 
  // 'VelocityEstimator:66' dpsi3 = (2*pi)/(dt*TicksPrRev) * EncoderDiffMeas(3); 
  // 'VelocityEstimator:67' dxdy = ForwardKinematics(dpsi1,dpsi2,dpsi3,qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // FORWARDKINEMATICS
  //     DXDY = FORWARDKINEMATICS(DPSI1,DPSI2,DPSI3,DQ1,DQ2,DQ3,DQ4,Q1,Q2,Q3,Q4,RK,RW) 
  //     This function was generated by the Symbolic Math Toolbox version 8.1.
  //     17-Oct-2018 09:53:34
  // 'ForwardKinematics:8' t2 = sqrt(2.0);
  // 'ForwardKinematics:9' t3 = 1.0./rk;
  // 'ForwardKinematics:10' t4 = dpsi1.*-2.0+dpsi2+dpsi3;
  // 'ForwardKinematics:11' t5 = q1.^2;
  // 'ForwardKinematics:12' t6 = q2.^2;
  // 'ForwardKinematics:13' t7 = q3.^2;
  // 'ForwardKinematics:14' t8 = q4.^2;
  // 'ForwardKinematics:15' t9 = sqrt(6.0);
  // 'ForwardKinematics:16' t10 = dpsi2-dpsi3;
  // 'ForwardKinematics:17' t11 = q1.*q4.*2.0;
  // 'ForwardKinematics:18' t12 = q2.*q3.*2.0;
  // 'ForwardKinematics:19' t13 = dpsi1+dpsi2+dpsi3;
  // 'ForwardKinematics:20' dxdy = [-rk.*(dq1.*q3.*2.0-dq3.*q1.*2.0-dq2.*q4.*2.0+dq4.*q2.*2.0+rw.*t2.*t3.*t13.*(q1.*q2.*2.0-q3.*q4.*2.0).*(1.0./3.0)+rw.*t3.*t9.*t10.*(t5-t6+t7-t8).*(1.0./3.0)-rw.*t2.*t3.*t4.*(t11+t12).*(1.0./3.0));-rk.*(dq1.*q2.*-2.0+dq2.*q1.*2.0-dq3.*q4.*2.0+dq4.*q3.*2.0+rw.*t2.*t3.*t13.*(q1.*q3.*2.0+q2.*q4.*2.0).*(1.0./3.0)+rw.*t2.*t3.*t4.*(t5+t6-t7-t8).*(1.0./3.0)+rw.*t3.*t9.*t10.*(t11-t12).*(1.0./3.0))]; 
  //  Propagate the velocity based on shape-accelerated model for acceleration	
  // acceleration = OffsetEstimator_Acceleration(Jk,Jw,Mb,Mk,g,l,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),0,0,rk,rw,xCOM,yCOM);     
  // 'VelocityEstimator:71' acceleration = SteadyStateAcceleration(xCOM,yCOM,l,Jk,Jw,Mb,Mk,dx_ball,dy_ball,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  SteadyStateAcceleration(COM[0], COM[1], l, Jk, Jw, Mb, Mk, X[0] -
    vel_2L_to_ball_correction[0], X[1] - vel_2L_to_ball_correction[1], g, qQEKF
    [0], qQEKF[1], qQEKF[2], qQEKF[3], rk, rw, acceleration);

  // acceleration = SteadyStateAcceleration(xCOM,yCOM,l,Jk,Jw,Mb,Mk,dxdy(1),dxdy(2),g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // dAcceleration_dCOM = OffsetEstimator_Acceleration_dCOM(Jk,Jw,Mb,Mk,g,l,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw,xCOM,yCOM) ; 
  //      q_eul = quat2eul(qQEKF','ZYX');
  //      c1 = 7;
  //      c2 = 1;
  //      c3 = 20.4918;
  //      COM_X = xCOM;
  //      COM_Y = yCOM;
  //      acceleration = [c2*(cos(q_eul(1))*(c1*q_eul(2)+c3*COM_X) - sin(q_eul(1))*(c1*q_eul(3)+c3*COM_Y)); 
  //                      c2*(sin(q_eul(1))*(c1*q_eul(2)+c3*COM_X) + cos(q_eul(1))*(c1*q_eul(3)+c3*COM_Y))]; 
  // 'VelocityEstimator:84' dx_2L_apriori = dx_2L + dt * acceleration(1);
  dx_2L_apriori = X[0] + SamplePeriod * acceleration[0];

  // 'VelocityEstimator:85' dy_2L_apriori = dy_2L + dt * acceleration(2);
  dy_2L_apriori = X[1] + SamplePeriod * acceleration[1];

  //  Determine model Jacobian (F)
  // 'VelocityEstimator:88' F_prev = eye(2);
  //  Set apriori state
  // 'VelocityEstimator:91' X_apriori = [dx_2L_apriori
  // 'VelocityEstimator:92' 				 dy_2L_apriori];
  //  Calculate apriori covariance of estimate error
  // 'VelocityEstimator:95' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 2; k++) {
      b_P_prev[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_P_prev[rtemp + (k << 1)] += P_prev[rtemp + (r1 << 1)] * (float)iv4[r1
          + (k << 1)];
      }
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 2; k++) {
      b_Cov_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        b_Cov_qQEKF[rtemp + (k << 2)] += Cov_qQEKF[rtemp + (r1 << 2)] * (maxval *
          b_dAcceleration_dqB[r1 + (k << 2)] * eta_qQEKF_velocity);
      }
    }
  }

  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 2; k++) {
      b_Var_COM[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_Var_COM[rtemp + (k << 1)] += Var_COM * (float)iv4[rtemp + (r1 << 1)] *
          (a21 * b_dAcceleration_dCOM[r1 + (k << 1)]);
      }

      dAcceleration_dqB[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        dAcceleration_dqB[rtemp + (k << 1)] += b_dAcceleration_dqB[r1 + (rtemp <<
          2)] * b_Cov_qQEKF[r1 + (k << 2)];
      }
    }
  }

  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 2; k++) {
      dAcceleration_dCOM[rtemp + (k << 1)] = 0.0F;
      maxval = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        dAcceleration_dCOM[rtemp + (k << 1)] += b_dAcceleration_dCOM[r1 + (rtemp
          << 1)] * b_Var_COM[r1 + (k << 1)];
        maxval += (float)iv4[rtemp + (r1 << 1)] * b_P_prev[r1 + (k << 1)];
      }

      P_apriori[rtemp + (k << 1)] = maxval + (dAcceleration_dqB[rtemp + (k << 1)]
        + dAcceleration_dCOM[rtemp + (k << 1)]);
    }
  }

  //     %% Update/correction step
  // 'VelocityEstimator:98' z = [EncoderDiffMeas];
  //  Accelerometer Measurement model
  // z_acc_hat = OffsetEstimator_Accelerometer(Jk,Mb,Mk,g,l,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),qxIMU_apriori,qyIMU_apriori,rk,xCOM_apriori,yCOM_apriori); 
  // z_acc_hat = devec * Phi(qQEKF)'*Gamma(qQEKF) * vec*([ddx_apriori;ddy_apriori;0] - [0;0;-g]); 
  // vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]'; 
  // 'VelocityEstimator:104' dx_ball_apriori = dx_2L_apriori - vel_2L_to_ball_correction(1); 
  // 'VelocityEstimator:105' dy_ball_apriori = dy_2L_apriori - vel_2L_to_ball_correction(2); 
  // z_encoder_hat = OffsetEstimator_Encoders(dt,dx_ball_apriori,dy_ball_apriori,n_gear,n_ticksRev,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),0,0,rw); 
  // z_encoder_hat = OffsetEstimator_Encoders2L(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dt,dx_2L_apriori,dy_2L_apriori,l,n_gear,n_ticksRev,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'VelocityEstimator:108' z_encoder_hat = (n_gear*n_ticksRev)/(2*pi) * dt * W * (1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-dy_ball_apriori;dx_ball_apriori;0] - 2*Phi(qQEKF)'*qdotQEKF); 
  a = n_gear * n_ticksRev / 6.28318548F * SamplePeriod;
  y = 1.0F / rk;

  // dencoders_dq = jacobian(encoders, q);
  // 'VelocityEstimator:110' z_hat = z_encoder_hat;
  //  Phi(qQEKF)' * Gamma(qQEKF) * [zeros(1,3);0,-1,0;1,0,0;zeros(1,3)] * -devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]' 
  //  qQEKF' o ([zeros(1,3);0,-1,0;1,0,0;zeros(1,3)] * -devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]') o qQEKF 
  //  qQEKF' o ([zeros(1,3);0,-1,0;1,0,0;zeros(1,3)] * -devec * (qdotQEKF o [0,0,0,2*l]' o qQEKF' + QEKF o [0,0,0,2*l]' o qdotQEKF') o qQEKF 
  // dEncoder_dqQEKF = (n_gear*n_ticksRev)/(2*pi) * dt * W * (1/rk * (Phi(qQEKF)'*Phi([0;-dy_ball_apriori;dx_ball_apriori;0]) + Gamma(qQEKF)*Gamma([0;-dy_ball_apriori;dx_ball_apriori;0])*I_conj) - 2*Gamma(qdotQEKF)*I_conj); 
  // 'VelocityEstimator:117' dEncoder_dqQEKF = OffsetEstimator_dEncoders2L_dq(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dt,dx_2L_apriori,dy_2L_apriori,l,n_gear,n_ticksRev,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  OffsetEstimator_dEncoders2L_dq(qdotQEKF[0], qdotQEKF[1], qdotQEKF[2],
    qdotQEKF[3], SamplePeriod, dx_2L_apriori, dy_2L_apriori, l, n_gear,
    n_ticksRev, qQEKF[0], qQEKF[1], qQEKF[2], qQEKF[3], rk, rw, dEncoder_dqQEKF);

  // dAcceleration_dqB_apriori = OffsetEstimator_Acceleration_dqB(Jk,Mb,Mk,g,l,qB(1),qB(2),qB(3),qB(4),rk,xCOM,yCOM); 
  // dAcceleration_dCOM_apriori = OffsetEstimator_Acceleration_dCOM(Jk,Mb,Mk,g,l,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),qxIMU,qyIMU,rk,xCOM,yCOM); 
  //  Measurement Jacobian	
  // 'VelocityEstimator:122' H = zeros(3,2);
  for (rtemp = 0; rtemp < 6; rtemp++) {
    H[rtemp] = 0.0;
  }

  // 'VelocityEstimator:123' H(1:3,1) = (n_gear*n_ticksRev)/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;0;1;0]; 
  maxval = n_gear * n_ticksRev / 6.28318548F * SamplePeriod;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 4; k++) {
      b_a[k + (rtemp << 2)] = maxval * W[k + (rtemp << 2)] / rk;
    }
  }

  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      e_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        e_qQEKF[rtemp + (k << 2)] += b_qQEKF[rtemp + (r1 << 2)] * b_a[r1 + (k <<
          2)];
      }
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      f_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        f_qQEKF[rtemp + (k << 2)] += c_qQEKF[rtemp + (r1 << 2)] * e_qQEKF[r1 +
          (k << 2)];
      }
    }
  }

  //  d encoder_meas  /  d dx_2L
  // 'VelocityEstimator:124' H(1:3,2) = (n_gear*n_ticksRev)/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-1;0;0]; 
  maxval = n_gear * n_ticksRev / 6.28318548F * SamplePeriod;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      b_a[k + (rtemp << 2)] = maxval * W[k + (rtemp << 2)] / rk;
      fv0[rtemp] += (float)iv5[k] * f_qQEKF[k + (rtemp << 2)];
    }

    H[rtemp << 1] = fv0[rtemp];
  }

  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      e_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        e_qQEKF[rtemp + (k << 2)] += b_qQEKF[rtemp + (r1 << 2)] * b_a[r1 + (k <<
          2)];
      }
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      f_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        f_qQEKF[rtemp + (k << 2)] += c_qQEKF[rtemp + (r1 << 2)] * e_qQEKF[r1 +
          (k << 2)];
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      fv0[rtemp] += (float)iv6[k] * f_qQEKF[k + (rtemp << 2)];
    }

    H[1 + (rtemp << 1)] = fv0[rtemp];
  }

  //  d encoder_meas  /  d dy_2L
  //  Calculate measurement covariance
  // R_acc = cov_acc + devec*Phi(qQEKF)'*Gamma(qQEKF)*vec* cov_ss_acc * devec*Gamma(qQEKF)'*Phi(qQEKF)*vec;     
  // 'VelocityEstimator:128' R_encoder = 4*cov_quantization + eta_dqQEKF_encoder*dEncoder_dqQEKF * Cov_qQEKF * dEncoder_dqQEKF'; 
  //  + ((n_gear*n_ticksRev)/(2*pi)*dt)^2 * W * vec * cov_omega * devec * W';
  // 'VelocityEstimator:129' R = [R_encoder];
  //  Calculate Kalman gain
  // 'VelocityEstimator:132' S = H * P_apriori * H' + R;
  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 3; k++) {
      K[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        K[rtemp + (k << 1)] += P_apriori[rtemp + (r1 << 1)] * (float)H[r1 + (k <<
          1)];
      }
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      e_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        e_qQEKF[rtemp + (k << 2)] += Cov_qQEKF[rtemp + (r1 << 2)] *
          (eta_dqQEKF_encoder * dEncoder_dqQEKF[r1 + (k << 2)]);
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_H[rtemp + 3 * k] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_H[rtemp + 3 * k] += (float)H[r1 + (rtemp << 1)] * K[r1 + (k << 1)];
      }

      maxval = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        maxval += dEncoder_dqQEKF[r1 + (rtemp << 2)] * e_qQEKF[r1 + (k << 2)];
      }

      fv6[rtemp + 3 * k] = (float)iv7[rtemp + 3 * k] + maxval;
    }
  }

  // K = P_apriori * H' * inv(S);
  // 'VelocityEstimator:134' K = P_apriori * H' / S;
  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      S[k + 3 * rtemp] = b_H[k + 3 * rtemp] + fv6[k + 3 * rtemp];
    }

    for (k = 0; k < 2; k++) {
      b_y[rtemp + 3 * k] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_y[rtemp + 3 * k] += (float)H[r1 + (rtemp << 1)] * P_apriori[r1 + (k <<
          1)];
      }
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = (float)fabs((double)S[0]);
  a21 = (float)fabs((double)S[3]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if ((float)fabs((double)S[6]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  S[3 * r2] /= S[3 * r1];
  S[3 * r3] /= S[3 * r1];
  S[1 + 3 * r2] -= S[3 * r2] * S[1 + 3 * r1];
  S[1 + 3 * r3] -= S[3 * r3] * S[1 + 3 * r1];
  S[2 + 3 * r2] -= S[3 * r2] * S[2 + 3 * r1];
  S[2 + 3 * r3] -= S[3 * r3] * S[2 + 3 * r1];
  if ((float)fabs((double)S[1 + 3 * r3]) > (float)fabs((double)S[1 + 3 * r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  S[1 + 3 * r3] /= S[1 + 3 * r2];
  S[2 + 3 * r3] -= S[1 + 3 * r3] * S[2 + 3 * r2];
  for (k = 0; k < 2; k++) {
    K[r1 + 3 * k] = b_y[3 * k] / S[3 * r1];
    K[r2 + 3 * k] = b_y[1 + 3 * k] - K[r1 + 3 * k] * S[1 + 3 * r1];
    K[r3 + 3 * k] = b_y[2 + 3 * k] - K[r1 + 3 * k] * S[2 + 3 * r1];
    K[r2 + 3 * k] /= S[1 + 3 * r2];
    K[r3 + 3 * k] -= K[r2 + 3 * k] * S[2 + 3 * r2];
    K[r3 + 3 * k] /= S[2 + 3 * r3];
    K[r2 + 3 * k] -= K[r3 + 3 * k] * S[1 + 3 * r3];
    K[r1 + 3 * k] -= K[r3 + 3 * k] * S[3 * r3];
    K[r1 + 3 * k] -= K[r2 + 3 * k] * S[3 * r2];
  }

  //  Correct using innovation
  // 'VelocityEstimator:137' X_aposteriori = X_apriori + K * (z - z_hat);
  dAcceleration_dqB[0] = 0.0F;
  dAcceleration_dqB[1] = -(dy_2L_apriori - vel_2L_to_ball_correction[1]);
  dAcceleration_dqB[2] = dx_2L_apriori - vel_2L_to_ball_correction[0];
  dAcceleration_dqB[3] = 0.0F;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[1] = -qQEKF[1];
  b_qQEKF[2] = -qQEKF[2];
  b_qQEKF[3] = -qQEKF[3];
  b_qQEKF[4] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[8] = qQEKF[2];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[12] = qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  b_qdotQEKF[0] = y * qQEKF[0];
  b_qdotQEKF[4] = y * -qQEKF[1];
  b_qdotQEKF[8] = y * -qQEKF[2];
  b_qdotQEKF[12] = y * -qQEKF[3];
  b_qdotQEKF[1] = y * qQEKF[1];
  b_qdotQEKF[5] = y * qQEKF[0];
  b_qdotQEKF[9] = y * -qQEKF[3];
  b_qdotQEKF[13] = y * qQEKF[2];
  b_qdotQEKF[2] = y * qQEKF[2];
  b_qdotQEKF[6] = y * qQEKF[3];
  b_qdotQEKF[10] = y * qQEKF[0];
  b_qdotQEKF[14] = y * -qQEKF[1];
  b_qdotQEKF[3] = y * qQEKF[3];
  b_qdotQEKF[7] = y * -qQEKF[2];
  b_qdotQEKF[11] = y * qQEKF[1];
  b_qdotQEKF[15] = y * qQEKF[0];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 4; k++) {
      c_qQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        c_qQEKF[rtemp + (k << 2)] += b_qQEKF[rtemp + (r1 << 2)] * b_qdotQEKF[r1
          + (k << 2)];
      }
    }
  }

  b_qdotQEKF[0] = 2.0F * qQEKF[0];
  b_qdotQEKF[4] = 2.0F * -qQEKF[1];
  b_qdotQEKF[8] = 2.0F * -qQEKF[2];
  b_qdotQEKF[12] = 2.0F * -qQEKF[3];
  b_qdotQEKF[1] = 2.0F * qQEKF[1];
  b_qdotQEKF[5] = 2.0F * qQEKF[0];
  b_qdotQEKF[9] = 2.0F * -qQEKF[3];
  b_qdotQEKF[13] = 2.0F * qQEKF[2];
  b_qdotQEKF[2] = 2.0F * qQEKF[2];
  b_qdotQEKF[6] = 2.0F * qQEKF[3];
  b_qdotQEKF[10] = 2.0F * qQEKF[0];
  b_qdotQEKF[14] = 2.0F * -qQEKF[1];
  b_qdotQEKF[3] = 2.0F * qQEKF[3];
  b_qdotQEKF[7] = 2.0F * -qQEKF[2];
  b_qdotQEKF[11] = 2.0F * qQEKF[1];
  b_qdotQEKF[15] = 2.0F * qQEKF[0];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    dAcceleration_dCOM[rtemp] = 0.0F;
    b_dAcceleration_dCOM[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      dAcceleration_dCOM[rtemp] += dAcceleration_dqB[k] * c_qQEKF[k + (rtemp <<
        2)];
      b_dAcceleration_dCOM[rtemp] += qdotQEKF[k] * b_qdotQEKF[k + (rtemp << 2)];
    }

    fv2[rtemp] = dAcceleration_dCOM[rtemp] - b_dAcceleration_dCOM[rtemp];
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    fv0[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      fv0[rtemp] += fv2[k] * (a * W[k + (rtemp << 2)]);
    }

    vel_2L_to_ball_correction[rtemp] = EncoderDiffMeas[rtemp] - fv0[rtemp];
  }

  b_dx_2L_apriori[0] = dx_2L_apriori;
  b_dx_2L_apriori[1] = dy_2L_apriori;
  for (rtemp = 0; rtemp < 2; rtemp++) {
    acceleration[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      acceleration[rtemp] += vel_2L_to_ball_correction[k] * K[k + 3 * rtemp];
    }

    X_out[rtemp] = b_dx_2L_apriori[rtemp] + acceleration[rtemp];
  }

  // 'VelocityEstimator:138' P_aposteriori = (eye(2) - K*H) * P_apriori;
  for (rtemp = 0; rtemp < 4; rtemp++) {
    I[rtemp] = 0;
  }

  for (k = 0; k < 2; k++) {
    I[k + (k << 1)] = 1;
  }

  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 2; k++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 3; r1++) {
        maxval += (float)H[rtemp + (r1 << 1)] * K[r1 + 3 * k];
      }

      b_dAcceleration_dCOM[rtemp + (k << 1)] = (float)I[rtemp + (k << 1)] -
        maxval;
    }
  }

  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 2; k++) {
      P_out[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        P_out[rtemp + (k << 1)] += P_apriori[rtemp + (r1 << 1)] *
          b_dAcceleration_dCOM[r1 + (k << 1)];
      }
    }
  }

  //     %% Send output to Simulink
  // 'VelocityEstimator:141' X_out = X_aposteriori;
  // 'VelocityEstimator:142' P_out = P_aposteriori;
}

//
// File trailer for VelocityEstimator.cpp
//
// [EOF]
//
