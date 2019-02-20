//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator2.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Feb-2019 12:31:44
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "VelocityEstimator2.h"
#include "SteadyStateAcceleration.h"
#include "SteadyStateAcceleration_dCOM.h"
#include "SteadyStateAcceleration_dq.h"

// Function Definitions

//
// function [X_out, P_out] = VelocityEstimator2(X, P_prev, EncoderDiffMeas, qQEKF, Cov_qQEKF, qdotQEKF, SamplePeriod, TicksPrRev, Jk,Mk,rk,Mb,Jw,rw,l,g,COM,CoR,  Var_COM, eta_encoder)
// for q o p = Phi(q) * p
// Arguments    : const float X[2]
//                const float P_prev[4]
//                const float EncoderDiffMeas[3]
//                const float qQEKF[4]
//                const float Cov_qQEKF[16]
//                const float qdotQEKF[4]
//                float SamplePeriod
//                float TicksPrRev
//                float Jk
//                float Mk
//                float rk
//                float Mb
//                float Jw
//                float rw
//                float l
//                float g
//                const float COM[3]
//                float CoR
//                float Var_COM
//                float eta_encoder
//                float X_out[2]
//                float P_out[4]
// Return Type  : void
//
__attribute__((optimize("O3"))) void VelocityEstimator2(const float X[2], const float P_prev[4], const float
  EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16], const
  float qdotQEKF[4], float SamplePeriod, float TicksPrRev, float Jk, float Mk,
  float rk, float Mb, float Jw, float rw, float l, float g, const float COM[3],
  float CoR, float Var_COM, float eta_encoder, float X_out[2], float P_out[4])
{
  float y;
  float b_y;
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

  float b_Var_COM[4];
  float b_dAcceleration_dCOM[4];
  float b_P_prev[4];
  float W[12];
  static const float fv3[16] = { 0.0F, 0.353553385F, 0.612372458F, -0.707106769F,
    -0.353553385F, 0.0F, 0.707106769F, 0.612372458F, -0.612372458F,
    -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, -0.612372458F,
    0.353553385F, 0.0F };

  static const float fv4[16] = { 0.0F, -0.707106769F, -0.0F, -0.707106769F,
    0.707106769F, 0.0F, 0.707106769F, -0.0F, 0.0F, -0.707106769F, 0.0F,
    0.707106769F, 0.707106769F, 0.0F, -0.707106769F, 0.0F };

  static const float fv5[16] = { 0.0F, 0.353553385F, -0.612372458F,
    -0.707106769F, -0.353553385F, 0.0F, 0.707106769F, -0.612372458F,
    0.612372458F, -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, 0.612372458F,
    0.353553385F, 0.0F };

  float b_dAcceleration_dqB[8];
  float maxval;
  float acceleration[2];
  int r1;
  float b_Cov_qQEKF[8];
  static const signed char iv4[4] = { 1, 0, 0, 1 };

  float P_apriori[4];
  float b_qQEKF[16];
  float b_qdotQEKF[16];
  float c_qdotQEKF[16];
  float c_qQEKF[16];
  float d_qQEKF[16];
  float d_qdotQEKF[16];
  float e_qQEKF[12];
  float vel_2L_to_ball_correction[3];
  float a;
  double H[6];
  float b_a[12];
  float f_qQEKF[12];
  static const signed char iv5[4] = { 0, 0, 1, 0 };

  static const signed char iv6[4] = { 0, -1, 0, 0 };

  float K[6];
  int r2;
  int r3;
  float S[9];
  float c_y[6];
  static const float fv6[9] = { 0.5F, 0.0F, 0.0F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F,
    0.5F };

  float b_X[2];
  signed char I[4];

  // 'VelocityEstimator2:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p 
  // 'VelocityEstimator2:4'               q(2) q(1)  -q(4) q(3);
  // 'VelocityEstimator2:5'               q(3) q(4)  q(1)  -q(2);
  // 'VelocityEstimator2:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'VelocityEstimator2:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q 
  // 'VelocityEstimator2:8'                  p(2) p(1) p(4) -p(3);
  // 'VelocityEstimator2:9'                  p(3) -p(4) p(1) p(2);
  // 'VelocityEstimator2:10'                  p(4) p(3) -p(2) p(1)];
  // 'VelocityEstimator2:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'VelocityEstimator2:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'VelocityEstimator2:14' I_conj = diag([1,-1,-1,-1]);
  // 'VelocityEstimator2:16' dt = SamplePeriod;
  // 'VelocityEstimator2:17' xCOM = COM(1);
  // 'VelocityEstimator2:18' yCOM = COM(2);
  //  motor mapping (inverse kinematics)
  // 'VelocityEstimator2:21' alpha = deg2rad(45);
  // 'VelocityEstimator2:22' gamma = deg2rad(120);
  // 'VelocityEstimator2:24' e1 = [1,0,0]';
  // 'VelocityEstimator2:25' e2 = [0,1,0]';
  // 'VelocityEstimator2:26' e3 = [0,0,1]';
  // 'VelocityEstimator2:27' R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1]; 
  // 'VelocityEstimator2:28' R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0]; 
  // 'VelocityEstimator2:30' W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1); 
  y = rk / rw;

  // 'VelocityEstimator2:31' W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2); 
  b_y = rk / rw;

  // 'VelocityEstimator2:32' W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3); 
  a21 = rk / rw;

  // 'VelocityEstimator2:33' W = [W1;W2;W3];
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
      fv0[rtemp] += fv1[rtemp + 3 * k] * (b_y * (float)iv2[k]);
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
      b_dAcceleration_dCOM[rtemp] += fv4[rtemp + (k << 2)] * dAcceleration_dqB[k];
      b_P_prev[rtemp] += fv5[rtemp + (k << 2)] * dAcceleration_dCOM[k];
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    b_Var_COM[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      b_Var_COM[rtemp] += fv3[rtemp + (k << 2)] * fv2[k];
    }

    W[rtemp] = b_dAcceleration_dCOM[rtemp];
    W[4 + rtemp] = b_P_prev[rtemp];
    W[8 + rtemp] = b_Var_COM[rtemp];
  }

  //  Split state vector, X[k-1], into individual variables
  // dx_2L = X(1);
  // dy_2L = X(2);
  // vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]'; 
  // dx_ball = dx_2L - vel_2L_to_ball_correction(1);
  // dy_ball = dy_2L - vel_2L_to_ball_correction(2);
  // 'VelocityEstimator2:43' dx = X(1);
  // 'VelocityEstimator2:44' dy = X(2);
  //  Process covariances
  // 'VelocityEstimator2:47' dAcceleration_dqB = SteadyStateAcceleration_dq(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dq(COM[0], COM[1], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
    qQEKF[2], qQEKF[3], rk, b_dAcceleration_dqB);

  // 'VelocityEstimator2:48' dAcceleration_dCOM = SteadyStateAcceleration_dCOM(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dCOM(COM[0], COM[1], l, Jk, Mb, Mk, g, qQEKF[0],
    qQEKF[1], qQEKF[2], qQEKF[3], rk, b_dAcceleration_dCOM);

  // 'VelocityEstimator2:50' cov_COM = Var_COM * eye(2);
  // 'VelocityEstimator2:51' cov_velocity = dt^2 * dAcceleration_dqB * Cov_qQEKF * dAcceleration_dqB' + dt^2 * dAcceleration_dCOM * cov_COM * dAcceleration_dCOM'; 
  a21 = SamplePeriod * SamplePeriod;
  maxval = SamplePeriod * SamplePeriod;

  //  Setup covariance matrices
  // 'VelocityEstimator2:54' Q = cov_velocity;
  //  Measurement covariances						
  // 'VelocityEstimator2:57' cov_quantization = 0.5 * eye(3);
  //     %% Prediction step
  // 'VelocityEstimator2:61' X_apriori = zeros(2,1);
  //  Propagate the velocity based on shape-accelerated model for acceleration	
  // 'VelocityEstimator2:64' acceleration = SteadyStateAcceleration(xCOM,yCOM,l,Jk,Jw,Mb,Mk,dx,dy,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  SteadyStateAcceleration(COM[0], COM[1], l, Jk, Jw, Mb, Mk, X[0], X[1], g,
    qQEKF[0], qQEKF[1], qQEKF[2], qQEKF[3], rk, rw, acceleration);

  // 'VelocityEstimator2:65' dx_apriori = dx + dt * acceleration(1);
  // 'VelocityEstimator2:66' dy_apriori = dy + dt * acceleration(2);
  //  Determine model Jacobian (F)
  // 'VelocityEstimator2:69' F_prev = eye(2);
  //  Set apriori state
  // 'VelocityEstimator2:72' X_apriori = [dx_apriori
  // 'VelocityEstimator2:73' 				 dy_apriori];
  //  Calculate apriori covariance of estimate error
  // 'VelocityEstimator2:76' P_apriori = F_prev * P_prev * F_prev' + Q;
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
        b_Cov_qQEKF[rtemp + (k << 2)] += Cov_qQEKF[rtemp + (r1 << 2)] * (a21 *
          b_dAcceleration_dqB[r1 + (k << 2)]);
      }
    }
  }

  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 2; k++) {
      b_Var_COM[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_Var_COM[rtemp + (k << 1)] += Var_COM * (float)iv4[rtemp + (r1 << 1)] *
          (maxval * b_dAcceleration_dCOM[r1 + (k << 1)]);
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
  // 'VelocityEstimator2:79' z = [EncoderDiffMeas];
  //  Encoder Measurement model
  // 'VelocityEstimator2:82' vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,CoR]'; 
  dAcceleration_dqB[0] = 0.0F;
  dAcceleration_dqB[1] = 0.0F;
  dAcceleration_dqB[2] = 0.0F;
  dAcceleration_dqB[3] = CoR;
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
    for (k = 0; k < 4; k++) {
      d_qQEKF[rtemp + (k << 2)] = 0.0F;
      d_qdotQEKF[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        d_qQEKF[rtemp + (k << 2)] += b_qQEKF[rtemp + (r1 << 2)] * b_qdotQEKF[r1
          + (k << 2)];
        d_qdotQEKF[rtemp + (k << 2)] += c_qdotQEKF[rtemp + (r1 << 2)] *
          c_qQEKF[r1 + (k << 2)];
      }
    }
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

  // 'VelocityEstimator2:83' dx_ball_apriori = dx - vel_2L_to_ball_correction(1); 
  // 'VelocityEstimator2:84' dy_ball_apriori = dy - vel_2L_to_ball_correction(2); 
  // 'VelocityEstimator2:86' dpsi_apriori = W * (1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-dy_ball_apriori;dx_ball_apriori;0] - 2*Phi(qQEKF)'*qdotQEKF); 
  y = 1.0F / rk;

  //  InverseKinematics(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dx_ball_apriori,dy_ball_apriori,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'VelocityEstimator2:87' z_encoder_hat = TicksPrRev/(2*pi) * dt * dpsi_apriori; 
  a = TicksPrRev / 6.28318548F * SamplePeriod;

  // 'VelocityEstimator2:88' z_hat = z_encoder_hat;
  //  Measurement Jacobian	
  // 'VelocityEstimator2:91' H = zeros(3,2);
  for (rtemp = 0; rtemp < 6; rtemp++) {
    H[rtemp] = 0.0;
  }

  // 'VelocityEstimator2:92' H(1:3,1) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;0;1;0]; 
  maxval = TicksPrRev / 6.28318548F * SamplePeriod;
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
  // 'VelocityEstimator2:93' H(1:3,2) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-1;0;0]; 
  maxval = TicksPrRev / 6.28318548F * SamplePeriod;
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
  // dEncoder_dqQEKF = OffsetEstimator_dEncoders2L_dq(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dt,dx_2L_apriori,dy_2L_apriori,l,1,TicksPrRev,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // R_encoder = 4*cov_quantization + ...
  //             dEncoder_dqQEKF * Cov_qQEKF * dEncoder_dqQEKF';% + ((n_gear*n_ticksRev)/(2*pi)*dt)^2 * W * vec * cov_omega * devec * W'; 
  // 'VelocityEstimator2:99' R_encoder = eta_encoder * 4*cov_quantization;
  b_y = eta_encoder * 4.0F;

  // 'VelocityEstimator2:100' R = R_encoder;
  //  Calculate Kalman gain
  // 'VelocityEstimator2:103' S = H * P_apriori * H' + R;
  for (rtemp = 0; rtemp < 2; rtemp++) {
    for (k = 0; k < 3; k++) {
      K[rtemp + (k << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        K[rtemp + (k << 1)] += P_apriori[rtemp + (r1 << 1)] * (float)H[r1 + (k <<
          1)];
      }
    }
  }

  // K = P_apriori * H' * inv(S);
  // 'VelocityEstimator2:105' K = P_apriori * H' / S;
  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        maxval += (float)H[r1 + (rtemp << 1)] * K[r1 + (k << 1)];
      }

      S[rtemp + 3 * k] = maxval + b_y * fv6[rtemp + 3 * k];
    }

    for (k = 0; k < 2; k++) {
      c_y[rtemp + 3 * k] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        c_y[rtemp + 3 * k] += (float)H[r1 + (rtemp << 1)] * P_apriori[r1 + (k <<
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
    K[r1 + 3 * k] = c_y[3 * k] / S[3 * r1];
    K[r2 + 3 * k] = c_y[1 + 3 * k] - K[r1 + 3 * k] * S[1 + 3 * r1];
    K[r3 + 3 * k] = c_y[2 + 3 * k] - K[r1 + 3 * k] * S[2 + 3 * r1];
    K[r2 + 3 * k] /= S[1 + 3 * r2];
    K[r3 + 3 * k] -= K[r2 + 3 * k] * S[2 + 3 * r2];
    K[r3 + 3 * k] /= S[2 + 3 * r3];
    K[r2 + 3 * k] -= K[r3 + 3 * k] * S[1 + 3 * r3];
    K[r1 + 3 * k] -= K[r3 + 3 * k] * S[3 * r3];
    K[r1 + 3 * k] -= K[r2 + 3 * k] * S[3 * r2];
  }

  //  Correct using innovation
  // 'VelocityEstimator2:108' X_aposteriori = X_apriori + K * (z - z_hat);
  dAcceleration_dqB[0] = 0.0F;
  dAcceleration_dqB[1] = -(X[1] - vel_2L_to_ball_correction[1]);
  dAcceleration_dqB[2] = X[0] - vel_2L_to_ball_correction[0];
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
      fv0[rtemp] += fv2[k] * W[k + (rtemp << 2)];
    }

    vel_2L_to_ball_correction[rtemp] = EncoderDiffMeas[rtemp] - a * fv0[rtemp];
  }

  b_X[0] = X[0] + SamplePeriod * acceleration[0];
  b_X[1] = X[1] + SamplePeriod * acceleration[1];
  for (rtemp = 0; rtemp < 2; rtemp++) {
    acceleration[rtemp] = 0.0F;
    for (k = 0; k < 3; k++) {
      acceleration[rtemp] += vel_2L_to_ball_correction[k] * K[k + 3 * rtemp];
    }

    X_out[rtemp] = b_X[rtemp] + acceleration[rtemp];
  }

  // 'VelocityEstimator2:109' P_aposteriori = (eye(2) - K*H) * P_apriori;
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
  // 'VelocityEstimator2:112' X_out = X_aposteriori;
  // 'VelocityEstimator2:113' P_out = P_aposteriori;
}

//
// File trailer for VelocityEstimator2.cpp
//
// [EOF]
//
