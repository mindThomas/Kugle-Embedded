//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator_WithCOM.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 05-Mar-2019 16:05:46
//

// Include Files
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "VelocityEstimator_WithCOM.h"
#include "SteadyStateAcceleration_dCOM.h"
#include "SteadyStateAcceleration.h"
#include "SteadyStateAcceleration_dq.h"

// Variable Definitions
static float dpsi_old[3];

// Function Definitions

//
// function [X_out, P_out] = VelocityEstimator_WithCOM(X, P_prev, EncoderDiffMeas, qQEKF, Cov_qQEKF, qdotQEKF, SamplePeriod, TicksPrRev, Jk,Mk,rk,Mb,Jw,rw,l,g,CoR,  Var_COM, eta_encoder, UseTiltForPrediction, EstimateVelocityAtCoR, EnableWheelSlipDetector, WheelSlipAccelerationThreshold, WheelSlipSetVelocityVariance, estimateCOM)
// for q o p = Phi(q) * p
// Arguments    : const float X[4]
//                const float P_prev[16]
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
//                float CoR
//                float Var_COM
//                float eta_encoder
//                boolean_T UseTiltForPrediction
//                boolean_T EstimateVelocityAtCoR
//                boolean_T EnableWheelSlipDetector
//                float WheelSlipAccelerationThreshold
//                float WheelSlipSetVelocityVariance
//                boolean_T estimateCOM
//                float X_out[4]
//                float P_out[16]
// Return Type  : void
//
__attribute__((optimize("O3"))) void VelocityEstimator_WithCOM(const float X[4], const float P_prev[16], const
  float EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16],
  const float qdotQEKF[4], float SamplePeriod, float TicksPrRev, float Jk, float
  Mk, float rk, float Mb, float Jw, float rw, float l, float g, float CoR, float
  Var_COM, float eta_encoder, boolean_T UseTiltForPrediction, boolean_T
  EstimateVelocityAtCoR, boolean_T EnableWheelSlipDetector, float
  WheelSlipAccelerationThreshold, float WheelSlipSetVelocityVariance, boolean_T
  estimateCOM, float X_out[4], float P_out[16])
{
  int i;
  float maxval;
  float y[3];
  float dpsi[3];
  float b_y;
  float a21;
  int i0;
  boolean_T WheelSlip[3];
  float fv0[4];
  float fv1[3];
  int i1;
  static const signed char iv0[12] = { 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  static const float fv2[9] = { 0.0F, 1.0F, 0.0F, -0.866025388F, -0.5F, 0.0F,
    0.866025388F, -0.5F, 0.0F };

  static const signed char iv1[3] = { 1, 0, 0 };

  float cov_COM[4];
  static const signed char iv2[3] = { 0, 1, 0 };

  float fv3[4];
  static const signed char iv3[3] = { 0, 0, 1 };

  float dAcceleration_dqB[4];
  float fv4[4];
  static const float fv5[16] = { 0.0F, -0.707106769F, -0.0F, -0.707106769F,
    0.707106769F, 0.0F, 0.707106769F, -0.0F, 0.0F, -0.707106769F, 0.0F,
    0.707106769F, 0.707106769F, 0.0F, -0.707106769F, 0.0F };

  static const float fv6[16] = { 0.0F, 0.353553385F, -0.612372458F,
    -0.707106769F, -0.353553385F, 0.0F, 0.707106769F, -0.612372458F,
    0.612372458F, -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, 0.612372458F,
    0.353553385F, 0.0F };

  float b_dAcceleration_dqB[8];
  float fv7[4];
  float W[12];
  static const float fv8[16] = { 0.0F, 0.353553385F, 0.612372458F, -0.707106769F,
    -0.353553385F, 0.0F, 0.707106769F, 0.612372458F, -0.612372458F,
    -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, -0.612372458F,
    0.353553385F, 0.0F };

  float dx_apriori;
  float dy_apriori;
  float acceleration[2];
  static double F_prev[16];
  float b_P_prev[16];
  float b_Cov_qQEKF[8];
  float b_F_prev[16];
  float c_dAcceleration_dqB[16];
  float P_apriori[16];
  float dx_ball_apriori;
  float dy_ball_apriori;
  float a;
  float b_qQEKF[16];
  double H[12];
  float c_qQEKF[16];
  float b_a[12];
  float K[12];
  float d_qQEKF[12];
  float b_qdotQEKF[16];
  static const signed char iv4[4] = { 0, 0, 1, 0 };

  static const signed char iv5[4] = { 0, -1, 0, 0 };

  int r1;
  int r2;
  int r3;
  float S[9];
  float c_y[12];
  static const float fv9[9] = { 0.5F, 0.0F, 0.0F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F,
    0.5F };

  static const signed char iv6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  // 'VelocityEstimator_WithCOM:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p 
  // 'VelocityEstimator_WithCOM:4'               q(2) q(1)  -q(4) q(3);
  // 'VelocityEstimator_WithCOM:5'               q(3) q(4)  q(1)  -q(2);
  // 'VelocityEstimator_WithCOM:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'VelocityEstimator_WithCOM:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q 
  // 'VelocityEstimator_WithCOM:8'                  p(2) p(1) p(4) -p(3);
  // 'VelocityEstimator_WithCOM:9'                  p(3) -p(4) p(1) p(2);
  // 'VelocityEstimator_WithCOM:10'                  p(4) p(3) -p(2) p(1)];
  // 'VelocityEstimator_WithCOM:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'VelocityEstimator_WithCOM:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'VelocityEstimator_WithCOM:14' I_conj = diag([1,-1,-1,-1]);
  // 'VelocityEstimator_WithCOM:16' dt = SamplePeriod;
  //  Wheel slip detection
  // 'VelocityEstimator_WithCOM:20' if (isempty(dpsi_old))
  // 'VelocityEstimator_WithCOM:23' dpsi = (2*pi) * (EncoderDiffMeas / dt) / TicksPrRev; 
  // 'VelocityEstimator_WithCOM:24' WheelSlip = ((abs(dpsi - dpsi_old) / dt) > WheelSlipAccelerationThreshold); 
  for (i = 0; i < 3; i++) {
    maxval = 6.28318548F * (EncoderDiffMeas[i] / SamplePeriod) / TicksPrRev;
    dpsi_old[i] = maxval - dpsi_old[i];
    dpsi[i] = maxval;
  }

  for (i = 0; i < 3; i++) {
    y[i] = (float)fabs((double)dpsi_old[i]);
  }

  // 'VelocityEstimator_WithCOM:25' dpsi_old = dpsi;
  // 'VelocityEstimator_WithCOM:27' Wheel1Slip = WheelSlip(1);
  // 'VelocityEstimator_WithCOM:28' Wheel2Slip = WheelSlip(2);
  // 'VelocityEstimator_WithCOM:29' Wheel3Slip = WheelSlip(3);
  //      if (Wheel1Slip || Wheel2Slip || Wheel3Slip)
  //          UseTiltForPrediction = false;
  //      end
  //  motor mapping (inverse kinematics)
  // 'VelocityEstimator_WithCOM:36' alpha = deg2rad(45);
  // 'VelocityEstimator_WithCOM:37' gamma = deg2rad(120);
  // 'VelocityEstimator_WithCOM:39' e1 = [1,0,0]';
  // 'VelocityEstimator_WithCOM:40' e2 = [0,1,0]';
  // 'VelocityEstimator_WithCOM:41' e3 = [0,0,1]';
  // 'VelocityEstimator_WithCOM:42' R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1]; 
  // 'VelocityEstimator_WithCOM:43' R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0]; 
  // 'VelocityEstimator_WithCOM:45' W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1); 
  b_y = rk / rw;

  // 'VelocityEstimator_WithCOM:46' W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2); 
  a21 = rk / rw;

  // 'VelocityEstimator_WithCOM:47' W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3); 
  maxval = rk / rw;

  // 'VelocityEstimator_WithCOM:48' W = [W1;W2;W3];
  for (i = 0; i < 3; i++) {
    WheelSlip[i] = (y[i] / SamplePeriod > WheelSlipAccelerationThreshold);
    dpsi_old[i] = dpsi[i];
    fv1[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      fv1[i] += fv2[i + 3 * i0] * (b_y * (float)iv1[i0]);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    fv0[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      fv0[i0] += (float)iv0[i0 + (i1 << 2)] * fv1[i1];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv1[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      fv1[i0] += fv2[i0 + 3 * i1] * (a21 * (float)iv2[i1]);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    cov_COM[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      cov_COM[i0] += (float)iv0[i0 + (i1 << 2)] * fv1[i1];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv1[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      fv1[i0] += fv2[i0 + 3 * i1] * (maxval * (float)iv3[i1]);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    fv3[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      fv3[i0] += (float)iv0[i0 + (i1 << 2)] * fv1[i1];
    }

    dAcceleration_dqB[i0] = 0.0F;
    fv4[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      dAcceleration_dqB[i0] += fv5[i0 + (i1 << 2)] * fv0[i1];
      fv4[i0] += fv6[i0 + (i1 << 2)] * cov_COM[i1];
    }
  }

  //  Split state vector, X[k-1], into individual variables
  // dx_2L = X(1);
  // dy_2L = X(2);
  // vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]'; 
  // dx_ball = dx_2L - vel_2L_to_ball_correction(1);
  // dy_ball = dy_2L - vel_2L_to_ball_correction(2);
  // 'VelocityEstimator_WithCOM:58' dx = X(1);
  // 'VelocityEstimator_WithCOM:59' dy = X(2);
  // 'VelocityEstimator_WithCOM:61' xCOM = X(3);
  // 'VelocityEstimator_WithCOM:62' yCOM = X(4);
  //  Process covariances
  // 'VelocityEstimator_WithCOM:65' dAcceleration_dqB = SteadyStateAcceleration_dq(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dq(X[2], X[3], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
    qQEKF[2], qQEKF[3], rk, b_dAcceleration_dqB);

  // 'VelocityEstimator_WithCOM:66' dAcceleration_dCOM = SteadyStateAcceleration_dCOM(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  // 'VelocityEstimator_WithCOM:68' cov_COM = single(zeros(2));
  for (i0 = 0; i0 < 4; i0++) {
    fv7[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      fv7[i0] += fv8[i0 + (i1 << 2)] * fv3[i1];
    }

    W[i0] = dAcceleration_dqB[i0];
    W[4 + i0] = fv4[i0];
    W[8 + i0] = fv7[i0];
    cov_COM[i0] = 0.0F;
  }

  // 'VelocityEstimator_WithCOM:69' if (estimateCOM)
  if (estimateCOM) {
    // 'VelocityEstimator_WithCOM:70' cov_COM(1,1) = Var_COM;
    cov_COM[0] = Var_COM;

    // 'VelocityEstimator_WithCOM:71' cov_COM(2,2) = Var_COM;
    cov_COM[3] = Var_COM;
  }

  // 'VelocityEstimator_WithCOM:73' cov_velocity = dt^2 * dAcceleration_dqB * Cov_qQEKF * dAcceleration_dqB'; 
  maxval = SamplePeriod * SamplePeriod;

  //  Setup covariance matrices
  // 'VelocityEstimator_WithCOM:76' Q = [cov_velocity, zeros(2,2);
  // 'VelocityEstimator_WithCOM:77'          zeros(2,2), cov_COM];
  //      if (Wheel1Slip || Wheel2Slip || Wheel3Slip)
  //          Q = 0*Q;
  //      end
  //  Measurement covariances						
  // 'VelocityEstimator_WithCOM:83' cov_quantization = 0.5 * eye(3);
  //     %% Prediction step
  // 'VelocityEstimator_WithCOM:87' X_apriori = zeros(2,1);
  //  Propagate the velocity based on shape-accelerated model for acceleration	
  // 'VelocityEstimator_WithCOM:90' dx_apriori = dx;
  dx_apriori = X[0];

  // 'VelocityEstimator_WithCOM:91' dy_apriori = dy;
  dy_apriori = X[1];

  // 'VelocityEstimator_WithCOM:93' acceleration = SteadyStateAcceleration(xCOM,yCOM,l,Jk,Jw,Mb,Mk,dx,dy,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  SteadyStateAcceleration(X[2], X[3], l, Jk, Jw, Mb, Mk, X[0], X[1], g, qQEKF[0],
    qQEKF[1], qQEKF[2], qQEKF[3], rk, rw, acceleration);

  // 'VelocityEstimator_WithCOM:94' if (UseTiltForPrediction && estimateCOM)
  if (UseTiltForPrediction && estimateCOM) {
    // 'VelocityEstimator_WithCOM:95' dx_apriori = dx_apriori + dt * acceleration(1); 
    dx_apriori = X[0] + SamplePeriod * acceleration[0];

    // 'VelocityEstimator_WithCOM:96' dy_apriori = dy_apriori + dt * acceleration(2); 
    dy_apriori = X[1] + SamplePeriod * acceleration[1];
  }

  // 'VelocityEstimator_WithCOM:99' xCOM_apriori = xCOM;
  // 'VelocityEstimator_WithCOM:100' yCOM_apriori = yCOM;
  //  Determine model Jacobian (F)
  // 'VelocityEstimator_WithCOM:103' F_prev = eye(4);
  memset(&F_prev[0], 0, sizeof(double) << 4);
  for (i = 0; i < 4; i++) {
    F_prev[i + (i << 2)] = 1.0;
  }

  // 'VelocityEstimator_WithCOM:104' if (estimateCOM)
  if (estimateCOM) {
    // 'VelocityEstimator_WithCOM:105' F_prev(1:2,3:4) = dt * dAcceleration_dCOM; 
    SteadyStateAcceleration_dCOM(X[2], X[3], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
      qQEKF[2], qQEKF[3], rk, fv0);
    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        F_prev[(i1 + (i0 << 2)) + 2] = SamplePeriod * fv0[i1 + (i0 << 1)];
      }
    }
  }

  //  Set apriori state
  // 'VelocityEstimator_WithCOM:109' X_apriori = [dx_apriori
  // 'VelocityEstimator_WithCOM:110' 				 dy_apriori;
  // 'VelocityEstimator_WithCOM:111'                  xCOM_apriori;
  // 'VelocityEstimator_WithCOM:112'                  yCOM_apriori];
  //  Calculate apriori covariance of estimate error
  // 'VelocityEstimator_WithCOM:115' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      b_P_prev[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        b_P_prev[i0 + (i1 << 2)] += P_prev[i0 + (i << 2)] * (float)F_prev[i +
          (i1 << 2)];
      }
    }

    for (i1 = 0; i1 < 2; i1++) {
      b_Cov_qQEKF[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        b_Cov_qQEKF[i0 + (i1 << 2)] += Cov_qQEKF[i0 + (i << 2)] * (maxval *
          b_dAcceleration_dqB[i + (i1 << 2)]);
      }
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 2; i1++) {
      dAcceleration_dqB[i0 + (i1 << 1)] = 0.0F;
      for (i = 0; i < 4; i++) {
        dAcceleration_dqB[i0 + (i1 << 1)] += b_dAcceleration_dqB[i + (i0 << 2)] *
          b_Cov_qQEKF[i + (i1 << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      b_F_prev[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        b_F_prev[i0 + (i1 << 2)] += (float)F_prev[i + (i0 << 2)] * b_P_prev[i +
          (i1 << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 2; i1++) {
      c_dAcceleration_dqB[i1 + (i0 << 2)] = dAcceleration_dqB[i1 + (i0 << 1)];
      c_dAcceleration_dqB[(i1 + (i0 << 2)) + 2] = 0.0F;
      c_dAcceleration_dqB[i1 + ((i0 + 2) << 2)] = 0.0F;
      c_dAcceleration_dqB[(i1 + ((i0 + 2) << 2)) + 2] = cov_COM[i1 + (i0 << 1)];
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      P_apriori[i1 + (i0 << 2)] = b_F_prev[i1 + (i0 << 2)] +
        c_dAcceleration_dqB[i1 + (i0 << 2)];
    }
  }

  //     %% Update/correction step
  // 'VelocityEstimator_WithCOM:118' z = [EncoderDiffMeas];
  //      if (Wheel1Slip)
  //          z(1) = TicksPrRev/(2*pi) * dt * dpsi_old(1);
  //      end
  //      if (Wheel2Slip)
  //          z(2) = TicksPrRev/(2*pi) * dt * dpsi_old(2);
  //      end
  //      if (Wheel3Slip)
  //          z(3) = TicksPrRev/(2*pi) * dt * dpsi_old(3);
  //      end
  //  Encoder Measurement model
  // 'VelocityEstimator_WithCOM:131' dx_ball_apriori = dx_apriori;
  dx_ball_apriori = dx_apriori;

  // 'VelocityEstimator_WithCOM:132' dy_ball_apriori = dy_apriori;
  dy_ball_apriori = dy_apriori;

  // 'VelocityEstimator_WithCOM:134' if (EstimateVelocityAtCoR)
  if (EstimateVelocityAtCoR) {
    // 'VelocityEstimator_WithCOM:135' vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,CoR]'; 
    fv0[0] = 0.0F;
    fv0[1] = 0.0F;
    fv0[2] = 0.0F;
    fv0[3] = CoR;
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
    b_P_prev[0] = qdotQEKF[0];
    b_P_prev[1] = -qdotQEKF[1];
    b_P_prev[2] = -qdotQEKF[2];
    b_P_prev[3] = -qdotQEKF[3];
    b_P_prev[4] = qdotQEKF[1];
    b_P_prev[5] = qdotQEKF[0];
    b_P_prev[6] = -qdotQEKF[3];
    b_P_prev[7] = qdotQEKF[2];
    b_P_prev[8] = qdotQEKF[2];
    b_P_prev[9] = qdotQEKF[3];
    b_P_prev[10] = qdotQEKF[0];
    b_P_prev[11] = -qdotQEKF[1];
    b_P_prev[12] = qdotQEKF[3];
    b_P_prev[13] = -qdotQEKF[2];
    b_P_prev[14] = qdotQEKF[1];
    b_P_prev[15] = qdotQEKF[0];
    b_F_prev[0] = qdotQEKF[0];
    b_F_prev[4] = -qdotQEKF[1];
    b_F_prev[8] = -qdotQEKF[2];
    b_F_prev[12] = -qdotQEKF[3];
    b_F_prev[1] = qdotQEKF[1];
    b_F_prev[5] = qdotQEKF[0];
    b_F_prev[9] = qdotQEKF[3];
    b_F_prev[13] = -qdotQEKF[2];
    b_F_prev[2] = qdotQEKF[2];
    b_F_prev[6] = -qdotQEKF[3];
    b_F_prev[10] = qdotQEKF[0];
    b_F_prev[14] = qdotQEKF[1];
    b_F_prev[3] = qdotQEKF[3];
    b_F_prev[7] = qdotQEKF[2];
    b_F_prev[11] = -qdotQEKF[1];
    b_F_prev[15] = qdotQEKF[0];
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
    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        c_dAcceleration_dqB[i0 + (i1 << 2)] = 0.0F;
        b_qdotQEKF[i0 + (i1 << 2)] = 0.0F;
        for (i = 0; i < 4; i++) {
          c_dAcceleration_dqB[i0 + (i1 << 2)] += b_qQEKF[i0 + (i << 2)] *
            b_P_prev[i + (i1 << 2)];
          b_qdotQEKF[i0 + (i1 << 2)] += b_F_prev[i0 + (i << 2)] * c_qQEKF[i +
            (i1 << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        b_qQEKF[i1 + (i0 << 2)] = c_dAcceleration_dqB[i1 + (i0 << 2)] +
          b_qdotQEKF[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        K[i0 + (i1 << 2)] = 0.0F;
        for (i = 0; i < 4; i++) {
          K[i0 + (i1 << 2)] += b_qQEKF[i0 + (i << 2)] * (float)iv0[i + (i1 << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      fv1[i0] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        fv1[i0] += fv0[i1] * K[i1 + (i0 << 2)];
      }

      dpsi[i0] = fv1[i0];
    }

    // 'VelocityEstimator_WithCOM:136' dx_ball_apriori = dx_ball_apriori - vel_2L_to_ball_correction(1); 
    dx_ball_apriori = dx_apriori - dpsi[0];

    // 'VelocityEstimator_WithCOM:137' dy_ball_apriori = dy_ball_apriori - vel_2L_to_ball_correction(2); 
    dy_ball_apriori = dy_apriori - dpsi[1];
  }

  // 'VelocityEstimator_WithCOM:140' dpsi_apriori = W * (1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-dy_ball_apriori;dx_ball_apriori;0] - 2*Phi(qQEKF)'*qdotQEKF); 
  b_y = 1.0F / rk;

  //  InverseKinematics(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dx_ball_apriori,dy_ball_apriori,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'VelocityEstimator_WithCOM:141' z_encoder_hat = TicksPrRev/(2*pi) * dt * dpsi_apriori; 
  a = TicksPrRev / 6.28318548F * SamplePeriod;

  // 'VelocityEstimator_WithCOM:142' z_hat = z_encoder_hat;
  //  Measurement Jacobian	
  // 'VelocityEstimator_WithCOM:145' H = zeros(3,4);
  memset(&H[0], 0, 12U * sizeof(double));

  // 'VelocityEstimator_WithCOM:146' H(1:3,1) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;0;1;0]; 
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
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      b_a[i1 + (i0 << 2)] = maxval * W[i1 + (i0 << 2)] / rk;
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
  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      K[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        K[i0 + (i1 << 2)] += b_qQEKF[i0 + (i << 2)] * b_a[i + (i1 << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      d_qQEKF[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        d_qQEKF[i0 + (i1 << 2)] += c_qQEKF[i0 + (i << 2)] * K[i + (i1 << 2)];
      }
    }
  }

  //  d encoder_meas  /  d dx_2L
  // 'VelocityEstimator_WithCOM:147' H(1:3,2) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-1;0;0]; 
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
  for (i0 = 0; i0 < 3; i0++) {
    fv1[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      b_a[i1 + (i0 << 2)] = maxval * W[i1 + (i0 << 2)] / rk;
      fv1[i0] += (float)iv4[i1] * d_qQEKF[i1 + (i0 << 2)];
    }

    H[i0 << 2] = fv1[i0];
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
  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      K[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        K[i0 + (i1 << 2)] += b_qQEKF[i0 + (i << 2)] * b_a[i + (i1 << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      d_qQEKF[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        d_qQEKF[i0 + (i1 << 2)] += c_qQEKF[i0 + (i << 2)] * K[i + (i1 << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv1[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      fv1[i0] += (float)iv5[i1] * d_qQEKF[i1 + (i0 << 2)];
    }

    H[1 + (i0 << 2)] = fv1[i0];
  }

  //  d encoder_meas  /  d dy_2L
  //  Calculate measurement covariance
  // dEncoder_dqQEKF = OffsetEstimator_dEncoders2L_dq(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dt,dx_2L_apriori,dy_2L_apriori,l,1,TicksPrRev,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // R_encoder = 4*cov_quantization + ...
  //             dEncoder_dqQEKF * Cov_qQEKF * dEncoder_dqQEKF';% + ((n_gear*n_ticksRev)/(2*pi)*dt)^2 * W * vec * cov_omega * devec * W'; 
  // 'VelocityEstimator_WithCOM:153' R_encoder = eta_encoder * 4*cov_quantization; 
  a21 = eta_encoder * 4.0F;

  // 'VelocityEstimator_WithCOM:154' R = R_encoder;
  //      WheelSlipCovarianceIncreaseFactor = 1;
  //      WheelSlipMapping = [];
  //      if (Wheel1Slip)
  //          R(1,1) = WheelSlipCovarianceIncreaseFactor*R(1,1);
  //      else
  //          WheelSlipMapping = [WheelSlipMapping; [1, 0, 0]];
  //      end
  //      if (Wheel2Slip)
  //          R(2,2) = WheelSlipCovarianceIncreaseFactor*R(2,2);
  //      else
  //          WheelSlipMapping = [WheelSlipMapping; [0, 1, 0]];
  //      end
  //      if (Wheel3Slip)
  //          R(3,3) = WheelSlipCovarianceIncreaseFactor*R(3,3);
  //      else
  //          WheelSlipMapping = [WheelSlipMapping; [0, 0, 1]];
  //      end
  //      WheelSlipMapping = eye(3);
  //
  //      if (Wheel1Slip || Wheel2Slip || Wheel3Slip)
  //          R = WheelSlipCovarianceIncreaseFactor * R;
  //      end
  //      if (~isempty(WheelSlipMapping)) % perform correction
  //          z = WheelSlipMapping * z;
  //          z_hat = WheelSlipMapping * z_hat;
  //          H = WheelSlipMapping * H;
  //          R = WheelSlipMapping * R * WheelSlipMapping';
  //  Calculate Kalman gain
  // 'VelocityEstimator_WithCOM:186' S = H * P_apriori * H' + R;
  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      K[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        K[i0 + (i1 << 2)] += P_apriori[i0 + (i << 2)] * (float)H[i + (i1 << 2)];
      }
    }
  }

  // K = P_apriori * H' * inv(S);
  // 'VelocityEstimator_WithCOM:188' K = P_apriori * H' / S;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      maxval = 0.0F;
      for (i = 0; i < 4; i++) {
        maxval += (float)H[i + (i0 << 2)] * K[i + (i1 << 2)];
      }

      S[i0 + 3 * i1] = maxval + a21 * fv9[i0 + 3 * i1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      c_y[i0 + 3 * i1] = 0.0F;
      for (i = 0; i < 4; i++) {
        c_y[i0 + 3 * i1] += (float)H[i + (i0 << 2)] * P_apriori[i + (i1 << 2)];
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
    i = r2;
    r2 = r3;
    r3 = i;
  }

  S[1 + 3 * r3] /= S[1 + 3 * r2];
  S[2 + 3 * r3] -= S[1 + 3 * r3] * S[2 + 3 * r2];

  //  Correct using innovation
  // 'VelocityEstimator_WithCOM:191' X_aposteriori = X_apriori + K * (z - z_hat); 
  fv0[0] = 0.0F;
  fv0[1] = -dy_ball_apriori;
  fv0[2] = dx_ball_apriori;
  fv0[3] = 0.0F;
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
  b_P_prev[0] = b_y * qQEKF[0];
  b_P_prev[4] = b_y * -qQEKF[1];
  b_P_prev[8] = b_y * -qQEKF[2];
  b_P_prev[12] = b_y * -qQEKF[3];
  b_P_prev[1] = b_y * qQEKF[1];
  b_P_prev[5] = b_y * qQEKF[0];
  b_P_prev[9] = b_y * -qQEKF[3];
  b_P_prev[13] = b_y * qQEKF[2];
  b_P_prev[2] = b_y * qQEKF[2];
  b_P_prev[6] = b_y * qQEKF[3];
  b_P_prev[10] = b_y * qQEKF[0];
  b_P_prev[14] = b_y * -qQEKF[1];
  b_P_prev[3] = b_y * qQEKF[3];
  b_P_prev[7] = b_y * -qQEKF[2];
  b_P_prev[11] = b_y * qQEKF[1];
  b_P_prev[15] = b_y * qQEKF[0];
  for (i = 0; i < 4; i++) {
    K[r1 + 3 * i] = c_y[3 * i] / S[3 * r1];
    K[r2 + 3 * i] = c_y[1 + 3 * i] - K[r1 + 3 * i] * S[1 + 3 * r1];
    K[r3 + 3 * i] = c_y[2 + 3 * i] - K[r1 + 3 * i] * S[2 + 3 * r1];
    K[r2 + 3 * i] /= S[1 + 3 * r2];
    K[r3 + 3 * i] -= K[r2 + 3 * i] * S[2 + 3 * r2];
    K[r3 + 3 * i] /= S[2 + 3 * r3];
    K[r2 + 3 * i] -= K[r3 + 3 * i] * S[1 + 3 * r3];
    K[r1 + 3 * i] -= K[r3 + 3 * i] * S[3 * r3];
    K[r1 + 3 * i] -= K[r2 + 3 * i] * S[3 * r2];
    for (i0 = 0; i0 < 4; i0++) {
      c_qQEKF[i + (i0 << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        c_qQEKF[i + (i0 << 2)] += b_qQEKF[i + (i1 << 2)] * b_P_prev[i1 + (i0 <<
          2)];
      }
    }
  }

  b_P_prev[0] = 2.0F * qQEKF[0];
  b_P_prev[4] = 2.0F * -qQEKF[1];
  b_P_prev[8] = 2.0F * -qQEKF[2];
  b_P_prev[12] = 2.0F * -qQEKF[3];
  b_P_prev[1] = 2.0F * qQEKF[1];
  b_P_prev[5] = 2.0F * qQEKF[0];
  b_P_prev[9] = 2.0F * -qQEKF[3];
  b_P_prev[13] = 2.0F * qQEKF[2];
  b_P_prev[2] = 2.0F * qQEKF[2];
  b_P_prev[6] = 2.0F * qQEKF[3];
  b_P_prev[10] = 2.0F * qQEKF[0];
  b_P_prev[14] = 2.0F * -qQEKF[1];
  b_P_prev[3] = 2.0F * qQEKF[3];
  b_P_prev[7] = 2.0F * -qQEKF[2];
  b_P_prev[11] = 2.0F * qQEKF[1];
  b_P_prev[15] = 2.0F * qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    cov_COM[i0] = 0.0F;
    dAcceleration_dqB[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      cov_COM[i0] += fv0[i1] * c_qQEKF[i1 + (i0 << 2)];
      dAcceleration_dqB[i0] += qdotQEKF[i1] * b_P_prev[i1 + (i0 << 2)];
    }

    fv3[i0] = cov_COM[i0] - dAcceleration_dqB[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv1[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      fv1[i0] += fv3[i1] * W[i1 + (i0 << 2)];
    }

    dpsi[i0] = EncoderDiffMeas[i0] - a * fv1[i0];
  }

  cov_COM[0] = dx_apriori;
  cov_COM[1] = dy_apriori;
  cov_COM[2] = X[2];
  cov_COM[3] = X[3];
  for (i0 = 0; i0 < 4; i0++) {
    dAcceleration_dqB[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      dAcceleration_dqB[i0] += dpsi[i1] * K[i1 + 3 * i0];
    }

    X_out[i0] = cov_COM[i0] + dAcceleration_dqB[i0];
  }

  // 'VelocityEstimator_WithCOM:192' P_aposteriori = (eye(4) - K*H) * P_apriori; 
  memset(&F_prev[0], 0, sizeof(double) << 4);
  for (i = 0; i < 4; i++) {
    F_prev[i + (i << 2)] = 1.0;
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      maxval = 0.0F;
      for (i = 0; i < 3; i++) {
        maxval += (float)H[i0 + (i << 2)] * K[i + 3 * i1];
      }

      b_F_prev[i0 + (i1 << 2)] = (float)F_prev[i0 + (i1 << 2)] - maxval;
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      P_out[i0 + (i1 << 2)] = 0.0F;
      for (i = 0; i < 4; i++) {
        P_out[i0 + (i1 << 2)] += P_apriori[i0 + (i << 2)] * b_F_prev[i + (i1 <<
          2)];
      }
    }
  }

  //      else
  //          % no valid sensor data to perform correction on
  //          X_aposteriori = X_apriori;
  //          P_aposteriori = P_apriori;
  //      end
  //     %% Send output to Simulink
  // 'VelocityEstimator_WithCOM:200' X_out = X_aposteriori;
  // 'VelocityEstimator_WithCOM:201' P_out = P_aposteriori;
  // 'VelocityEstimator_WithCOM:203' if (EnableWheelSlipDetector && (Wheel1Slip || Wheel2Slip || Wheel3Slip)) 
  if (EnableWheelSlipDetector && (WheelSlip[0] || WheelSlip[1] || WheelSlip[2]))
  {
    // 'VelocityEstimator_WithCOM:204' X_out = X;
    for (i = 0; i < 4; i++) {
      X_out[i] = X[i];
    }

    // 'VelocityEstimator_WithCOM:205' P_out = eye(4) * WheelSlipSetVelocityVariance; 
    for (i0 = 0; i0 < 16; i0++) {
      P_out[i0] = (float)iv6[i0] * WheelSlipSetVelocityVariance;
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void VelocityEstimator_WithCOM_init()
{
  int i;

  // 'VelocityEstimator_WithCOM:21' dpsi_old = single([0;0;0]);
  for (i = 0; i < 3; i++) {
    dpsi_old[i] = 0.0F;
  }
}

//
// File trailer for VelocityEstimator_WithCOM.cpp
//
// [EOF]
//
