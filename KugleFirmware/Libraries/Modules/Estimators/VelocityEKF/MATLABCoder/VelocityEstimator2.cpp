//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator2.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-Mar-2019 12:57:31
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "VelocityEstimator2.h"
#include "SteadyStateAcceleration.h"
#include "SteadyStateAcceleration_dCOM.h"
#include "SteadyStateAcceleration_dq.h"

// Variable Definitions
static float dpsi_old[3];

// Function Definitions

//
// function [X_out, P_out] = VelocityEstimator2(X, P_prev, EncoderDiffMeas, qQEKF, Cov_qQEKF, qdotQEKF, SamplePeriod, TicksPrRev, Jk,Mk,rk,Mb,Jw,rw,l,g,COM,CoR,  Var_COM, eta_encoder, UseTiltForPrediction, EstimateVelocityAtCoR, EnableWheelSlipDetector, WheelSlipAccelerationThreshold, WheelSlipSetVelocityVariance)
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
//                boolean_T UseTiltForPrediction
//                boolean_T EstimateVelocityAtCoR
//                boolean_T EnableWheelSlipDetector
//                float WheelSlipAccelerationThreshold
//                float WheelSlipSetVelocityVariance
//                float X_out[2]
//                float P_out[4]
// Return Type  : void
//
__attribute__((optimize("O3"))) void VelocityEstimator2(const float X[2], const float P_prev[4], const float
  EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16], const
  float qdotQEKF[4], float SamplePeriod, float TicksPrRev, float Jk, float Mk,
  float rk, float Mb, float Jw, float rw, float l, float g, const float COM[3],
  float CoR, float Var_COM, float eta_encoder, boolean_T UseTiltForPrediction,
  boolean_T EstimateVelocityAtCoR, boolean_T EnableWheelSlipDetector, float
  WheelSlipAccelerationThreshold, float WheelSlipSetVelocityVariance, float
  X_out[2], float P_out[4])
{
  int i;
  float maxval;
  float y[3];
  float dpsi[3];
  float b_y;
  float a21;
  int r2;
  boolean_T WheelSlip[3];
  float dAcceleration_dqB[4];
  float fv0[3];
  static const signed char iv0[12] = { 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  static const float fv1[9] = { 0.0F, 1.0F, 0.0F, -0.866025388F, -0.5F, 0.0F,
    0.866025388F, -0.5F, 0.0F };

  static const signed char iv1[3] = { 1, 0, 0 };

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
  float dx_apriori;
  float dy_apriori;
  float acceleration[2];
  int r1;
  float b_Cov_qQEKF[8];
  static const signed char iv4[4] = { 1, 0, 0, 1 };

  float P_apriori[4];
  float dx_ball_apriori;
  float dy_ball_apriori;
  float a;
  float b_qQEKF[16];
  double H[6];
  float b_qdotQEKF[16];
  float c_qQEKF[16];
  float b_a[12];
  float c_qdotQEKF[16];
  float d_qQEKF[12];
  float e_qQEKF[12];
  float f_qQEKF[16];
  float d_qdotQEKF[16];
  static const signed char iv5[4] = { 0, 0, 1, 0 };

  static const signed char iv6[4] = { 0, -1, 0, 0 };

  float K[6];
  int r3;
  float S[9];
  float c_y[6];
  static const float fv6[9] = { 0.5F, 0.0F, 0.0F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F,
    0.5F };

  float b_dx_apriori[2];
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
  //  Wheel slip detection
  // 'VelocityEstimator2:22' if (isempty(dpsi_old))
  // 'VelocityEstimator2:25' dpsi = (2*pi) * (EncoderDiffMeas / dt) / TicksPrRev; 
  // 'VelocityEstimator2:26' WheelSlip = ((abs(dpsi - dpsi_old) / dt) > WheelSlipAccelerationThreshold); 
  for (i = 0; i < 3; i++) {
    maxval = 6.28318548F * (EncoderDiffMeas[i] / SamplePeriod) / TicksPrRev;
    dpsi_old[i] = maxval - dpsi_old[i];
    dpsi[i] = maxval;
  }

  for (i = 0; i < 3; i++) {
    y[i] = (float)fabs((double)dpsi_old[i]);
  }

  // 'VelocityEstimator2:27' dpsi_old = dpsi;
  // 'VelocityEstimator2:29' Wheel1Slip = WheelSlip(1);
  // 'VelocityEstimator2:30' Wheel2Slip = WheelSlip(2);
  // 'VelocityEstimator2:31' Wheel3Slip = WheelSlip(3);
  //      if (Wheel1Slip || Wheel2Slip || Wheel3Slip)
  //          UseTiltForPrediction = false;
  //      end
  //  motor mapping (inverse kinematics)
  // 'VelocityEstimator2:38' alpha = deg2rad(45);
  // 'VelocityEstimator2:39' gamma = deg2rad(120);
  // 'VelocityEstimator2:41' e1 = [1,0,0]';
  // 'VelocityEstimator2:42' e2 = [0,1,0]';
  // 'VelocityEstimator2:43' e3 = [0,0,1]';
  // 'VelocityEstimator2:44' R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1]; 
  // 'VelocityEstimator2:45' R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0]; 
  // 'VelocityEstimator2:47' W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1); 
  b_y = rk / rw;

  // 'VelocityEstimator2:48' W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2); 
  a21 = rk / rw;

  // 'VelocityEstimator2:49' W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3); 
  maxval = rk / rw;

  // 'VelocityEstimator2:50' W = [W1;W2;W3];
  for (i = 0; i < 3; i++) {
    WheelSlip[i] = (y[i] / SamplePeriod > WheelSlipAccelerationThreshold);
    dpsi_old[i] = dpsi[i];
    fv0[i] = 0.0F;
    for (r2 = 0; r2 < 3; r2++) {
      fv0[i] += fv1[i + 3 * r2] * (b_y * (float)iv1[r2]);
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    dAcceleration_dqB[r2] = 0.0F;
    for (i = 0; i < 3; i++) {
      dAcceleration_dqB[r2] += (float)iv0[r2 + (i << 2)] * fv0[i];
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    fv0[r2] = 0.0F;
    for (i = 0; i < 3; i++) {
      fv0[r2] += fv1[r2 + 3 * i] * (a21 * (float)iv2[i]);
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    dAcceleration_dCOM[r2] = 0.0F;
    for (i = 0; i < 3; i++) {
      dAcceleration_dCOM[r2] += (float)iv0[r2 + (i << 2)] * fv0[i];
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    fv0[r2] = 0.0F;
    for (i = 0; i < 3; i++) {
      fv0[r2] += fv1[r2 + 3 * i] * (maxval * (float)iv3[i]);
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    fv2[r2] = 0.0F;
    for (i = 0; i < 3; i++) {
      fv2[r2] += (float)iv0[r2 + (i << 2)] * fv0[i];
    }

    b_dAcceleration_dCOM[r2] = 0.0F;
    b_P_prev[r2] = 0.0F;
    for (i = 0; i < 4; i++) {
      b_dAcceleration_dCOM[r2] += fv4[r2 + (i << 2)] * dAcceleration_dqB[i];
      b_P_prev[r2] += fv5[r2 + (i << 2)] * dAcceleration_dCOM[i];
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    b_Var_COM[r2] = 0.0F;
    for (i = 0; i < 4; i++) {
      b_Var_COM[r2] += fv3[r2 + (i << 2)] * fv2[i];
    }

    W[r2] = b_dAcceleration_dCOM[r2];
    W[4 + r2] = b_P_prev[r2];
    W[8 + r2] = b_Var_COM[r2];
  }

  //  Split state vector, X[k-1], into individual variables
  // dx_2L = X(1);
  // dy_2L = X(2);
  // vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,2*l]'; 
  // dx_ball = dx_2L - vel_2L_to_ball_correction(1);
  // dy_ball = dy_2L - vel_2L_to_ball_correction(2);
  // 'VelocityEstimator2:60' dx = X(1);
  // 'VelocityEstimator2:61' dy = X(2);
  //  Process covariances
  // 'VelocityEstimator2:64' dAcceleration_dqB = SteadyStateAcceleration_dq(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dq(COM[0], COM[1], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
    qQEKF[2], qQEKF[3], rk, b_dAcceleration_dqB);

  // 'VelocityEstimator2:65' dAcceleration_dCOM = SteadyStateAcceleration_dCOM(xCOM,yCOM,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dCOM(COM[0], COM[1], l, Jk, Mb, Mk, g, qQEKF[0],
    qQEKF[1], qQEKF[2], qQEKF[3], rk, b_dAcceleration_dCOM);

  // 'VelocityEstimator2:67' cov_COM = Var_COM * eye(2);
  // 'VelocityEstimator2:68' cov_velocity = dt^2 * dAcceleration_dqB * Cov_qQEKF * dAcceleration_dqB' + dt^2 * dAcceleration_dCOM * cov_COM * dAcceleration_dCOM'; 
  maxval = SamplePeriod * SamplePeriod;
  a21 = SamplePeriod * SamplePeriod;

  //  Setup covariance matrices
  // 'VelocityEstimator2:71' Q = cov_velocity;
  //      if (Wheel1Slip || Wheel2Slip || Wheel3Slip)
  //          Q = 0*Q;
  //      end
  //  Measurement covariances						
  // 'VelocityEstimator2:77' cov_quantization = 0.5 * eye(3);
  //     %% Prediction step
  // 'VelocityEstimator2:81' X_apriori = zeros(2,1);
  //  Propagate the velocity based on shape-accelerated model for acceleration	
  // 'VelocityEstimator2:84' dx_apriori = dx;
  dx_apriori = X[0];

  // 'VelocityEstimator2:85' dy_apriori = dy;
  dy_apriori = X[1];

  // 'VelocityEstimator2:87' acceleration = SteadyStateAcceleration(xCOM,yCOM,l,Jk,Jw,Mb,Mk,dx,dy,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  SteadyStateAcceleration(COM[0], COM[1], l, Jk, Jw, Mb, Mk, X[0], X[1], g,
    qQEKF[0], qQEKF[1], qQEKF[2], qQEKF[3], rk, rw, acceleration);

  // 'VelocityEstimator2:88' if (UseTiltForPrediction)
  if (UseTiltForPrediction) {
    // 'VelocityEstimator2:89' dx_apriori = dx_apriori + dt * acceleration(1);
    dx_apriori = X[0] + SamplePeriod * acceleration[0];

    // 'VelocityEstimator2:90' dy_apriori = dy_apriori + dt * acceleration(2);
    dy_apriori = X[1] + SamplePeriod * acceleration[1];
  }

  //  Determine model Jacobian (F)
  // 'VelocityEstimator2:94' F_prev = eye(2);
  //  Set apriori state
  // 'VelocityEstimator2:97' X_apriori = [dx_apriori
  // 'VelocityEstimator2:98' 				 dy_apriori];
  //  Calculate apriori covariance of estimate error
  // 'VelocityEstimator2:101' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (r2 = 0; r2 < 2; r2++) {
    for (i = 0; i < 2; i++) {
      b_P_prev[r2 + (i << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_P_prev[r2 + (i << 1)] += P_prev[r2 + (r1 << 1)] * (float)iv4[r1 + (i <<
          1)];
      }
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    for (i = 0; i < 2; i++) {
      b_Cov_qQEKF[r2 + (i << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        b_Cov_qQEKF[r2 + (i << 2)] += Cov_qQEKF[r2 + (r1 << 2)] * (maxval *
          b_dAcceleration_dqB[r1 + (i << 2)]);
      }
    }
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (i = 0; i < 2; i++) {
      b_Var_COM[r2 + (i << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        b_Var_COM[r2 + (i << 1)] += Var_COM * (float)iv4[r2 + (r1 << 1)] * (a21 *
          b_dAcceleration_dCOM[r1 + (i << 1)]);
      }

      dAcceleration_dqB[r2 + (i << 1)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        dAcceleration_dqB[r2 + (i << 1)] += b_dAcceleration_dqB[r1 + (r2 << 2)] *
          b_Cov_qQEKF[r1 + (i << 2)];
      }
    }
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (i = 0; i < 2; i++) {
      dAcceleration_dCOM[r2 + (i << 1)] = 0.0F;
      maxval = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        dAcceleration_dCOM[r2 + (i << 1)] += b_dAcceleration_dCOM[r1 + (r2 << 1)]
          * b_Var_COM[r1 + (i << 1)];
        maxval += (float)iv4[r2 + (r1 << 1)] * b_P_prev[r1 + (i << 1)];
      }

      P_apriori[r2 + (i << 1)] = maxval + (dAcceleration_dqB[r2 + (i << 1)] +
        dAcceleration_dCOM[r2 + (i << 1)]);
    }
  }

  //     %% Update/correction step
  // 'VelocityEstimator2:104' z = [EncoderDiffMeas];
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
  // 'VelocityEstimator2:117' dx_ball_apriori = dx_apriori;
  dx_ball_apriori = dx_apriori;

  // 'VelocityEstimator2:118' dy_ball_apriori = dy_apriori;
  dy_ball_apriori = dy_apriori;

  // 'VelocityEstimator2:120' if (EstimateVelocityAtCoR)
  if (EstimateVelocityAtCoR) {
    // 'VelocityEstimator2:121' vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,CoR]'; 
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
    for (r2 = 0; r2 < 4; r2++) {
      for (i = 0; i < 4; i++) {
        f_qQEKF[r2 + (i << 2)] = 0.0F;
        d_qdotQEKF[r2 + (i << 2)] = 0.0F;
        for (r1 = 0; r1 < 4; r1++) {
          f_qQEKF[r2 + (i << 2)] += b_qQEKF[r2 + (r1 << 2)] * b_qdotQEKF[r1 + (i
            << 2)];
          d_qdotQEKF[r2 + (i << 2)] += c_qdotQEKF[r2 + (r1 << 2)] * c_qQEKF[r1 +
            (i << 2)];
        }
      }
    }

    for (r2 = 0; r2 < 4; r2++) {
      for (i = 0; i < 4; i++) {
        b_qQEKF[i + (r2 << 2)] = f_qQEKF[i + (r2 << 2)] + d_qdotQEKF[i + (r2 <<
          2)];
      }
    }

    for (r2 = 0; r2 < 4; r2++) {
      for (i = 0; i < 3; i++) {
        d_qQEKF[r2 + (i << 2)] = 0.0F;
        for (r1 = 0; r1 < 4; r1++) {
          d_qQEKF[r2 + (i << 2)] += b_qQEKF[r2 + (r1 << 2)] * (float)iv0[r1 + (i
            << 2)];
        }
      }
    }

    for (r2 = 0; r2 < 3; r2++) {
      fv0[r2] = 0.0F;
      for (i = 0; i < 4; i++) {
        fv0[r2] += dAcceleration_dqB[i] * d_qQEKF[i + (r2 << 2)];
      }

      dpsi[r2] = fv0[r2];
    }

    // 'VelocityEstimator2:122' dx_ball_apriori = dx_ball_apriori - vel_2L_to_ball_correction(1); 
    dx_ball_apriori = dx_apriori - dpsi[0];

    // 'VelocityEstimator2:123' dy_ball_apriori = dy_ball_apriori - vel_2L_to_ball_correction(2); 
    dy_ball_apriori = dy_apriori - dpsi[1];
  }

  // 'VelocityEstimator2:126' dpsi_apriori = W * (1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-dy_ball_apriori;dx_ball_apriori;0] - 2*Phi(qQEKF)'*qdotQEKF); 
  b_y = 1.0F / rk;

  //  InverseKinematics(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dx_ball_apriori,dy_ball_apriori,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'VelocityEstimator2:127' z_encoder_hat = TicksPrRev/(2*pi) * dt * dpsi_apriori; 
  a = TicksPrRev / 6.28318548F * SamplePeriod;

  // 'VelocityEstimator2:128' z_hat = z_encoder_hat;
  //  Measurement Jacobian	
  // 'VelocityEstimator2:131' H = zeros(3,2);
  for (r2 = 0; r2 < 6; r2++) {
    H[r2] = 0.0;
  }

  // 'VelocityEstimator2:132' H(1:3,1) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;0;1;0]; 
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
  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 4; i++) {
      b_a[i + (r2 << 2)] = maxval * W[i + (r2 << 2)] / rk;
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
  for (r2 = 0; r2 < 4; r2++) {
    for (i = 0; i < 3; i++) {
      d_qQEKF[r2 + (i << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        d_qQEKF[r2 + (i << 2)] += b_qQEKF[r2 + (r1 << 2)] * b_a[r1 + (i << 2)];
      }
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    for (i = 0; i < 3; i++) {
      e_qQEKF[r2 + (i << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        e_qQEKF[r2 + (i << 2)] += c_qQEKF[r2 + (r1 << 2)] * d_qQEKF[r1 + (i << 2)];
      }
    }
  }

  //  d encoder_meas  /  d dx_2L
  // 'VelocityEstimator2:133' H(1:3,2) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-1;0;0]; 
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
  for (r2 = 0; r2 < 3; r2++) {
    fv0[r2] = 0.0F;
    for (i = 0; i < 4; i++) {
      b_a[i + (r2 << 2)] = maxval * W[i + (r2 << 2)] / rk;
      fv0[r2] += (float)iv5[i] * e_qQEKF[i + (r2 << 2)];
    }

    H[r2 << 1] = fv0[r2];
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
  for (r2 = 0; r2 < 4; r2++) {
    for (i = 0; i < 3; i++) {
      d_qQEKF[r2 + (i << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        d_qQEKF[r2 + (i << 2)] += b_qQEKF[r2 + (r1 << 2)] * b_a[r1 + (i << 2)];
      }
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    for (i = 0; i < 3; i++) {
      e_qQEKF[r2 + (i << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        e_qQEKF[r2 + (i << 2)] += c_qQEKF[r2 + (r1 << 2)] * d_qQEKF[r1 + (i << 2)];
      }
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    fv0[r2] = 0.0F;
    for (i = 0; i < 4; i++) {
      fv0[r2] += (float)iv6[i] * e_qQEKF[i + (r2 << 2)];
    }

    H[1 + (r2 << 1)] = fv0[r2];
  }

  //  d encoder_meas  /  d dy_2L
  //  Calculate measurement covariance
  // dEncoder_dqQEKF = OffsetEstimator_dEncoders2L_dq(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dt,dx_2L_apriori,dy_2L_apriori,l,1,TicksPrRev,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // R_encoder = 4*cov_quantization + ...
  //             dEncoder_dqQEKF * Cov_qQEKF * dEncoder_dqQEKF';% + ((n_gear*n_ticksRev)/(2*pi)*dt)^2 * W * vec * cov_omega * devec * W'; 
  // 'VelocityEstimator2:139' R_encoder = eta_encoder * 4*cov_quantization;
  a21 = eta_encoder * 4.0F;

  // 'VelocityEstimator2:140' R = R_encoder;
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
  // 'VelocityEstimator2:172' S = H * P_apriori * H' + R;
  for (r2 = 0; r2 < 2; r2++) {
    for (i = 0; i < 3; i++) {
      K[r2 + (i << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        K[r2 + (i << 1)] += P_apriori[r2 + (r1 << 1)] * (float)H[r1 + (i << 1)];
      }
    }
  }

  // K = P_apriori * H' * inv(S);
  // 'VelocityEstimator2:174' K = P_apriori * H' / S;
  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        maxval += (float)H[r1 + (r2 << 1)] * K[r1 + (i << 1)];
      }

      S[r2 + 3 * i] = maxval + a21 * fv6[r2 + 3 * i];
    }

    for (i = 0; i < 2; i++) {
      c_y[r2 + 3 * i] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        c_y[r2 + 3 * i] += (float)H[r1 + (r2 << 1)] * P_apriori[r1 + (i << 1)];
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
  for (i = 0; i < 2; i++) {
    K[r1 + 3 * i] = c_y[3 * i] / S[3 * r1];
    K[r2 + 3 * i] = c_y[1 + 3 * i] - K[r1 + 3 * i] * S[1 + 3 * r1];
    K[r3 + 3 * i] = c_y[2 + 3 * i] - K[r1 + 3 * i] * S[2 + 3 * r1];
    K[r2 + 3 * i] /= S[1 + 3 * r2];
    K[r3 + 3 * i] -= K[r2 + 3 * i] * S[2 + 3 * r2];
    K[r3 + 3 * i] /= S[2 + 3 * r3];
    K[r2 + 3 * i] -= K[r3 + 3 * i] * S[1 + 3 * r3];
    K[r1 + 3 * i] -= K[r3 + 3 * i] * S[3 * r3];
    K[r1 + 3 * i] -= K[r2 + 3 * i] * S[3 * r2];
  }

  //  Correct using innovation
  // 'VelocityEstimator2:177' X_aposteriori = X_apriori + K * (z - z_hat);
  dAcceleration_dqB[0] = 0.0F;
  dAcceleration_dqB[1] = -dy_ball_apriori;
  dAcceleration_dqB[2] = dx_ball_apriori;
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
  b_qdotQEKF[0] = b_y * qQEKF[0];
  b_qdotQEKF[4] = b_y * -qQEKF[1];
  b_qdotQEKF[8] = b_y * -qQEKF[2];
  b_qdotQEKF[12] = b_y * -qQEKF[3];
  b_qdotQEKF[1] = b_y * qQEKF[1];
  b_qdotQEKF[5] = b_y * qQEKF[0];
  b_qdotQEKF[9] = b_y * -qQEKF[3];
  b_qdotQEKF[13] = b_y * qQEKF[2];
  b_qdotQEKF[2] = b_y * qQEKF[2];
  b_qdotQEKF[6] = b_y * qQEKF[3];
  b_qdotQEKF[10] = b_y * qQEKF[0];
  b_qdotQEKF[14] = b_y * -qQEKF[1];
  b_qdotQEKF[3] = b_y * qQEKF[3];
  b_qdotQEKF[7] = b_y * -qQEKF[2];
  b_qdotQEKF[11] = b_y * qQEKF[1];
  b_qdotQEKF[15] = b_y * qQEKF[0];
  for (r2 = 0; r2 < 4; r2++) {
    for (i = 0; i < 4; i++) {
      c_qQEKF[r2 + (i << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        c_qQEKF[r2 + (i << 2)] += b_qQEKF[r2 + (r1 << 2)] * b_qdotQEKF[r1 + (i <<
          2)];
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
  for (r2 = 0; r2 < 4; r2++) {
    dAcceleration_dCOM[r2] = 0.0F;
    b_dAcceleration_dCOM[r2] = 0.0F;
    for (i = 0; i < 4; i++) {
      dAcceleration_dCOM[r2] += dAcceleration_dqB[i] * c_qQEKF[i + (r2 << 2)];
      b_dAcceleration_dCOM[r2] += qdotQEKF[i] * b_qdotQEKF[i + (r2 << 2)];
    }

    fv2[r2] = dAcceleration_dCOM[r2] - b_dAcceleration_dCOM[r2];
  }

  for (r2 = 0; r2 < 3; r2++) {
    fv0[r2] = 0.0F;
    for (i = 0; i < 4; i++) {
      fv0[r2] += fv2[i] * W[i + (r2 << 2)];
    }

    dpsi[r2] = EncoderDiffMeas[r2] - a * fv0[r2];
  }

  b_dx_apriori[0] = dx_apriori;
  b_dx_apriori[1] = dy_apriori;
  for (r2 = 0; r2 < 2; r2++) {
    acceleration[r2] = 0.0F;
    for (i = 0; i < 3; i++) {
      acceleration[r2] += dpsi[i] * K[i + 3 * r2];
    }

    X_out[r2] = b_dx_apriori[r2] + acceleration[r2];
  }

  // 'VelocityEstimator2:178' P_aposteriori = (eye(2) - K*H) * P_apriori;
  for (r2 = 0; r2 < 4; r2++) {
    I[r2] = 0;
  }

  for (i = 0; i < 2; i++) {
    I[i + (i << 1)] = 1;
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (i = 0; i < 2; i++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 3; r1++) {
        maxval += (float)H[r2 + (r1 << 1)] * K[r1 + 3 * i];
      }

      b_dAcceleration_dCOM[r2 + (i << 1)] = (float)I[r2 + (i << 1)] - maxval;
    }
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (i = 0; i < 2; i++) {
      P_out[r2 + (i << 1)] = 0.0F;
      for (r1 = 0; r1 < 2; r1++) {
        P_out[r2 + (i << 1)] += P_apriori[r2 + (r1 << 1)] *
          b_dAcceleration_dCOM[r1 + (i << 1)];
      }
    }
  }

  //      else
  //          % no valid sensor data to perform correction on
  //          X_aposteriori = X_apriori;
  //          P_aposteriori = P_apriori;
  //      end
  //     %% Send output to Simulink
  // 'VelocityEstimator2:186' X_out = X_aposteriori;
  // 'VelocityEstimator2:187' P_out = P_aposteriori;
  // 'VelocityEstimator2:189' if (EnableWheelSlipDetector && (Wheel1Slip || Wheel2Slip || Wheel3Slip)) 
  if (EnableWheelSlipDetector && (WheelSlip[0] || WheelSlip[1] || WheelSlip[2]))
  {
    // 'VelocityEstimator2:190' X_out = X;
    for (i = 0; i < 2; i++) {
      X_out[i] = X[i];
    }

    // 'VelocityEstimator2:191' P_out = eye(2) * WheelSlipSetVelocityVariance;
    for (r2 = 0; r2 < 4; r2++) {
      P_out[r2] = (float)iv4[r2] * WheelSlipSetVelocityVariance;
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void VelocityEstimator2_init()
{
  int i;

  // 'VelocityEstimator2:23' dpsi_old = single([0;0;0]);
  for (i = 0; i < 3; i++) {
    dpsi_old[i] = 0.0F;
  }
}

//
// File trailer for VelocityEstimator2.cpp
//
// [EOF]
//
