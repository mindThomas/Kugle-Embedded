//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: COMEstimator.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 25-Feb-2019 11:02:50
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "COMEstimator.h"
#include "SteadyStateAcceleration.h"
#include "SteadyStateAcceleration_dq.h"
#include "SteadyStateAcceleration_dCOM.h"

// Function Definitions

//
// function [X_out, P_out] = COMEstimator(X, P_prev, qQEKF, Cov_qQEKF, qdotQEKF, Velocity, VelocityDiff, Cov_Velocity_meas, SamplePeriod, Jk,Mk,rk,Mb,Jw,rw,l,g,CoR)
// for q o p = Phi(q) * p
// Arguments    : const float X[2]
//                const float P_prev[4]
//                const float qQEKF[4]
//                const float Cov_qQEKF[16]
//                const float qdotQEKF[4]
//                const float Velocity[2]
//                const float VelocityDiff[2]
//                const float Cov_Velocity_meas[4]
//                float SamplePeriod
//                float Jk
//                float Mk
//                float rk
//                float Mb
//                float Jw
//                float rw
//                float l
//                float g
//                float CoR
//                float X_out[2]
//                float P_out[4]
// Return Type  : void
//
__attribute__((optimize("O3"))) void COMEstimator(const float X[2], const float P_prev[4], const float qQEKF[4],
                  const float Cov_qQEKF[16], const float qdotQEKF[4], const
                  float Velocity[2], const float VelocityDiff[2], const float
                  Cov_Velocity_meas[4], float SamplePeriod, float Jk, float Mk,
                  float rk, float Mb, float Jw, float rw, float l, float g,
                  float CoR, float X_out[2], float P_out[4])
{
  float b_P_prev[4];
  float b_qQEKF[16];
  float b_qdotQEKF[16];
  float c_qdotQEKF[16];
  float c_qQEKF[16];
  int r1;
  int r2;
  float d_qQEKF[16];
  float d_qdotQEKF[16];
  int k;
  float fv0[3];
  float e_qQEKF[12];
  float vel_2L_to_ball_correction[3];
  static const signed char iv0[12] = { 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  static const signed char iv1[4] = { 1, 0, 0, 1 };

  float dAcceleration_dCOM[4];
  float dAcceleration_dqB[8];
  float a21;
  float a22;
  float b_Cov_qQEKF[8];
  float K[4];
  float P_apriori[4];
  float S[4];
  float b_Cov_Velocity_meas[4];
  float y[4];
  float b_VelocityDiff[2];
  float b_X[2];
  float c_VelocityDiff[2];
  signed char I[4];

  // 'COMEstimator:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p 
  // 'COMEstimator:4'               q(2) q(1)  -q(4) q(3);
  // 'COMEstimator:5'               q(3) q(4)  q(1)  -q(2);
  // 'COMEstimator:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'COMEstimator:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q 
  // 'COMEstimator:8'                  p(2) p(1) p(4) -p(3);
  // 'COMEstimator:9'                  p(3) -p(4) p(1) p(2);
  // 'COMEstimator:10'                  p(4) p(3) -p(2) p(1)];
  // 'COMEstimator:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'COMEstimator:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'COMEstimator:14' I_conj = diag([1,-1,-1,-1]);
  // 'COMEstimator:16' dt = SamplePeriod;
  // 'COMEstimator:18' dx_2L = Velocity(1);
  // 'COMEstimator:19' dy_2L = Velocity(2);
  //  Split state vector, X[k-1], into individual variables
  // 'COMEstimator:22' xCOM = X(1);
  // 'COMEstimator:23' yCOM = X(2);
  // 'COMEstimator:25' vel_2L_to_ball_correction = devec * (Phi(qdotQEKF)*Gamma(qQEKF)' + Phi(qQEKF)*Gamma(qdotQEKF)') * [0,0,0,CoR]'; 
  b_P_prev[0] = 0.0F;
  b_P_prev[1] = 0.0F;
  b_P_prev[2] = 0.0F;
  b_P_prev[3] = CoR;
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
  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 4; r2++) {
      d_qQEKF[r1 + (r2 << 2)] = 0.0F;
      d_qdotQEKF[r1 + (r2 << 2)] = 0.0F;
      for (k = 0; k < 4; k++) {
        d_qQEKF[r1 + (r2 << 2)] += b_qQEKF[r1 + (k << 2)] * b_qdotQEKF[k + (r2 <<
          2)];
        d_qdotQEKF[r1 + (r2 << 2)] += c_qdotQEKF[r1 + (k << 2)] * c_qQEKF[k +
          (r2 << 2)];
      }
    }
  }

  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 4; r2++) {
      b_qQEKF[r2 + (r1 << 2)] = d_qQEKF[r2 + (r1 << 2)] + d_qdotQEKF[r2 + (r1 <<
        2)];
    }
  }

  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      e_qQEKF[r1 + (r2 << 2)] = 0.0F;
      for (k = 0; k < 4; k++) {
        e_qQEKF[r1 + (r2 << 2)] += b_qQEKF[r1 + (k << 2)] * (float)iv0[k + (r2 <<
          2)];
      }
    }
  }

  for (r1 = 0; r1 < 3; r1++) {
    fv0[r1] = 0.0F;
    for (r2 = 0; r2 < 4; r2++) {
      fv0[r1] += b_P_prev[r2] * e_qQEKF[r2 + (r1 << 2)];
    }

    vel_2L_to_ball_correction[r1] = fv0[r1];
  }

  // 'COMEstimator:26' dx_ball = dx_2L - vel_2L_to_ball_correction(1);
  // 'COMEstimator:27' dy_ball = dy_2L - vel_2L_to_ball_correction(2);
  //  Process covariances
  // 'COMEstimator:30' cov_COM = zeros(2);
  //  1e-18 * eye(2)
  //  Setup covariance matrices
  // 'COMEstimator:33' Q = cov_COM;
  //     %% Prediction step
  // 'COMEstimator:36' X_apriori = zeros(2,1);
  //  Propagate COM offsets (stays constant)
  // 'COMEstimator:39' xCOM_apriori = xCOM;
  // 'COMEstimator:40' yCOM_apriori = yCOM;
  //  Determine model Jacobian (F)
  // 'COMEstimator:43' F_prev = eye(2);
  //  d COM_apriori / d COM
  //  Set apriori state
  // 'COMEstimator:46' X_apriori = [xCOM_apriori;
  // 'COMEstimator:47'                  yCOM_apriori];
  //  Calculate apriori covariance of estimate error
  // 'COMEstimator:50' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      b_P_prev[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 2; k++) {
        b_P_prev[r1 + (r2 << 1)] += P_prev[r1 + (k << 1)] * (float)iv1[k + (r2 <<
          1)];
      }
    }
  }

  //     %% Update/correction step
  //  z = (vel[k] - vel[k-1]) / dt
  //  cov(z) = (1/dt)^2 * (cov(vel[k]) + cov(vel[k-1])) ~ (1/dt)^2 * 2 * cov(vel[k])     
  // 'COMEstimator:55' z = VelocityDiff / dt;
  //  Measurement model
  // 'COMEstimator:58' acceleration = SteadyStateAcceleration(xCOM_apriori,yCOM_apriori,l,Jk,Jw,Mb,Mk,dx_ball,dy_ball,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'COMEstimator:59' z_hat = acceleration;
  //  Measurement Jacobian	
  // 'COMEstimator:62' dAcceleration_dCOM = SteadyStateAcceleration_dCOM(xCOM_apriori,yCOM_apriori,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dCOM(X[0], X[1], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
    qQEKF[2], qQEKF[3], rk, dAcceleration_dCOM);

  // 'COMEstimator:63' H = dAcceleration_dCOM;
  //  Measurement covariance
  // 'COMEstimator:66' dAcceleration_dqB = SteadyStateAcceleration_dq(xCOM_apriori,yCOM_apriori,l,Jk,Mb,Mk,g,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk); 
  SteadyStateAcceleration_dq(X[0], X[1], l, Jk, Mb, Mk, g, qQEKF[0], qQEKF[1],
    qQEKF[2], qQEKF[3], rk, dAcceleration_dqB);

  // 'COMEstimator:67' cov_acceleration_uncertainty = dt^2 * dAcceleration_dqB * Cov_qQEKF * dAcceleration_dqB'; 
  a21 = SamplePeriod * SamplePeriod;

  // 'COMEstimator:68' R = Cov_Velocity_meas + cov_acceleration_uncertainty;
  //  Calculate Kalman gain
  // 'COMEstimator:71' S = H * P_apriori * H' + R;
  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      a22 = 0.0F;
      for (k = 0; k < 2; k++) {
        a22 += (float)iv1[r1 + (k << 1)] * b_P_prev[k + (r2 << 1)];
      }

      P_apriori[r1 + (r2 << 1)] = a22;
    }

    for (r2 = 0; r2 < 2; r2++) {
      K[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 2; k++) {
        K[r1 + (r2 << 1)] += P_apriori[r1 + (k << 1)] * dAcceleration_dCOM[k +
          (r2 << 1)];
      }
    }
  }

  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      b_Cov_qQEKF[r1 + (r2 << 2)] = 0.0F;
      for (k = 0; k < 4; k++) {
        b_Cov_qQEKF[r1 + (r2 << 2)] += Cov_qQEKF[r1 + (k << 2)] * (a21 *
          dAcceleration_dqB[k + (r2 << 2)]);
      }
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      b_P_prev[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 2; k++) {
        b_P_prev[r1 + (r2 << 1)] += dAcceleration_dCOM[k + (r1 << 1)] * K[k +
          (r2 << 1)];
      }

      a22 = 0.0F;
      for (k = 0; k < 4; k++) {
        a22 += dAcceleration_dqB[k + (r1 << 2)] * b_Cov_qQEKF[k + (r2 << 2)];
      }

      b_Cov_Velocity_meas[r1 + (r2 << 1)] = Cov_Velocity_meas[r1 + (r2 << 1)] +
        a22;
    }
  }

  // K = P_apriori * H' * inv(S);
  // 'COMEstimator:73' K = P_apriori * H' / S;
  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      S[r2 + (r1 << 1)] = b_P_prev[r2 + (r1 << 1)] + b_Cov_Velocity_meas[r2 +
        (r1 << 1)];
      y[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 2; k++) {
        y[r1 + (r2 << 1)] += dAcceleration_dCOM[k + (r1 << 1)] * P_apriori[k +
          (r2 << 1)];
      }
    }
  }

  if ((float)fabs((double)S[2]) > (float)fabs((double)S[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = S[r2 << 1] / S[r1 << 1];
  a22 = S[1 + (r2 << 1)] - a21 * S[1 + (r1 << 1)];

  //  Correct using innovation
  // 'COMEstimator:76' X_aposteriori = X_apriori + K * (z - z_hat);
  SteadyStateAcceleration(X[0], X[1], l, Jk, Jw, Mb, Mk, Velocity[0] -
    vel_2L_to_ball_correction[0], Velocity[1] - vel_2L_to_ball_correction[1], g,
    qQEKF[0], qQEKF[1], qQEKF[2], qQEKF[3], rk, rw, b_VelocityDiff);
  for (k = 0; k < 2; k++) {
    K[r1 + (k << 1)] = y[k << 1] / S[r1 << 1];
    K[r2 + (k << 1)] = (y[1 + (k << 1)] - K[r1 + (k << 1)] * S[1 + (r1 << 1)]) /
      a22;
    K[r1 + (k << 1)] -= K[r2 + (k << 1)] * a21;
    c_VelocityDiff[k] = VelocityDiff[k] / SamplePeriod - b_VelocityDiff[k];
  }

  b_X[0] = X[0];
  b_X[1] = X[1];
  for (r1 = 0; r1 < 2; r1++) {
    b_VelocityDiff[r1] = 0.0F;
    for (r2 = 0; r2 < 2; r2++) {
      b_VelocityDiff[r1] += c_VelocityDiff[r2] * K[r2 + (r1 << 1)];
    }

    X_out[r1] = b_X[r1] + b_VelocityDiff[r1];
  }

  // 'COMEstimator:77' P_aposteriori = (eye(2) - K*H) * P_apriori;
  for (r1 = 0; r1 < 4; r1++) {
    I[r1] = 0;
  }

  for (k = 0; k < 2; k++) {
    I[k + (k << 1)] = 1;
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      a22 = 0.0F;
      for (k = 0; k < 2; k++) {
        a22 += dAcceleration_dCOM[r1 + (k << 1)] * K[k + (r2 << 1)];
      }

      b_P_prev[r1 + (r2 << 1)] = (float)I[r1 + (r2 << 1)] - a22;
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      P_out[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 2; k++) {
        P_out[r1 + (r2 << 1)] += P_apriori[r1 + (k << 1)] * b_P_prev[k + (r2 <<
          1)];
      }
    }
  }

  //     %% Send output to Simulink
  // 'COMEstimator:80' X_out = X_aposteriori;
  // 'COMEstimator:81' P_out = P_aposteriori;
}

//
// File trailer for COMEstimator.cpp
//
// [EOF]
//
