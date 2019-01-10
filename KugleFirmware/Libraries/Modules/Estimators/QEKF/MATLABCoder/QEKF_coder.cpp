//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 30-Oct-2018 20:00:29
//

// Include Files
#include <math.h>
#include <string.h>
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include "QEKF.h"

// Function Definitions

//
// function [X_out, P_out] = QEKF(X, P_prev, Gyroscope, Accelerometer, SamplePeriod, BiasEstimationEnabled, NormalizeAccelerometer, cov_gyro, cov_acc, sigma2_bias, g)
// for q o p = Phi(q) * p
// Arguments    : const float X[10]
//                const float P_prev[100]
//                const float Gyroscope[3]
//                const float Accelerometer[3]
//                float SamplePeriod
//                boolean_T BiasEstimationEnabled
//                boolean_T NormalizeAccelerometer
//                const float cov_gyro[9]
//                const float cov_acc[9]
//                float sigma2_bias
//                float g
//                float X_out[10]
//                float P_out[100]
// Return Type  : void
//
__attribute__((optimize("O3"))) void _QEKF(const float X[10], const float P_prev[100], const float Gyroscope[3],
          const float Accelerometer[3], float SamplePeriod, boolean_T
          BiasEstimationEnabled, boolean_T NormalizeAccelerometer, const float
          cov_gyro[9], const float cov_acc[9], float sigma2_bias, float g, float
          X_out[10], float P_out[100])
{
  float gyro_bias[3];
  float gyro_input[3];
  float maxval;
  float norm_acc;
  int i;
  float scale;
  float z[3];
  float f0;
  float absxk;
  float t;
  float X_apriori[10];
  float q_apriori[4];
  static float F_prev[100];
  int r1;
  static const signed char iv0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  float b_gyro_input[3];
  float y[4];
  float c_gyro_input[4];
  static const signed char iv1[12] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b_y;
  float c_y[16];
  float b_q_apriori[12];
  int r2;
  static const signed char iv2[6] = { 1, 0, 0, 1, 0, 0 };

  static const signed char b[4] = { 1, 0, 0, 1 };

  static float P_apriori[100];
  float c_q_apriori[16];
  float d_q_apriori[12];
  float b_cov_gyro[12];
  static float b_F_prev[100];
  float fv0[100];
  static const signed char iv3[4] = { 1, 0, 0, 1 };

  float fv1[4];
  static const signed char a[12] = { 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1 };

  static const signed char iv4[4] = { 0, 0, 0, -1 };

  float fv2[4];
  float q[4];
  float b_q[16];
  static const signed char b_b[16] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0,
    0, 0, -1 };

  float H[30];
  float K[30];
  int r3;
  float S[9];
  float d_y[30];
  float b_z[10];
  signed char I[100];
  float f1;

  // 'QEKF:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p
  // 'QEKF:4'               q(2) q(1)  -q(4) q(3);
  // 'QEKF:5'               q(3) q(4)  q(1)  -q(2);
  // 'QEKF:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'QEKF:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q
  // 'QEKF:8'                  p(2) p(1) p(4) -p(3);
  // 'QEKF:9'                  p(3) -p(4) p(1) p(2);
  // 'QEKF:10'                  p(4) p(3) -p(2) p(1)];
  // 'QEKF:12' devec = single([0,1,0,0;0,0,1,0;0,0,0,1]);
  //  'v' in notes
  // 'QEKF:13' vec = single([0,0,0;1,0,0;0,1,0;0,0,1]);
  //  '^' in notes
  // 'QEKF:14' I_conj = single(diag([1,-1,-1,-1]));
  // 'QEKF:16' dt = SamplePeriod;
  // BiasEstimationEnabled = 1*(BiasEstimationEnabled > 0);
  //  Split state vector, X[k-1], into individual variables
  // 'QEKF:20' q1 = X(1);
  // 'QEKF:21' q2 = X(2);
  // 'QEKF:22' q3 = X(3);
  // 'QEKF:23' q4 = X(4);
  // 'QEKF:24' q = [q1, q2, q3, q4]';
  // 'QEKF:26' dq1 = X(5);
  // 'QEKF:27' dq2 = X(6);
  // 'QEKF:28' dq3 = X(7);
  // 'QEKF:29' dq4 = X(8);
  // 'QEKF:30' dq = [dq1, dq2, dq3, dq4]';
  //  omeg = 2 * devec * Phi(q)' * dq;
  //  dq = 1/2 * Phi(q) * [zeros(1,3); eye(3)] * omeg
  // 'QEKF:34' gyro_bias_x = X(9);
  // 'QEKF:35' gyro_bias_y = X(10);
  // 'QEKF:36' gyro_bias = [gyro_bias_x; gyro_bias_y; 0];
  gyro_bias[0] = X[8];
  gyro_bias[1] = X[9];
  gyro_bias[2] = 0.0F;

  // 'QEKF:38' gyro_x = Gyroscope(1);
  // 'QEKF:39' gyro_y = Gyroscope(2);
  // 'QEKF:40' gyro_z = Gyroscope(3);
  // 'QEKF:41' gyro_input = [gyro_x; gyro_y; gyro_z];
  gyro_input[0] = Gyroscope[0];
  gyro_input[1] = Gyroscope[1];
  gyro_input[2] = Gyroscope[2];

  //  Process covariances
  // 'QEKF:44' cov_q = single(zeros(4,4));
  //  quaternion kinematics is correct since we propagate with Quaternion exponential 
  // 'QEKF:45' cov_omega = cov_gyro;
  // 'QEKF:46' cov_q_dot = (1/2 * Phi(q) * [zeros(1,3); eye(3)]) * cov_omega * (1/2 * Phi(q) * [zeros(1,3); eye(3)])'; 
  //  convert covariance from omega/gyro to q_dot states
  // 'QEKF:47' cov_bias = sigma2_bias*dt*eye(2)*BiasEstimationEnabled + zeros(2,2); 
  maxval = sigma2_bias * SamplePeriod;

  //  bias stays constant
  //  Setup covariance matrices
  // 'QEKF:50' Q = [cov_q, zeros(4,4), zeros(4,2);
  // 'QEKF:51'          zeros(4,4), single(cov_q_dot), zeros(4,2);
  // 'QEKF:52'          zeros(2,4), zeros(2,4), cov_bias];
  // 'QEKF:54' R = cov_acc;
  //  Measurement vector
  // 'QEKF:57' if (NormalizeAccelerometer)
  if (NormalizeAccelerometer) {
    // 'QEKF:58' norm_acc = norm(Accelerometer);
    norm_acc = 0.0F;
    scale = 1.29246971E-26F;
    for (i = 0; i < 3; i++) {
      absxk = (float)fabs((double)Accelerometer[i]);
      if (absxk > scale) {
        t = scale / absxk;
        norm_acc = 1.0F + norm_acc * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        norm_acc += t * t;
      }
    }

    mw_arm_sqrt_f32(norm_acc, &f0);
    norm_acc = scale * f0;

    // 'QEKF:59' if (norm_acc > 0)
    if (norm_acc > 0.0F) {
      //  && norm_acc < 9.85)
      // 'QEKF:60' z = Accelerometer / norm_acc;
      for (i = 0; i < 3; i++) {
        z[i] = Accelerometer[i] / norm_acc;
      }
    } else {
      // 'QEKF:61' else
      // 'QEKF:62' z = 0*Accelerometer;
      for (i = 0; i < 3; i++) {
        z[i] = 0.0F * Accelerometer[i];
      }
    }
  } else {
    // 'QEKF:64' else
    // 'QEKF:65' z = Accelerometer;
    for (i = 0; i < 3; i++) {
      z[i] = Accelerometer[i];
    }
  }

  //     %% Prediction step
  // 'QEKF:69' X_apriori = single(zeros(10,1));
  for (i = 0; i < 10; i++) {
    X_apriori[i] = 0.0F;
  }

  //  Propagate quaternion correctly using Forward Euler
  // 'QEKF:72' q_apriori = q + dt * dq;
  q_apriori[0] = X[0] + SamplePeriod * X[4];
  q_apriori[1] = X[1] + SamplePeriod * X[5];
  q_apriori[2] = X[2] + SamplePeriod * X[6];
  q_apriori[3] = X[3] + SamplePeriod * X[7];

  //  Forward Euler
  //  Propagate/set angular velocity states
  // 'QEKF:75' q_dot_apriori = 1/2 * Phi(q) * [zeros(1,3); eye(3)] * (gyro_input - gyro_bias); 
  //  Propagate gyro bias
  // 'QEKF:78' gyro_bias_apriori = gyro_bias;
  //  Determine model Jacobian (F)
  // 'QEKF:81' F_prev = single(zeros(10,10));
  memset(&F_prev[0], 0, 100U * sizeof(float));

  // 'QEKF:82' F_prev(1:4,1:4) = eye(4);
  // 'QEKF:83' F_prev(1:4,5:8) = dt*eye(4);
  // 'QEKF:84' F_prev(1:4,9:10) = zeros(4,2);
  for (i = 0; i < 4; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      F_prev[r1 + 10 * i] = iv0[r1 + (i << 2)];
      F_prev[(r1 + 10 * i) + 4] = SamplePeriod * (float)iv0[r1 + (i << 2)];
    }

    for (r1 = 0; r1 < 2; r1++) {
      F_prev[(r1 + 10 * i) + 8] = 0.0F;
    }
  }

  // 'QEKF:85' F_prev(5:8,1:4) = 1/2 * Gamma([zeros(1,3); eye(3)] * (gyro_input - gyro_bias)); 
  for (i = 0; i < 3; i++) {
    b_gyro_input[i] = gyro_input[i] - gyro_bias[i];
  }

  for (i = 0; i < 4; i++) {
    c_gyro_input[i] = 0.0F;
    for (r1 = 0; r1 < 3; r1++) {
      c_gyro_input[i] += b_gyro_input[r1] * (float)iv1[r1 + 3 * i];
    }

    y[i] = c_gyro_input[i];
    for (r1 = 0; r1 < 4; r1++) {
      F_prev[(r1 + 10 * (4 + i)) + 4] = 0.0F;
    }
  }

  F_prev[40] = 0.5F * y[0];
  F_prev[41] = 0.5F * -y[1];
  F_prev[42] = 0.5F * -y[2];
  F_prev[43] = 0.5F * -y[3];
  F_prev[50] = 0.5F * y[1];
  F_prev[51] = 0.5F * y[0];
  F_prev[52] = 0.5F * y[3];
  F_prev[53] = 0.5F * -y[2];
  F_prev[60] = 0.5F * y[2];
  F_prev[61] = 0.5F * -y[3];
  F_prev[62] = 0.5F * y[0];
  F_prev[63] = 0.5F * y[1];
  F_prev[70] = 0.5F * y[3];
  F_prev[71] = 0.5F * y[2];
  F_prev[72] = 0.5F * -y[1];
  F_prev[73] = 0.5F * y[0];

  // 'QEKF:86' F_prev(5:8,5:8) = zeros(4,4);
  // 'QEKF:87' F_prev(5:8,9:10) = BiasEstimationEnabled*-1/2 * Phi(q) * [zeros(1,3); eye(3)] * [eye(2); zeros(1,2)]; 
  b_y = -(double)BiasEstimationEnabled / 2.0;
  c_y[0] = (float)b_y * X[0];
  c_y[1] = (float)b_y * -X[1];
  c_y[2] = (float)b_y * -X[2];
  c_y[3] = (float)b_y * -X[3];
  c_y[4] = (float)b_y * X[1];
  c_y[5] = (float)b_y * X[0];
  c_y[6] = (float)b_y * -X[3];
  c_y[7] = (float)b_y * X[2];
  c_y[8] = (float)b_y * X[2];
  c_y[9] = (float)b_y * X[3];
  c_y[10] = (float)b_y * X[0];
  c_y[11] = (float)b_y * -X[1];
  c_y[12] = (float)b_y * X[3];
  c_y[13] = (float)b_y * -X[2];
  c_y[14] = (float)b_y * X[1];
  c_y[15] = (float)b_y * X[0];
  for (i = 0; i < 3; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      b_q_apriori[i + 3 * r1] = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        b_q_apriori[i + 3 * r1] += (float)iv1[i + 3 * r2] * c_y[r2 + (r1 << 2)];
      }
    }
  }

  for (i = 0; i < 2; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      F_prev[(i + 10 * (4 + r1)) + 8] = 0.0F;
      for (r2 = 0; r2 < 3; r2++) {
        F_prev[(i + 10 * (4 + r1)) + 8] += (float)iv2[i + (r2 << 1)] *
          b_q_apriori[r2 + 3 * r1];
      }
    }
  }

  // 'QEKF:88' F_prev(9:10,1:4) = zeros(2,4);
  // 'QEKF:89' F_prev(9:10,5:8) = zeros(2,4);
  // 'QEKF:90' F_prev(9:10,9:10) = BiasEstimationEnabled*eye(2);
  for (i = 0; i < 2; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      F_prev[r1 + 10 * (8 + i)] = 0.0F;
      F_prev[(r1 + 10 * (8 + i)) + 4] = 0.0F;
    }

    for (r1 = 0; r1 < 2; r1++) {
      F_prev[(r1 + 10 * (8 + i)) + 8] = (float)BiasEstimationEnabled * (float)
        b[r1 + (i << 1)];
    }
  }

  //  Set apriori state
  // 'QEKF:93' X_apriori(1:4) = q_apriori;
  for (i = 0; i < 4; i++) {
    X_apriori[i] = q_apriori[i];
  }

  // 'QEKF:94' X_apriori(5:8) = q_dot_apriori;
  c_y[0] = 0.5F * X[0];
  c_y[1] = 0.5F * -X[1];
  c_y[2] = 0.5F * -X[2];
  c_y[3] = 0.5F * -X[3];
  c_y[4] = 0.5F * X[1];
  c_y[5] = 0.5F * X[0];
  c_y[6] = 0.5F * -X[3];
  c_y[7] = 0.5F * X[2];
  c_y[8] = 0.5F * X[2];
  c_y[9] = 0.5F * X[3];
  c_y[10] = 0.5F * X[0];
  c_y[11] = 0.5F * -X[1];
  c_y[12] = 0.5F * X[3];
  c_y[13] = 0.5F * -X[2];
  c_y[14] = 0.5F * X[1];
  c_y[15] = 0.5F * X[0];
  for (i = 0; i < 3; i++) {
    b_gyro_input[i] = gyro_input[i] - gyro_bias[i];
    for (r1 = 0; r1 < 4; r1++) {
      b_q_apriori[i + 3 * r1] = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        b_q_apriori[i + 3 * r1] += (float)iv1[i + 3 * r2] * c_y[r2 + (r1 << 2)];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    c_gyro_input[i] = 0.0F;
    for (r1 = 0; r1 < 3; r1++) {
      c_gyro_input[i] += b_gyro_input[r1] * b_q_apriori[r1 + 3 * i];
    }

    X_apriori[4 + i] = c_gyro_input[i];
  }

  // 'QEKF:95' X_apriori(9:10) = gyro_bias_apriori(1:2);
  for (i = 0; i < 2; i++) {
    X_apriori[i + 8] = gyro_bias[i];
  }

  //  Calculate apriori covariance of estimate error
  // 'QEKF:98' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (i = 0; i < 10; i++) {
    for (r1 = 0; r1 < 10; r1++) {
      P_apriori[i + 10 * r1] = 0.0F;
      for (r2 = 0; r2 < 10; r2++) {
        P_apriori[i + 10 * r1] += P_prev[i + 10 * r2] * F_prev[r2 + 10 * r1];
      }
    }
  }

  c_y[0] = 0.5F * X[0];
  c_y[1] = 0.5F * -X[1];
  c_y[2] = 0.5F * -X[2];
  c_y[3] = 0.5F * -X[3];
  c_y[4] = 0.5F * X[1];
  c_y[5] = 0.5F * X[0];
  c_y[6] = 0.5F * -X[3];
  c_y[7] = 0.5F * X[2];
  c_y[8] = 0.5F * X[2];
  c_y[9] = 0.5F * X[3];
  c_y[10] = 0.5F * X[0];
  c_y[11] = 0.5F * -X[1];
  c_y[12] = 0.5F * X[3];
  c_y[13] = 0.5F * -X[2];
  c_y[14] = 0.5F * X[1];
  c_y[15] = 0.5F * X[0];
  c_q_apriori[0] = 0.5F * X[0];
  c_q_apriori[1] = 0.5F * -X[1];
  c_q_apriori[2] = 0.5F * -X[2];
  c_q_apriori[3] = 0.5F * -X[3];
  c_q_apriori[4] = 0.5F * X[1];
  c_q_apriori[5] = 0.5F * X[0];
  c_q_apriori[6] = 0.5F * -X[3];
  c_q_apriori[7] = 0.5F * X[2];
  c_q_apriori[8] = 0.5F * X[2];
  c_q_apriori[9] = 0.5F * X[3];
  c_q_apriori[10] = 0.5F * X[0];
  c_q_apriori[11] = 0.5F * -X[1];
  c_q_apriori[12] = 0.5F * X[3];
  c_q_apriori[13] = 0.5F * -X[2];
  c_q_apriori[14] = 0.5F * X[1];
  c_q_apriori[15] = 0.5F * X[0];
  for (i = 0; i < 3; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      b_q_apriori[i + 3 * r1] = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        b_q_apriori[i + 3 * r1] += (float)iv1[i + 3 * r2] * c_q_apriori[r2 + (r1
          << 2)];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (r1 = 0; r1 < 3; r1++) {
      d_q_apriori[i + (r1 << 2)] = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        d_q_apriori[i + (r1 << 2)] += (float)iv1[r1 + 3 * r2] * c_y[r2 + (i << 2)];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      b_cov_gyro[i + 3 * r1] = 0.0F;
      for (r2 = 0; r2 < 3; r2++) {
        b_cov_gyro[i + 3 * r1] += cov_gyro[i + 3 * r2] * b_q_apriori[r2 + 3 * r1];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      c_y[i + (r1 << 2)] = 0.0F;
      for (r2 = 0; r2 < 3; r2++) {
        c_y[i + (r1 << 2)] += d_q_apriori[i + (r2 << 2)] * b_cov_gyro[r2 + 3 *
          r1];
      }
    }
  }

  for (i = 0; i < 10; i++) {
    for (r1 = 0; r1 < 10; r1++) {
      b_F_prev[i + 10 * r1] = 0.0F;
      for (r2 = 0; r2 < 10; r2++) {
        b_F_prev[i + 10 * r1] += F_prev[r2 + 10 * i] * P_apriori[r2 + 10 * r1];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      fv0[r1 + 10 * i] = 0.0F;
      fv0[(r1 + 10 * i) + 4] = 0.0F;
    }

    for (r1 = 0; r1 < 2; r1++) {
      fv0[(r1 + 10 * i) + 8] = 0.0F;
    }

    for (r1 = 0; r1 < 4; r1++) {
      fv0[r1 + 10 * (i + 4)] = 0.0F;
      fv0[(r1 + 10 * (i + 4)) + 4] = c_y[r1 + (i << 2)];
    }

    for (r1 = 0; r1 < 2; r1++) {
      fv0[(r1 + 10 * (i + 4)) + 8] = 0.0F;
    }
  }

  for (i = 0; i < 2; i++) {
    for (r1 = 0; r1 < 4; r1++) {
      fv0[r1 + 10 * (i + 8)] = 0.0F;
      fv0[(r1 + 10 * (i + 8)) + 4] = 0.0F;
    }

    for (r1 = 0; r1 < 2; r1++) {
      fv0[(r1 + 10 * (i + 8)) + 8] = maxval * (float)iv3[r1 + (i << 1)] * (float)
        BiasEstimationEnabled;
    }
  }

  for (i = 0; i < 10; i++) {
    for (r1 = 0; r1 < 10; r1++) {
      P_apriori[r1 + 10 * i] = b_F_prev[r1 + 10 * i] + fv0[r1 + 10 * i];
    }
  }

  //     %% Update/correction step
  // 'QEKF:101' if (NormalizeAccelerometer)
  if (NormalizeAccelerometer) {
    //  Accelerometer Measurement model
    // 'QEKF:103' z_hat = -devec * Phi(q_apriori)' * Gamma(q_apriori) * [0;0;0;-1]; 
    c_y[0] = q_apriori[0];
    c_y[4] = -q_apriori[1];
    c_y[8] = -q_apriori[2];
    c_y[12] = -q_apriori[3];
    c_y[1] = q_apriori[1];
    c_y[5] = q_apriori[0];
    c_y[9] = -q_apriori[3];
    c_y[13] = q_apriori[2];
    c_y[2] = q_apriori[2];
    c_y[6] = q_apriori[3];
    c_y[10] = q_apriori[0];
    c_y[14] = -q_apriori[1];
    c_y[3] = q_apriori[3];
    c_y[7] = -q_apriori[2];
    c_y[11] = q_apriori[1];
    c_y[15] = q_apriori[0];
    c_q_apriori[0] = q_apriori[0];
    c_q_apriori[1] = -q_apriori[1];
    c_q_apriori[2] = -q_apriori[2];
    c_q_apriori[3] = -q_apriori[3];
    c_q_apriori[4] = q_apriori[1];
    c_q_apriori[5] = q_apriori[0];
    c_q_apriori[6] = q_apriori[3];
    c_q_apriori[7] = -q_apriori[2];
    c_q_apriori[8] = q_apriori[2];
    c_q_apriori[9] = -q_apriori[3];
    c_q_apriori[10] = q_apriori[0];
    c_q_apriori[11] = q_apriori[1];
    c_q_apriori[12] = q_apriori[3];
    c_q_apriori[13] = q_apriori[2];
    c_q_apriori[14] = -q_apriori[1];
    c_q_apriori[15] = q_apriori[0];
    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        b_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          b_q_apriori[i + (r1 << 2)] += c_y[i + (r2 << 2)] * (float)a[r2 + (r1 <<
            2)];
        }
      }
    }

    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        d_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          d_q_apriori[i + (r1 << 2)] += c_q_apriori[i + (r2 << 2)] *
            b_q_apriori[r2 + (r1 << 2)];
        }
      }
    }

    for (i = 0; i < 3; i++) {
      gyro_input[i] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        gyro_input[i] += (float)iv4[r1] * d_q_apriori[r1 + (i << 2)];
      }

      gyro_bias[i] = gyro_input[i];
    }

    //  Measurement Jacobian
    // 'QEKF:106' H = [-devec * (Gamma(Gamma(q_apriori)*[0;0;0;-1])*I_conj + Phi(Phi(q_apriori)'*[0;0;0;-1])), zeros(3,4), zeros(3,2)]; 
    c_y[0] = q_apriori[0];
    c_y[1] = -q_apriori[1];
    c_y[2] = -q_apriori[2];
    c_y[3] = -q_apriori[3];
    c_y[4] = q_apriori[1];
    c_y[5] = q_apriori[0];
    c_y[6] = q_apriori[3];
    c_y[7] = -q_apriori[2];
    c_y[8] = q_apriori[2];
    c_y[9] = -q_apriori[3];
    c_y[10] = q_apriori[0];
    c_y[11] = q_apriori[1];
    c_y[12] = q_apriori[3];
    c_y[13] = q_apriori[2];
    c_y[14] = -q_apriori[1];
    c_y[15] = q_apriori[0];
    c_q_apriori[0] = q_apriori[0];
    c_q_apriori[4] = -q_apriori[1];
    c_q_apriori[8] = -q_apriori[2];
    c_q_apriori[12] = -q_apriori[3];
    c_q_apriori[1] = q_apriori[1];
    c_q_apriori[5] = q_apriori[0];
    c_q_apriori[9] = -q_apriori[3];
    c_q_apriori[13] = q_apriori[2];
    c_q_apriori[2] = q_apriori[2];
    c_q_apriori[6] = q_apriori[3];
    c_q_apriori[10] = q_apriori[0];
    c_q_apriori[14] = -q_apriori[1];
    c_q_apriori[3] = q_apriori[3];
    c_q_apriori[7] = -q_apriori[2];
    c_q_apriori[11] = q_apriori[1];
    c_q_apriori[15] = q_apriori[0];
    for (i = 0; i < 4; i++) {
      fv1[i] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        fv1[i] += (float)iv4[r1] * c_y[r1 + (i << 2)];
      }

      y[i] = fv1[i];
      c_gyro_input[i] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        c_gyro_input[i] += (float)iv4[r1] * c_q_apriori[r1 + (i << 2)];
      }

      q[i] = c_gyro_input[i];
    }

    c_y[0] = y[0];
    c_y[1] = -y[1];
    c_y[2] = -y[2];
    c_y[3] = -y[3];
    c_y[4] = y[1];
    c_y[5] = y[0];
    c_y[6] = y[3];
    c_y[7] = -y[2];
    c_y[8] = y[2];
    c_y[9] = -y[3];
    c_y[10] = y[0];
    c_y[11] = y[1];
    c_y[12] = y[3];
    c_y[13] = y[2];
    c_y[14] = -y[1];
    c_y[15] = y[0];
    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 4; r1++) {
        c_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          c_q_apriori[i + (r1 << 2)] += (float)b_b[i + (r2 << 2)] * c_y[r2 + (r1
            << 2)];
        }
      }
    }

    b_q[0] = q[0];
    b_q[1] = -q[1];
    b_q[2] = -q[2];
    b_q[3] = -q[3];
    b_q[4] = q[1];
    b_q[5] = q[0];
    b_q[6] = -q[3];
    b_q[7] = q[2];
    b_q[8] = q[2];
    b_q[9] = q[3];
    b_q[10] = q[0];
    b_q[11] = -q[1];
    b_q[12] = q[3];
    b_q[13] = -q[2];
    b_q[14] = q[1];
    b_q[15] = q[0];
    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 4; r1++) {
        c_y[r1 + (i << 2)] = c_q_apriori[r1 + (i << 2)] + b_q[r1 + (i << 2)];
      }
    }

    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        b_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          b_q_apriori[i + (r1 << 2)] += c_y[i + (r2 << 2)] * (float)a[r2 + (r1 <<
            2)];
        }
      }
    }

    for (i = 0; i < 3; i++) {
      for (r1 = 0; r1 < 4; r1++) {
        H[r1 + 10 * i] = b_q_apriori[r1 + (i << 2)];
        H[(r1 + 10 * i) + 4] = 0.0F;
      }

      for (r1 = 0; r1 < 2; r1++) {
        H[(r1 + 10 * i) + 8] = 0.0F;
      }
    }
  } else {
    // 'QEKF:107' else
    //  Accelerometer Measurement model
    // 'QEKF:109' z_hat = -devec * Phi(q_apriori)' * Gamma(q_apriori) * [0;0;0;-g]; 
    fv1[0] = 0.0F;
    fv1[1] = 0.0F;
    fv1[2] = 0.0F;
    fv1[3] = -g;
    c_y[0] = q_apriori[0];
    c_y[4] = -q_apriori[1];
    c_y[8] = -q_apriori[2];
    c_y[12] = -q_apriori[3];
    c_y[1] = q_apriori[1];
    c_y[5] = q_apriori[0];
    c_y[9] = -q_apriori[3];
    c_y[13] = q_apriori[2];
    c_y[2] = q_apriori[2];
    c_y[6] = q_apriori[3];
    c_y[10] = q_apriori[0];
    c_y[14] = -q_apriori[1];
    c_y[3] = q_apriori[3];
    c_y[7] = -q_apriori[2];
    c_y[11] = q_apriori[1];
    c_y[15] = q_apriori[0];
    c_q_apriori[0] = q_apriori[0];
    c_q_apriori[1] = -q_apriori[1];
    c_q_apriori[2] = -q_apriori[2];
    c_q_apriori[3] = -q_apriori[3];
    c_q_apriori[4] = q_apriori[1];
    c_q_apriori[5] = q_apriori[0];
    c_q_apriori[6] = q_apriori[3];
    c_q_apriori[7] = -q_apriori[2];
    c_q_apriori[8] = q_apriori[2];
    c_q_apriori[9] = -q_apriori[3];
    c_q_apriori[10] = q_apriori[0];
    c_q_apriori[11] = q_apriori[1];
    c_q_apriori[12] = q_apriori[3];
    c_q_apriori[13] = q_apriori[2];
    c_q_apriori[14] = -q_apriori[1];
    c_q_apriori[15] = q_apriori[0];
    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        b_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          b_q_apriori[i + (r1 << 2)] += c_y[i + (r2 << 2)] * (float)a[r2 + (r1 <<
            2)];
        }
      }
    }

    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        d_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          d_q_apriori[i + (r1 << 2)] += c_q_apriori[i + (r2 << 2)] *
            b_q_apriori[r2 + (r1 << 2)];
        }
      }
    }

    for (i = 0; i < 3; i++) {
      gyro_input[i] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        gyro_input[i] += fv1[r1] * d_q_apriori[r1 + (i << 2)];
      }

      gyro_bias[i] = gyro_input[i];
    }

    //  Measurement Jacobian
    // 'QEKF:112' H = [-devec * (Gamma(Gamma(q_apriori)*[0;0;0;-g])*I_conj + Phi(Phi(q_apriori)'*[0;0;0;-g])), zeros(3,4), zeros(3,2)]; 
    fv1[0] = 0.0F;
    fv1[1] = 0.0F;
    fv1[2] = 0.0F;
    fv1[3] = -g;
    c_y[0] = q_apriori[0];
    c_y[1] = -q_apriori[1];
    c_y[2] = -q_apriori[2];
    c_y[3] = -q_apriori[3];
    c_y[4] = q_apriori[1];
    c_y[5] = q_apriori[0];
    c_y[6] = q_apriori[3];
    c_y[7] = -q_apriori[2];
    c_y[8] = q_apriori[2];
    c_y[9] = -q_apriori[3];
    c_y[10] = q_apriori[0];
    c_y[11] = q_apriori[1];
    c_y[12] = q_apriori[3];
    c_y[13] = q_apriori[2];
    c_y[14] = -q_apriori[1];
    c_y[15] = q_apriori[0];
    fv2[0] = 0.0F;
    fv2[1] = 0.0F;
    fv2[2] = 0.0F;
    fv2[3] = -g;
    c_q_apriori[0] = q_apriori[0];
    c_q_apriori[4] = -q_apriori[1];
    c_q_apriori[8] = -q_apriori[2];
    c_q_apriori[12] = -q_apriori[3];
    c_q_apriori[1] = q_apriori[1];
    c_q_apriori[5] = q_apriori[0];
    c_q_apriori[9] = -q_apriori[3];
    c_q_apriori[13] = q_apriori[2];
    c_q_apriori[2] = q_apriori[2];
    c_q_apriori[6] = q_apriori[3];
    c_q_apriori[10] = q_apriori[0];
    c_q_apriori[14] = -q_apriori[1];
    c_q_apriori[3] = q_apriori[3];
    c_q_apriori[7] = -q_apriori[2];
    c_q_apriori[11] = q_apriori[1];
    c_q_apriori[15] = q_apriori[0];
    for (i = 0; i < 4; i++) {
      c_gyro_input[i] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        c_gyro_input[i] += fv1[r1] * c_y[r1 + (i << 2)];
      }

      y[i] = c_gyro_input[i];
      q_apriori[i] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        q_apriori[i] += fv2[r1] * c_q_apriori[r1 + (i << 2)];
      }

      q[i] = q_apriori[i];
    }

    c_y[0] = y[0];
    c_y[1] = -y[1];
    c_y[2] = -y[2];
    c_y[3] = -y[3];
    c_y[4] = y[1];
    c_y[5] = y[0];
    c_y[6] = y[3];
    c_y[7] = -y[2];
    c_y[8] = y[2];
    c_y[9] = -y[3];
    c_y[10] = y[0];
    c_y[11] = y[1];
    c_y[12] = y[3];
    c_y[13] = y[2];
    c_y[14] = -y[1];
    c_y[15] = y[0];
    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 4; r1++) {
        c_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          c_q_apriori[i + (r1 << 2)] += (float)b_b[i + (r2 << 2)] * c_y[r2 + (r1
            << 2)];
        }
      }
    }

    b_q[0] = q[0];
    b_q[1] = -q[1];
    b_q[2] = -q[2];
    b_q[3] = -q[3];
    b_q[4] = q[1];
    b_q[5] = q[0];
    b_q[6] = -q[3];
    b_q[7] = q[2];
    b_q[8] = q[2];
    b_q[9] = q[3];
    b_q[10] = q[0];
    b_q[11] = -q[1];
    b_q[12] = q[3];
    b_q[13] = -q[2];
    b_q[14] = q[1];
    b_q[15] = q[0];
    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 4; r1++) {
        c_y[r1 + (i << 2)] = c_q_apriori[r1 + (i << 2)] + b_q[r1 + (i << 2)];
      }
    }

    for (i = 0; i < 4; i++) {
      for (r1 = 0; r1 < 3; r1++) {
        b_q_apriori[i + (r1 << 2)] = 0.0F;
        for (r2 = 0; r2 < 4; r2++) {
          b_q_apriori[i + (r1 << 2)] += c_y[i + (r2 << 2)] * (float)a[r2 + (r1 <<
            2)];
        }
      }
    }

    for (i = 0; i < 3; i++) {
      for (r1 = 0; r1 < 4; r1++) {
        H[r1 + 10 * i] = b_q_apriori[r1 + (i << 2)];
        H[(r1 + 10 * i) + 4] = 0.0F;
      }

      for (r1 = 0; r1 < 2; r1++) {
        H[(r1 + 10 * i) + 8] = 0.0F;
      }
    }
  }

  //  Calculate Kalman gain
  // 'QEKF:116' S = H * P_apriori * H' + R;
  for (i = 0; i < 10; i++) {
    for (r1 = 0; r1 < 3; r1++) {
      K[i + 10 * r1] = 0.0F;
      for (r2 = 0; r2 < 10; r2++) {
        K[i + 10 * r1] += P_apriori[i + 10 * r2] * H[r2 + 10 * r1];
      }
    }
  }

  // K = P_apriori * H' * inv(S);
  // 'QEKF:118' K = P_apriori * H' / S;
  for (i = 0; i < 3; i++) {
    for (r1 = 0; r1 < 3; r1++) {
      maxval = 0.0F;
      for (r2 = 0; r2 < 10; r2++) {
        maxval += H[r2 + 10 * i] * K[r2 + 10 * r1];
      }

      S[i + 3 * r1] = maxval + cov_acc[i + 3 * r1];
    }

    for (r1 = 0; r1 < 10; r1++) {
      d_y[i + 3 * r1] = 0.0F;
      for (r2 = 0; r2 < 10; r2++) {
        d_y[i + 3 * r1] += H[r2 + 10 * i] * P_apriori[r2 + 10 * r1];
      }
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = (float)fabs((double)S[0]);
  norm_acc = (float)fabs((double)S[3]);
  if (norm_acc > maxval) {
    maxval = norm_acc;
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
  for (i = 0; i < 10; i++) {
    K[r1 + 3 * i] = d_y[3 * i] / S[3 * r1];
    K[r2 + 3 * i] = d_y[1 + 3 * i] - K[r1 + 3 * i] * S[1 + 3 * r1];
    K[r3 + 3 * i] = d_y[2 + 3 * i] - K[r1 + 3 * i] * S[2 + 3 * r1];
    K[r2 + 3 * i] /= S[1 + 3 * r2];
    K[r3 + 3 * i] -= K[r2 + 3 * i] * S[2 + 3 * r2];
    K[r3 + 3 * i] /= S[2 + 3 * r3];
    K[r2 + 3 * i] -= K[r3 + 3 * i] * S[1 + 3 * r3];
    K[r1 + 3 * i] -= K[r3 + 3 * i] * S[3 * r3];
    K[r1 + 3 * i] -= K[r2 + 3 * i] * S[3 * r2];
  }

  //  Correct using innovation
  // 'QEKF:121' X_aposteriori = X_apriori + K * (z - z_hat);
  for (i = 0; i < 3; i++) {
    gyro_input[i] = z[i] - gyro_bias[i];
  }

  for (i = 0; i < 10; i++) {
    b_z[i] = 0.0F;
    for (r1 = 0; r1 < 3; r1++) {
      b_z[i] += gyro_input[r1] * K[r1 + 3 * i];
    }

    X_out[i] = X_apriori[i] + b_z[i];
  }

  // 'QEKF:122' P_aposteriori = (eye(10) - K*H) * P_apriori;
  memset(&I[0], 0, 100U * sizeof(signed char));
  for (i = 0; i < 10; i++) {
    I[i + 10 * i] = 1;
  }

  for (i = 0; i < 10; i++) {
    for (r1 = 0; r1 < 10; r1++) {
      maxval = 0.0F;
      for (r2 = 0; r2 < 3; r2++) {
        maxval += H[i + 10 * r2] * K[r2 + 3 * r1];
      }

      F_prev[i + 10 * r1] = (float)I[i + 10 * r1] - maxval;
    }
  }

  for (i = 0; i < 10; i++) {
    for (r1 = 0; r1 < 10; r1++) {
      P_out[i + 10 * r1] = 0.0F;
      for (r2 = 0; r2 < 10; r2++) {
        P_out[i + 10 * r1] += P_apriori[i + 10 * r2] * F_prev[r2 + 10 * r1];
      }
    }
  }

  //     %% Normalize quaternion
  // 'QEKF:125' X_aposteriori(1:4) = X_aposteriori(1:4) / norm(X_aposteriori(1:4)); 
  maxval = 0.0F;
  scale = 1.29246971E-26F;
  for (i = 0; i < 4; i++) {
    absxk = (float)fabs((double)X_out[i % 4]);
    if (absxk > scale) {
      t = scale / absxk;
      maxval = 1.0F + maxval * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      maxval += t * t;
    }
  }

  mw_arm_sqrt_f32(maxval, &f1);
  maxval = scale * f1;
  for (i = 0; i < 4; i++) {
    X_out[i] /= maxval;
  }

  //     %% Send output to Simulink
  // 'QEKF:128' X_out = X_aposteriori;
  // 'QEKF:129' P_out = P_aposteriori;
}

//
// File trailer for QEKF.cpp
//
// [EOF]
//
