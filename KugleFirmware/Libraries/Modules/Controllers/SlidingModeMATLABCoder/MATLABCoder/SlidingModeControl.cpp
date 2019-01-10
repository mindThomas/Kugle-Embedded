//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SlidingModeControl.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 20-Nov-2018 22:55:49
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "SlidingModeControl.h"
#include "friction.h"
#include "gravity.h"
#include "coriolis.h"
#include "input_forces.h"
#include "inv.h"
#include "mass.h"

// Function Definitions

//
// function [tau, S] = SlidingModeControl(X,q_ref,Jk,Mk,rk,Mb,Jbx,Jby,Jbz,Jw,rw,Bvk,Bvm,Bvb,l,g,COM_X,COM_Y,COM_Z,K_vec,eta,epsilon, ContinousSwitching)
// for q o p = Phi(q) * p
// Arguments    : const float X[12]
//                const float q_ref[4]
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
//                float COM_X
//                float COM_Y
//                float COM_Z
//                const float K_vec[3]
//                float eta
//                float epsilon
//                boolean_T ContinousSwitching
//                float tau[3]
//                float S[3]
// Return Type  : void
//
__attribute__((optimize("O3"))) void SlidingModeControl(const float X[12], const float q_ref[4], float Jk, float
  Mk, float rk, float Mb, float Jbx, float Jby, float Jbz, float Jw, float rw,
  float Bvk, float Bvm, float Bvb, float, float g, float COM_X, float COM_Y,
  float COM_Z, const float K_vec[3], float eta, float epsilon, boolean_T
  ContinousSwitching, float tau[3], float S[3])
{
  float fv0[36];
  float Minv[36];
  float b_q_ref[16];
  int p1;
  float b_X[4];
  float q_err[4];
  int p2;
  float fv1[18];
  float b_Minv[24];
  int p3;
  static const signed char iv0[24] = { 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  float c_q_ref[12];
  float c_X[12];
  float x[9];
  float InputInv[9];
  static const signed char iv1[12] = { 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2 };

  float absx11;
  float absx21;
  float absx31;
  int itmp;
  float d_X[3];
  float satS[3];
  static const signed char iv2[12] = { 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  float tau_switching[3];
  float b_eta[3];
  float fv2[36];
  float fv3[6];
  float fv4[6];
  float e_X[6];
  static const signed char iv3[12] = { 0, -2, 0, 0, 0, 0, -2, 0, 0, 0, 0, -2 };

  float f_X[6];
  float fv5[36];
  float g_X[3];

  // 'SlidingModeControl:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p 
  // 'SlidingModeControl:4'               q(2) q(1)  -q(4) q(3);
  // 'SlidingModeControl:5'               q(3) q(4)  q(1)  -q(2);
  // 'SlidingModeControl:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'SlidingModeControl:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q 
  // 'SlidingModeControl:8'                  p(2) p(1) p(4) -p(3);
  // 'SlidingModeControl:9'                  p(3) -p(4) p(1) p(2);
  // 'SlidingModeControl:10'                  p(4) p(3) -p(2) p(1)];
  // 'SlidingModeControl:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'SlidingModeControl:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'SlidingModeControl:15' sat = @(x) min(max(x, -1), 1);
  // 'SlidingModeControl:17' beta = 0;
  // 'SlidingModeControl:19' x = X(1);
  // 'SlidingModeControl:20' y = X(2);
  // 'SlidingModeControl:21' q1 = X(3);
  // 'SlidingModeControl:22' q2 = X(4);
  // 'SlidingModeControl:23' q3 = X(5);
  // 'SlidingModeControl:24' q4 = X(6);
  // 'SlidingModeControl:26' dx = X(7);
  // 'SlidingModeControl:27' dy = X(8);
  // 'SlidingModeControl:28' dq1 = X(9);
  // 'SlidingModeControl:29' dq2 = X(10);
  // 'SlidingModeControl:30' dq3 = X(11);
  // 'SlidingModeControl:31' dq4 = X(12);
  // 'SlidingModeControl:33' q = X(3:6);
  // 'SlidingModeControl:34' dq = X(9:12);
  // 'SlidingModeControl:35' dchi = reshape(X(7:12), 6, 1);
  // M = mass(Jbx,Jby,Jbz,Jk,Mb,Mk,l,q1,q2,q3,q4,rk);
  // C = coriolis(Jbx,Jby,Jbz,Mb,beta,dq1,dq2,dq3,dq4,l,q1,q2,q3,q4);
  // G = gravity(Mb,beta,g,l,q1,q2,q3,q4);
  // D = friction(Bvb,Bvk,Bvm,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
  // Q = input_forces(q1,q2,q3,q4,rk,rw);
  // 'SlidingModeControl:42' M = mass(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,q1,q2,q3,q4,rk,rw); 
  // 'SlidingModeControl:43' C = coriolis(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jw,Mb,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw); 
  // 'SlidingModeControl:44' G = gravity(COM_X,COM_Y,COM_Z,Mb,beta,g,q1,q2,q3,q4); 
  // 'SlidingModeControl:45' D = friction(Bvb,Bvk,Bvm,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw); 
  // 'SlidingModeControl:46' Q = input_forces(q1,q2,q3,q4,rk,rw);
  // 'SlidingModeControl:48' Minv = inv(M);
  mass(COM_X, COM_Y, COM_Z, Jbx, Jby, Jbz, Jk, Jw, Mb, Mk, X[2], X[3], X[4], X[5],
       rk, rw, fv0);
  inv(fv0, Minv);

  // 'SlidingModeControl:49' fq = [zeros(4,2), eye(4)] * Minv * (-C*dchi - G - D); 
  // 'SlidingModeControl:50' gq = [zeros(4,2), eye(4)] * Minv * Q;
  // fq = [zeros(4,2), eye(4)] * (M \ (-C*dchi - G - D));
  // gq = [zeros(4,2), eye(4)] * (M \ Q);
  // SlidingManifold = 3; % 1=q_dot,  2=Body angular velocity,  3=Inertial angular velocity 
  //  Quaternion error based on reference input
  // 'SlidingModeControl:58' q_err = Gamma(q_ref)' * q;
  b_q_ref[0] = q_ref[0];
  b_q_ref[4] = -q_ref[1];
  b_q_ref[8] = -q_ref[2];
  b_q_ref[12] = -q_ref[3];
  b_q_ref[1] = q_ref[1];
  b_q_ref[5] = q_ref[0];
  b_q_ref[9] = q_ref[3];
  b_q_ref[13] = -q_ref[2];
  b_q_ref[2] = q_ref[2];
  b_q_ref[6] = -q_ref[3];
  b_q_ref[10] = q_ref[0];
  b_q_ref[14] = q_ref[1];
  b_q_ref[3] = q_ref[3];
  b_q_ref[7] = q_ref[2];
  b_q_ref[11] = -q_ref[1];
  b_q_ref[15] = q_ref[0];
  for (p1 = 0; p1 < 4; p1++) {
    b_X[p1] = 0.0F;
    for (p2 = 0; p2 < 4; p2++) {
      b_X[p1] += X[2 + p2] * b_q_ref[p2 + (p1 << 2)];
    }

    q_err[p1] = b_X[p1];
  }

  // 'SlidingModeControl:59' if (q_err(1) < 0)
  if (q_err[0] < 0.0F) {
    // 'SlidingModeControl:60' q_err = -q_err;
    for (p1 = 0; p1 < 4; p1++) {
      q_err[p1] = -q_err[p1];
    }

    //  opposite
  }

  //  %% Sliding manifold with q_dot
  //  if (SlidingManifold == 1)
  // InputInv = inv(devec*gq);
  // tau_eq = InputInv * (-K_vec.*(devec*dq) - devec*fq);
  // S = devec*dq + K_vec.*(devec*q_err);
  //  end
  //  Sliding manifold with omega
  //  if (SlidingManifold == 2 || SlidingManifold == 3)
  //      if (SlidingManifold == 2)
  //      % body angular velocity
  // InputInv = inv(2 * devec*Phi(q)' * gq);
  // tau_eq = InputInv * (-2*devec*Phi(dq)'*dq - 2*devec*Phi(q)'*fq - K_vec.*(devec*dq)); 
  // omeg = 2*devec*Phi(q)'*dq; % body angular velocity
  //      end
  // if (SlidingManifold == 3)
  //  inertial angular velocity
  // 'SlidingModeControl:83' InputInv = inv(2 * devec*Gamma(q)' * gq);
  input_forces(X[2], X[3], X[4], X[5], rk, rw, fv1);
  for (p1 = 0; p1 < 6; p1++) {
    for (p2 = 0; p2 < 4; p2++) {
      b_Minv[p1 + 6 * p2] = 0.0F;
      for (p3 = 0; p3 < 6; p3++) {
        b_Minv[p1 + 6 * p2] += Minv[p1 + 6 * p3] * (float)iv0[p3 + 6 * p2];
      }
    }
  }

  b_q_ref[0] = X[2];
  b_q_ref[4] = -X[3];
  b_q_ref[8] = -X[4];
  b_q_ref[12] = -X[5];
  b_q_ref[1] = X[3];
  b_q_ref[5] = X[2];
  b_q_ref[9] = X[5];
  b_q_ref[13] = -X[4];
  b_q_ref[2] = X[4];
  b_q_ref[6] = -X[5];
  b_q_ref[10] = X[2];
  b_q_ref[14] = X[3];
  b_q_ref[3] = X[5];
  b_q_ref[7] = X[4];
  b_q_ref[11] = -X[3];
  b_q_ref[15] = X[2];
  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 4; p2++) {
      c_q_ref[p1 + 3 * p2] = 0.0F;
      for (p3 = 0; p3 < 6; p3++) {
        c_q_ref[p1 + 3 * p2] += fv1[p1 + 3 * p3] * b_Minv[p3 + 6 * p2];
      }
    }
  }

  for (p1 = 0; p1 < 4; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      c_X[p1 + (p2 << 2)] = 0.0F;
      for (p3 = 0; p3 < 4; p3++) {
        c_X[p1 + (p2 << 2)] += b_q_ref[p1 + (p3 << 2)] * (float)iv1[p3 + (p2 <<
          2)];
      }
    }
  }

  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      InputInv[p1 + 3 * p2] = 0.0F;
      for (p3 = 0; p3 < 4; p3++) {
        InputInv[p1 + 3 * p2] += c_q_ref[p1 + 3 * p3] * c_X[p3 + (p2 << 2)];
      }
    }
  }

  for (p1 = 0; p1 < 9; p1++) {
    x[p1] = InputInv[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = (float)fabs((double)InputInv[0]);
  absx21 = (float)fabs((double)InputInv[3]);
  absx31 = (float)fabs((double)InputInv[6]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = InputInv[3];
    x[3] = InputInv[0];
    x[1] = InputInv[4];
    x[4] = InputInv[1];
    x[2] = InputInv[5];
    x[5] = InputInv[2];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      x[0] = InputInv[6];
      x[6] = InputInv[0];
      x[1] = InputInv[7];
      x[7] = InputInv[1];
      x[2] = InputInv[8];
      x[8] = InputInv[2];
    }
  }

  absx11 = x[3] / x[0];
  x[3] /= x[0];
  absx21 = x[6] / x[0];
  x[6] /= x[0];
  x[4] -= absx11 * x[1];
  x[7] -= absx21 * x[1];
  x[5] -= absx11 * x[2];
  x[8] -= absx21 * x[2];
  if ((float)fabs((double)x[7]) > (float)fabs((double)x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    x[3] = absx21;
    x[6] = absx11;
    absx11 = x[4];
    x[4] = x[7];
    x[7] = absx11;
    absx11 = x[5];
    x[5] = x[8];
    x[8] = absx11;
  }

  absx11 = x[7] / x[4];
  x[7] /= x[4];
  x[8] -= absx11 * x[5];
  absx11 = (x[7] * x[3] - x[6]) / x[8];
  absx21 = -(x[3] + x[5] * absx11) / x[4];
  InputInv[p1 % 3 * 3 + p1 / 3] = ((1.0F - x[1] * absx21) - x[2] * absx11) / x[0];
  InputInv[(p1 + 1) % 3 * 3 + (p1 + 1) / 3] = absx21;
  InputInv[(p1 + 2) % 3 * 3 + (p1 + 2) / 3] = absx11;
  absx11 = -x[7] / x[8];
  absx21 = (1.0F - x[5] * absx11) / x[4];
  InputInv[p2 % 3 * 3 + p2 / 3] = -(x[1] * absx21 + x[2] * absx11) / x[0];
  InputInv[(p2 + 1) % 3 * 3 + (p2 + 1) / 3] = absx21;
  InputInv[(p2 + 2) % 3 * 3 + (p2 + 2) / 3] = absx11;
  absx11 = 1.0F / x[8];
  absx21 = -x[5] * absx11 / x[4];
  InputInv[p3 % 3 * 3 + p3 / 3] = -(x[1] * absx21 + x[2] * absx11) / x[0];
  InputInv[(p3 + 1) % 3 * 3 + (p3 + 1) / 3] = absx21;
  InputInv[(p3 + 2) % 3 * 3 + (p3 + 2) / 3] = absx11;

  // 'SlidingModeControl:84' tau_eq = InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq - K_vec.*(devec*Gamma(q_ref)'*dq)); 
  // 'SlidingModeControl:85' omeg = 2*devec*Gamma(q)'*dq;
  //  inertial angular velocity
  // end
  // 'SlidingModeControl:88' S = omeg + K_vec.*(devec*q_err);
  b_q_ref[0] = X[2];
  b_q_ref[4] = -X[3];
  b_q_ref[8] = -X[4];
  b_q_ref[12] = -X[5];
  b_q_ref[1] = X[3];
  b_q_ref[5] = X[2];
  b_q_ref[9] = X[5];
  b_q_ref[13] = -X[4];
  b_q_ref[2] = X[4];
  b_q_ref[6] = -X[5];
  b_q_ref[10] = X[2];
  b_q_ref[14] = X[3];
  b_q_ref[3] = X[5];
  b_q_ref[7] = X[4];
  b_q_ref[11] = -X[3];
  b_q_ref[15] = X[2];
  for (p1 = 0; p1 < 4; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      c_X[p1 + (p2 << 2)] = 0.0F;
      for (p3 = 0; p3 < 4; p3++) {
        c_X[p1 + (p2 << 2)] += b_q_ref[p1 + (p3 << 2)] * (float)iv1[p3 + (p2 <<
          2)];
      }
    }
  }

  for (p1 = 0; p1 < 3; p1++) {
    d_X[p1] = 0.0F;
    satS[p1] = 0.0F;
    for (p2 = 0; p2 < 4; p2++) {
      d_X[p1] += X[8 + p2] * c_X[p2 + (p1 << 2)];
      satS[p1] += q_err[p2] * (float)iv2[p2 + (p1 << 2)];
    }

    S[p1] = d_X[p1] + K_vec[p1] * satS[p1];
  }

  // end
  //  %% Discontinous switching law
  // 'SlidingModeControl:93' if (~ContinousSwitching)
  if (!ContinousSwitching) {
    // 'SlidingModeControl:94' sgnS = sign(S);
    for (p1 = 0; p1 < 3; p1++) {
      satS[p1] = S[p1];
    }

    for (p1 = 0; p1 < 3; p1++) {
      absx11 = satS[p1];
      if (satS[p1] < 0.0F) {
        absx11 = -1.0F;
      } else if (satS[p1] > 0.0F) {
        absx11 = 1.0F;
      } else {
        if (satS[p1] == 0.0F) {
          absx11 = 0.0F;
        }
      }

      satS[p1] = absx11;
    }

    // 'SlidingModeControl:95' sgnS = sgnS + (sgnS==0);
    for (p1 = 0; p1 < 3; p1++) {
      satS[p1] += (float)(satS[p1] == 0.0F);
    }

    //  force S >= 0 to cause positive input
    // 'SlidingModeControl:96' u = -eta * sgnS;
    // 'SlidingModeControl:97' tau_switching = InputInv * u;
    for (p1 = 0; p1 < 3; p1++) {
      b_eta[p1] = 0.0F;
      for (p2 = 0; p2 < 3; p2++) {
        b_eta[p1] += -eta * satS[p2] * InputInv[p2 + 3 * p1];
      }

      tau_switching[p1] = b_eta[p1];
    }

    //  Continous switching law
  } else {
    // 'SlidingModeControl:100' else
    // 'SlidingModeControl:101' satS = sat(S/epsilon);
    for (p1 = 0; p1 < 3; p1++) {
      satS[p1] = S[p1] / epsilon;
    }

    for (p1 = 0; p1 < 3; p1++) {
      if (rtIsNaNF(satS[p1]) || (satS[p1] < -1.0F)) {
        tau_switching[p1] = -1.0F;
      } else {
        tau_switching[p1] = satS[p1];
      }
    }

    for (p1 = 0; p1 < 3; p1++) {
      if (rtIsNaNF(tau_switching[p1]) || (tau_switching[p1] > 1.0F)) {
        satS[p1] = 1.0F;
      } else {
        satS[p1] = tau_switching[p1];
      }
    }

    // 'SlidingModeControl:102' u = -eta * satS;
    // 'SlidingModeControl:103' tau_switching = InputInv * u;
    for (p1 = 0; p1 < 3; p1++) {
      b_eta[p1] = 0.0F;
      for (p2 = 0; p2 < 3; p2++) {
        b_eta[p1] += -eta * satS[p2] * InputInv[p2 + 3 * p1];
      }

      tau_switching[p1] = b_eta[p1];
    }
  }

  //  Control output
  // 'SlidingModeControl:107' tau = tau_eq + tau_switching;
  coriolis(COM_X, COM_Y, COM_Z, Jbx, Jby, Jbz, Jw, Mb, 0.0, X[8], X[9], X[10],
           X[11], X[6], X[7], X[2], X[3], X[4], X[5], rk, rw, fv2);
  gravity(COM_X, COM_Y, COM_Z, Mb, 0.0, g, X[2], X[3], X[4], X[5], fv3);
  friction(Bvb, Bvk, Bvm, 0.0, X[8], X[9], X[10], X[11], X[6], X[7], X[2], X[3],
           X[4], X[5], rk, rw, fv4);
  b_q_ref[0] = X[8];
  b_q_ref[4] = -X[9];
  b_q_ref[8] = -X[10];
  b_q_ref[12] = -X[11];
  b_q_ref[1] = X[9];
  b_q_ref[5] = X[8];
  b_q_ref[9] = X[11];
  b_q_ref[13] = -X[10];
  b_q_ref[2] = X[10];
  b_q_ref[6] = -X[11];
  b_q_ref[10] = X[8];
  b_q_ref[14] = X[9];
  b_q_ref[3] = X[11];
  b_q_ref[7] = X[10];
  b_q_ref[11] = -X[9];
  b_q_ref[15] = X[8];
  for (p1 = 0; p1 < 4; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      c_X[p1 + (p2 << 2)] = 0.0F;
      for (p3 = 0; p3 < 4; p3++) {
        c_X[p1 + (p2 << 2)] += b_q_ref[p1 + (p3 << 2)] * (float)iv3[p3 + (p2 <<
          2)];
      }
    }
  }

  for (p1 = 0; p1 < 3; p1++) {
    d_X[p1] = 0.0F;
    for (p2 = 0; p2 < 4; p2++) {
      d_X[p1] += X[8 + p2] * c_X[p2 + (p1 << 2)];
    }
  }

  for (p1 = 0; p1 < 6; p1++) {
    e_X[p1] = 0.0F;
    for (p2 = 0; p2 < 6; p2++) {
      fv5[p2 + 6 * p1] = -fv2[p2 + 6 * p1];
      e_X[p1] += X[6 + p2] * fv5[p2 + 6 * p1];
    }

    f_X[p1] = (e_X[p1] - fv3[p1]) - fv4[p1];
    for (p2 = 0; p2 < 4; p2++) {
      b_Minv[p1 + 6 * p2] = 0.0F;
      for (p3 = 0; p3 < 6; p3++) {
        b_Minv[p1 + 6 * p2] += Minv[p1 + 6 * p3] * (float)iv0[p3 + 6 * p2];
      }
    }
  }

  b_q_ref[0] = X[2];
  b_q_ref[4] = -X[3];
  b_q_ref[8] = -X[4];
  b_q_ref[12] = -X[5];
  b_q_ref[1] = X[3];
  b_q_ref[5] = X[2];
  b_q_ref[9] = X[5];
  b_q_ref[13] = -X[4];
  b_q_ref[2] = X[4];
  b_q_ref[6] = -X[5];
  b_q_ref[10] = X[2];
  b_q_ref[14] = X[3];
  b_q_ref[3] = X[5];
  b_q_ref[7] = X[4];
  b_q_ref[11] = -X[3];
  b_q_ref[15] = X[2];
  for (p1 = 0; p1 < 4; p1++) {
    b_X[p1] = 0.0F;
    for (p2 = 0; p2 < 6; p2++) {
      b_X[p1] += f_X[p2] * b_Minv[p2 + 6 * p1];
    }

    for (p2 = 0; p2 < 3; p2++) {
      c_X[p1 + (p2 << 2)] = 0.0F;
      for (p3 = 0; p3 < 4; p3++) {
        c_X[p1 + (p2 << 2)] += b_q_ref[p1 + (p3 << 2)] * (float)iv1[p3 + (p2 <<
          2)];
      }
    }
  }

  for (p1 = 0; p1 < 3; p1++) {
    satS[p1] = 0.0F;
    for (p2 = 0; p2 < 4; p2++) {
      satS[p1] += b_X[p2] * c_X[p2 + (p1 << 2)];
    }
  }

  b_q_ref[0] = q_ref[0];
  b_q_ref[4] = -q_ref[1];
  b_q_ref[8] = -q_ref[2];
  b_q_ref[12] = -q_ref[3];
  b_q_ref[1] = q_ref[1];
  b_q_ref[5] = q_ref[0];
  b_q_ref[9] = q_ref[3];
  b_q_ref[13] = -q_ref[2];
  b_q_ref[2] = q_ref[2];
  b_q_ref[6] = -q_ref[3];
  b_q_ref[10] = q_ref[0];
  b_q_ref[14] = q_ref[1];
  b_q_ref[3] = q_ref[3];
  b_q_ref[7] = q_ref[2];
  b_q_ref[11] = -q_ref[1];
  b_q_ref[15] = q_ref[0];
  for (p1 = 0; p1 < 4; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      c_q_ref[p1 + (p2 << 2)] = 0.0F;
      for (p3 = 0; p3 < 4; p3++) {
        c_q_ref[p1 + (p2 << 2)] += b_q_ref[p1 + (p3 << 2)] * (float)iv2[p3 + (p2
          << 2)];
      }
    }
  }

  for (p1 = 0; p1 < 3; p1++) {
    b_eta[p1] = 0.0F;
    for (p2 = 0; p2 < 4; p2++) {
      b_eta[p1] += X[8 + p2] * c_q_ref[p2 + (p1 << 2)];
    }

    g_X[p1] = (d_X[p1] - satS[p1]) - K_vec[p1] * b_eta[p1];
  }

  for (p1 = 0; p1 < 3; p1++) {
    d_X[p1] = 0.0F;
    for (p2 = 0; p2 < 3; p2++) {
      d_X[p1] += g_X[p2] * InputInv[p2 + 3 * p1];
    }

    tau[p1] = d_X[p1] + tau_switching[p1];
  }
}

//
// File trailer for SlidingModeControl.cpp
//
// [EOF]
//
