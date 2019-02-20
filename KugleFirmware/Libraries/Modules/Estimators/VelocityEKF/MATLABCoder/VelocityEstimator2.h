//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator2.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Feb-2019 12:31:44
//
#ifndef VELOCITYESTIMATOR2_H
#define VELOCITYESTIMATOR2_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_types.h"

// Function Declarations
extern void VelocityEstimator2(const float X[2], const float P_prev[4], const
  float EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16],
  const float qdotQEKF[4], float SamplePeriod, float TicksPrRev, float Jk, float
  Mk, float rk, float Mb, float Jw, float rw, float l, float g, const float COM
  [3], float CoR, float Var_COM, float eta_encoder, float X_out[2], float P_out
  [4]);

#endif

//
// File trailer for VelocityEstimator2.h
//
// [EOF]
//
