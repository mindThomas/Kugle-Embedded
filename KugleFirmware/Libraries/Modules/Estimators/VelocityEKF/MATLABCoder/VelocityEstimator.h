//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Nov-2018 11:57:59
//
#ifndef VELOCITYESTIMATOR_H
#define VELOCITYESTIMATOR_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_types.h"

// Function Declarations
extern void VelocityEstimator(const float X[2], const float P_prev[4], const
  float EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16],
  const float qdotQEKF[4], float SamplePeriod, float n_gear, float n_ticksRev,
  float Jk, float Mk, float rk, float Mb, float Jbx, float Jby, float Jbz, float
  Jw, float rw, float Bvk, float Bvm, float Bvb, float l, float g, const float
  COM[3], float Var_COM, float eta_qQEKF_velocity, float eta_dqQEKF_encoder,
  float X_out[2], float P_out[4]);

#endif

//
// File trailer for VelocityEstimator.h
//
// [EOF]
//
