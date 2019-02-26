//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: COMEstimator.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 25-Feb-2019 11:02:50
//
#ifndef COMESTIMATOR_H
#define COMESTIMATOR_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_types.h"

// Function Declarations
extern void COMEstimator(const float X[2], const float P_prev[4], const float
  qQEKF[4], const float Cov_qQEKF[16], const float qdotQEKF[4], const float
  Velocity[2], const float VelocityDiff[2], const float Cov_Velocity_meas[4],
  float SamplePeriod, float Jk, float Mk, float rk, float Mb, float Jw, float rw,
  float l, float g, float CoR, float X_out[2], float P_out[4]);

#endif

//
// File trailer for COMEstimator.h
//
// [EOF]
//
