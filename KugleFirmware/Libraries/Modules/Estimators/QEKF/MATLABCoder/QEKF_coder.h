//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF_WithAllCorrections.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 22-Feb-2019 19:54:01
//
#ifndef QEKF_H
#define QEKF_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "QEKF_types.h"

// Function Declarations
extern void _QEKF(const float X[10], const float P_prev[100],
  const float Gyroscope[3], const float Accelerometer[3], float Heading,
  boolean_T UseHeadingForCorrection, float SamplePeriod, boolean_T
  BiasEstimationEnabled, boolean_T YawBiasEstimationEnabled, boolean_T
  NormalizeAccelerometer, const float cov_gyro[9], const float cov_acc[9], float
  GyroscopeTrustFactor, float sigma2_omega, float sigma2_heading, float
  sigma2_bias, float g, float X_out[10], float P_out[100]);

#endif

//
// File trailer for QEKF_WithAllCorrections.h
//
// [EOF]
//
