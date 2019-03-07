//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator_WithCOM.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 05-Mar-2019 16:05:46
//
#ifndef VELOCITYESTIMATOR_WITHCOM_H
#define VELOCITYESTIMATOR_WITHCOM_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_WithCOM_types.h"

// Function Declarations
extern void VelocityEstimator_WithCOM(const float X[4], const float P_prev[16],
  const float EncoderDiffMeas[3], const float qQEKF[4], const float Cov_qQEKF[16],
  const float qdotQEKF[4], float SamplePeriod, float TicksPrRev, float Jk, float
  Mk, float rk, float Mb, float Jw, float rw, float l, float g, float CoR, float
  Var_COM, float eta_encoder, boolean_T UseTiltForPrediction, boolean_T
  EstimateVelocityAtCoR, boolean_T EnableWheelSlipDetector, float
  WheelSlipAccelerationThreshold, float WheelSlipSetVelocityVariance, boolean_T
  estimateCOM, float X_out[4], float P_out[16]);
extern void VelocityEstimator_WithCOM_init();

#endif

//
// File trailer for VelocityEstimator_WithCOM.h
//
// [EOF]
//
