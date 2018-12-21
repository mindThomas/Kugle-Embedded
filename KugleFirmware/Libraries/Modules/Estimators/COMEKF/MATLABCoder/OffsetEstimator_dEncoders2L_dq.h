//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OffsetEstimator_dEncoders2L_dq.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Nov-2018 11:57:59
//
#ifndef OFFSETESTIMATOR_DENCODERS2L_DQ_H
#define OFFSETESTIMATOR_DENCODERS2L_DQ_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_types.h"

// Function Declarations
extern void OffsetEstimator_dEncoders2L_dq(float dq1, float dq2, float dq3,
  float dq4, float dt, float dx_2L, float dy_2L, float l, float n_gear, float
  n_ticksRev, float q1, float q2, float q3, float q4, float rk, float rw, float
  Hencoder[12]);

#endif

//
// File trailer for OffsetEstimator_dEncoders2L_dq.h
//
// [EOF]
//
