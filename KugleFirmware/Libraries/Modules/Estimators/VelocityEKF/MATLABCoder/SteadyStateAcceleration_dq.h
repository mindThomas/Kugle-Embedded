//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SteadyStateAcceleration_dq.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Nov-2018 11:57:59
//
#ifndef STEADYSTATEACCELERATION_DQ_H
#define STEADYSTATEACCELERATION_DQ_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_types.h"

// Function Declarations
extern void SteadyStateAcceleration_dq(float COM_X, float COM_Y, float COM_Z,
  float Jk, float Mb, float Mk, float g, float q1, float q2, float q3, float q4,
  float rk, float dAcceleration_dq[8]);

#endif

//
// File trailer for SteadyStateAcceleration_dq.h
//
// [EOF]
//
