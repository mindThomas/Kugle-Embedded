//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SteadyStateAcceleration.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 19-Nov-2018 11:57:59
//
#ifndef STEADYSTATEACCELERATION_H
#define STEADYSTATEACCELERATION_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "COMEstimator_types.h"

// Function Declarations
extern void SteadyStateAcceleration(float COM_X, float COM_Y, float COM_Z, float
  Jk, float Jw, float Mb, float Mk, float dx, float dy, float g, float q1, float
  q2, float q3, float q4, float rk, float rw, float acceleration[2]);

#endif

//
// File trailer for SteadyStateAcceleration.h
//
// [EOF]
//
