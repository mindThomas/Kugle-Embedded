//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SlidingModeControl.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 20-Nov-2018 22:55:49
//
#ifndef SLIDINGMODECONTROL_H
#define SLIDINGMODECONTROL_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "SlidingModeControl_types.h"

// Function Declarations
extern void SlidingModeControl(const float X[12], const float q_ref[4], float Jk,
  float Mk, float rk, float Mb, float Jbx, float Jby, float Jbz, float Jw, float
  rw, float Bvk, float Bvm, float Bvb, float l, float g, float COM_X, float
  COM_Y, float COM_Z, const float K_vec[3], float eta, float epsilon, boolean_T
  ContinousSwitching, float tau[3], float S[3]);

#endif

//
// File trailer for SlidingModeControl.h
//
// [EOF]
//
