//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF_initialize.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 29-Oct-2018 20:49:17
//
#ifndef COMESTIMATOR_INITIALIZE_H
#define COMESTIMATOR_INITIALIZE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEstimator_types.h"

// Function Declarations
extern void COMEstimator_initialize(const float P_diagonal_init[2], float X[2], float P[2*2]);

#endif

//
// File trailer for COMEstimator_initialize.h
//
// [EOF]
//
