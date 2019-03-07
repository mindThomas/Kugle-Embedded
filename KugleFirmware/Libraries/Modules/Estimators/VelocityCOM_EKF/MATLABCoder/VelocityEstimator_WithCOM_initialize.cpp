//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator_WithCOM_initialize.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 05-Mar-2019 16:05:46
//

// Include Files
#include "rt_nonfinite.h"
#include "VelocityEstimator_WithCOM.h"
#include "VelocityEstimator_WithCOM_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void VelocityEstimator_WithCOM_initialize(const float P_diagonal_init[4], float X[4], float P[4*4])
{
  for (int i = 0; i < 4; i++) {
    X[i] = 0.f;
  }
  for (int i = 0; i < 4*4; i++) {
    P[i] = 0.f;
  }

  // Set the initial state vector
  // Leave as zero	

  // Set diagonal elements of the covariance P
  for (int i = 0; i < 4; i++) {
      P[5*i] = P_diagonal_init[i];
  }

  VelocityEstimator_WithCOM_init();
}

//
// File trailer for VelocityEstimator_WithCOM_initialize.cpp
//
// [EOF]
//
