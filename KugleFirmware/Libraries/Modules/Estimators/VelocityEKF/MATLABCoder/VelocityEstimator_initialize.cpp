//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEstimator_initialize.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 29-Oct-2018 20:49:17
//

// Include Files
#include "rt_nonfinite.h"
#include "VelocityEstimator.h"
#include "VelocityEstimator2.h"
#include "VelocityEstimator_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void VelocityEstimator_initialize(const float P_diagonal_init[2], float X[2], float P[2*2])
{ 
  for (int i = 0; i < 2; i++) {
    X[i] = 0.f;
  }
  for (int i = 0; i < 2*2; i++) {
    P[i] = 0.f;
  }

  // Set the initial state vector
  // Leave as zero	

  // Set diagonal elements of the covariance P
  for (int i = 0; i < 2; i++) {
      P[3*i] = P_diagonal_init[i];
  }

  VelocityEstimator2_init();
}

//
// File trailer for QEKF_initialize.cpp
//
// [EOF]
//
