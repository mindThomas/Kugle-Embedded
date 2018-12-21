//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 29-Oct-2018 20:49:17
//

// Include Files
#include <math.h>
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include "QEKF.h"
#include "norm.h"

// Function Definitions

//
// Arguments    : const float x[3]
// Return Type  : float
//
float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float f1;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 3; k++) {
    absxk = (float)fabs((double)x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  mw_arm_sqrt_f32(y, &f1);
  return scale * f1;
}

//
// File trailer for norm.cpp
//
// [EOF]
//
