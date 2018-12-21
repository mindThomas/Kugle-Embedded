//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 20-Nov-2018 22:55:49
//

// Include Files
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "SlidingModeControl.h"
#include "inv.h"

// Function Definitions

//
// Arguments    : const float x[36]
//                float y[36]
// Return Type  : void
//
void inv(const float x[36], float y[36])
{
  int i12;
  float b_y[36];
  int iy;
  float b_x[36];
  int j;
  signed char ipiv[6];
  int c;
  int k;
  signed char p[6];
  int ix;
  float smax;
  float s;
  int jy;
  int i;
  for (i12 = 0; i12 < 6; i12++) {
    for (iy = 0; iy < 6; iy++) {
      b_x[iy + 6 * i12] = x[i12 + 6 * iy];
    }
  }

  memset(&b_y[0], 0, 36U * sizeof(float));
  for (i12 = 0; i12 < 6; i12++) {
    ipiv[i12] = (signed char)(1 + i12);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    smax = (float)fabs((double)b_x[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = (float)fabs((double)b_x[ix]);
      if (s > smax) {
        iy = k - 1;
        smax = s;
      }
    }

    if (b_x[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 6; k++) {
          smax = b_x[ix];
          b_x[ix] = b_x[iy];
          b_x[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i12 = (c - j) + 6;
      for (i = c + 1; i < i12; i++) {
        b_x[i] /= b_x[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (i = 1; i <= 5 - j; i++) {
      smax = b_x[jy];
      if (b_x[jy] != 0.0F) {
        ix = c + 1;
        i12 = (iy - j) + 12;
        for (k = 7 + iy; k < i12; k++) {
          b_x[k] += b_x[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (i12 = 0; i12 < 6; i12++) {
    p[i12] = (signed char)(1 + i12);
  }

  for (k = 0; k < 5; k++) {
    if (ipiv[k] > 1 + k) {
      iy = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (signed char)iy;
    }
  }

  for (k = 0; k < 6; k++) {
    c = p[k] - 1;
    b_y[k + 6 * (p[k] - 1)] = 1.0F;
    for (j = k; j + 1 < 7; j++) {
      if (b_y[j + 6 * c] != 0.0F) {
        for (i = j + 1; i + 1 < 7; i++) {
          b_y[i + 6 * c] -= b_y[j + 6 * c] * b_x[i + 6 * j];
        }
      }
    }
  }

  for (j = 0; j < 6; j++) {
    iy = 6 * j;
    for (k = 5; k >= 0; k--) {
      jy = 6 * k;
      if (b_y[k + iy] != 0.0F) {
        b_y[k + iy] /= b_x[k + jy];
        for (i = 0; i < k; i++) {
          b_y[i + iy] -= b_y[k + iy] * b_x[i + jy];
        }
      }
    }
  }

  for (i12 = 0; i12 < 6; i12++) {
    for (iy = 0; iy < 6; iy++) {
      y[iy + 6 * i12] = b_y[i12 + 6 * iy];
    }
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
