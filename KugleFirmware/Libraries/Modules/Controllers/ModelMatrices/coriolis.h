//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: coriolis.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 20-Nov-2018 22:55:49
//
#ifndef CORIOLIS_H
#define CORIOLIS_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"

// Function Declarations
extern void coriolis(float COM_X, float COM_Y, float COM_Z, float Jbx, float Jby,
                     float Jbz, float Jw, float Mb, double beta, float dq1,
                     float dq2, float dq3, float dq4, float dx, float dy, float
                     q1, float q2, float q3, float q4, float rk, float rw, float
                     C[36]);

#endif

//
// File trailer for coriolis.h
//
// [EOF]
//
