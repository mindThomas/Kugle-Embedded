/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "Matrix.h"
#include <math.h>
#include <stdlib.h>
#include "Debug.h"
 
Matrix::Matrix()
{
	
}

Matrix::~Matrix()
{
	
}


void Matrix_Extract(const float * in, const int in_rows, const int in_cols, const int in_row, const int in_col, const int out_rows, const int out_cols, float * out)
{
    /*assert(out_rows <= in_rows);
    assert(out_cols <= in_cols);
    assert((in_row + out_rows) < in_rows);
    assert((in_col + out_cols) < in_cols);*/
    for (int m = 0; m < out_rows; m++)
    {
        for (int n = 0; n < out_cols; n++) {
            out[m*out_cols + n] = in[(in_row+m)*in_cols + (in_col+n)];
        }
    }
}

void Matrix_Round(float * matrix, int rows, int cols)
{
  for (int m = 0; m < rows; m++) {
    for (int n = 0; n < cols; n++) {
      matrix[cols*m + n] = roundf(matrix[cols*m + n] * 10) / 10;
      if (matrix[cols*m + n] == -0.f) {
        matrix[cols*m + n] = 0.f;
      }
    }
  }
}

void Matrix_Print(float * matrix, int rows, int cols)
{
  for (int m = 0; m < rows; m++) {
    Debug::print(" ");
    for (int n = 0; n < cols; n++) {
    	Debug::printf("%8.4f ", matrix[cols*m + n]);
    }
    Debug::print("\n");
  }
}
