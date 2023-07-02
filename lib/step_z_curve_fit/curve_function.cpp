//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: curve_function.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "curve_function.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[2]
//                const coder::array<double, 2U> &xdata
//                coder::array<double, 2U> &y
// Return Type  : void
//
void curve_function(const double x[2], const coder::array<double, 2U> &xdata,
                    coder::array<double, 2U> &y)
{
  int nx;
  //  Function to fit the curve
  y.set_size(1, xdata.size(1));
  nx = xdata.size(1);
  for (int k{0}; k < nx; k++) {
    y[k] = 6.2831853071795862 * (xdata[k] / x[1]);
  }
  nx = y.size(1);
  for (int k{0}; k < nx; k++) {
    y[k] = std::cos(y[k]);
  }
  y.set_size(1, y.size(1));
  nx = y.size(1) - 1;
  for (int k{0}; k <= nx; k++) {
    y[k] = 0.0 - 0.5 * (x[0] * (y[k] - 1.0));
  }
}

//
// File trailer for curve_function.cpp
//
// [EOF]
//
