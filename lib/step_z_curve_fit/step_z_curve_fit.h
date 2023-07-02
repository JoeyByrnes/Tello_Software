//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: step_z_curve_fit.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

#ifndef STEP_Z_CURVE_FIT_H
#define STEP_Z_CURVE_FIT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void step_z_curve_fit(const coder::array<double, 2U> &xdata,
                             const coder::array<double, 2U> &ydata,
                             const double x0[2], const double lb[2],
                             const double ub[2], double *AH, double *end_time);

#endif
//
// File trailer for step_z_curve_fit.h
//
// [EOF]
//
