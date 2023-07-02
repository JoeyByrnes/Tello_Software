//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: step_z_curve_fit_internal_types.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

#ifndef STEP_Z_CURVE_FIT_INTERNAL_TYPES_H
#define STEP_Z_CURVE_FIT_INTERNAL_TYPES_H

// Include Files
#include "anonymous_function.h"
#include "rtwtypes.h"
#include "step_z_curve_fit_types.h"
#include "coder_array.h"

// Type Definitions
struct c_struct_T {
  coder::b_anonymous_function nonlin;
  double f_1;
  coder::array<double, 1U> cEq_1;
  double f_2;
  coder::array<double, 1U> cEq_2;
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool isEmptyNonlcon;
  bool hasLB[2];
  bool hasUB[2];
  bool hasBounds;
  int FiniteDifferenceType;
};

#endif
//
// File trailer for step_z_curve_fit_internal_types.h
//
// [EOF]
//
