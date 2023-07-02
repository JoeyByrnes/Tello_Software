//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: _coder_step_z_curve_fit_api.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

#ifndef _CODER_STEP_Z_CURVE_FIT_API_H
#define _CODER_STEP_Z_CURVE_FIT_API_H

// Include Files
#include "coder_array_mex.h"
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void step_z_curve_fit(coder::array<real_T, 2U> *xdata,
                      coder::array<real_T, 2U> *ydata, real_T x0[2],
                      real_T lb[2], real_T ub[2], real_T *AH, real_T *end_time);

void step_z_curve_fit_api(const mxArray *const prhs[5], int32_T nlhs,
                          const mxArray *plhs[2]);

void step_z_curve_fit_atexit();

void step_z_curve_fit_initialize();

void step_z_curve_fit_terminate();

void step_z_curve_fit_xil_shutdown();

void step_z_curve_fit_xil_terminate();

#endif
//
// File trailer for _coder_step_z_curve_fit_api.h
//
// [EOF]
//
