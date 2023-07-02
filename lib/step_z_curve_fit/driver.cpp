//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: driver.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "driver.h"
#include "anonymous_function.h"
#include "anonymous_function1.h"
#include "rt_nonfinite.h"
#include "step_z_curve_fit_internal_types.h"
#include "step_z_curve_fit_internal_types1.h"
#include "step_z_curve_fit_internal_types11.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : c_struct_T &in1
//                const double in2[2]
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
void binary_expand_op(c_struct_T &in1, const double in2[2],
                      const coder::array<double, 2U> &in3)
{
  double b_in2;
  int in3_idx_0;
  int stride_0_0;
  int stride_1_0;
  b_in2 = in2[0];
  if (in3.size(1) == 1) {
    in3_idx_0 = in1.nonlin.workspace.fun.workspace.ydata.size(1);
  } else {
    in3_idx_0 = in3.size(1);
  }
  in1.cEq_1.set_size(in3_idx_0);
  stride_0_0 = (in3.size(1) != 1);
  stride_1_0 = (in1.nonlin.workspace.fun.workspace.ydata.size(1) != 1);
  for (int i{0}; i < in3_idx_0; i++) {
    in1.cEq_1[i] = (0.0 - 0.5 * (b_in2 * (in3[i * stride_0_0] - 1.0))) -
                   in1.nonlin.workspace.fun.workspace.ydata[i * stride_1_0];
  }
}

//
// Arguments    : const double gradf[2]
//                bool hasFiniteBounds
//                const double &projSteepestDescentInfNorm
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
double computeFirstOrderOpt(const double gradf[2], bool hasFiniteBounds,
                            const double &projSteepestDescentInfNorm)
{
  double firstOrderOpt;
  if (hasFiniteBounds) {
    double absx;
    double b;
    b = 0.0;
    absx = std::abs(gradf[0]);
    if (std::isnan(absx) || (absx > 0.0)) {
      b = absx;
    }
    absx = std::abs(gradf[1]);
    if (std::isnan(absx) || (absx > b)) {
      b = absx;
    }
    if ((std::abs(projSteepestDescentInfNorm - b) <
         2.2204460492503131E-16 * std::fmax(projSteepestDescentInfNorm, b)) ||
        (b == 0.0)) {
      firstOrderOpt = projSteepestDescentInfNorm;
    } else {
      firstOrderOpt =
          projSteepestDescentInfNorm * projSteepestDescentInfNorm / b;
    }
  } else {
    double absx;
    firstOrderOpt = 0.0;
    absx = std::abs(gradf[0]);
    if (std::isnan(absx) || (absx > 0.0)) {
      firstOrderOpt = absx;
    }
    absx = std::abs(gradf[1]);
    if (std::isnan(absx) || (absx > firstOrderOpt)) {
      firstOrderOpt = absx;
    }
  }
  return firstOrderOpt;
}

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for driver.cpp
//
// [EOF]
//
