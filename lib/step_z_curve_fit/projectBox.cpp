//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: projectBox.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "projectBox.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[2]
//                double dx[2]
//                const double lb[2]
//                const double ub[2]
//                const bool hasLB[2]
//                const bool hasUB[2]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
double projectBox(const double x[2], double dx[2], const double lb[2],
                  const double ub[2], const bool hasLB[2], const bool hasUB[2])
{
  if (hasLB[0]) {
    dx[0] = std::fmax(lb[0] - x[0], dx[0]);
  }
  if (hasUB[0]) {
    dx[0] = std::fmin(ub[0] - x[0], dx[0]);
  }
  if (hasLB[1]) {
    dx[1] = std::fmax(lb[1] - x[1], dx[1]);
  }
  if (hasUB[1]) {
    dx[1] = std::fmin(ub[1] - x[1], dx[1]);
  }
  return std::fmax(std::fmax(0.0, std::abs(dx[0])), std::abs(dx[1]));
}

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for projectBox.cpp
//
// [EOF]
//
