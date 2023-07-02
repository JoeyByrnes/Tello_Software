//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: hasFiniteBounds.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "hasFiniteBounds.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : bool hasLB[2]
//                bool hasUB[2]
//                const double lb[2]
//                const double ub[2]
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
bool hasFiniteBounds(bool hasLB[2], bool hasUB[2], const double lb[2],
                     const double ub[2])
{
  int idx;
  bool hasBounds;
  hasBounds = false;
  idx = 0;
  while ((!hasBounds) && (idx + 1 <= 2)) {
    hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    if (hasLB[idx] || hasUB[idx]) {
      hasBounds = true;
    }
    idx++;
  }
  while (idx + 1 <= 2) {
    hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    idx++;
  }
  return hasBounds;
}

} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for hasFiniteBounds.cpp
//
// [EOF]
//
