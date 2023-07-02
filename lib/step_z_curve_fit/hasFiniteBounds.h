//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: hasFiniteBounds.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

#ifndef HASFINITEBOUNDS_H
#define HASFINITEBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
bool hasFiniteBounds(bool hasLB[2], bool hasUB[2], const double lb[2],
                     const double ub[2]);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for hasFiniteBounds.h
//
// [EOF]
//
