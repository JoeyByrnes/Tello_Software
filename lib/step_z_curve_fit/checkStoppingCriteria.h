//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: checkStoppingCriteria.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

#ifndef CHECKSTOPPINGCRITERIA_H
#define CHECKSTOPPINGCRITERIA_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
int b_checkStoppingCriteria(const double gradf[2], double relFactor,
                            double funDiff, const double x[2],
                            const double dx[2], int funcCount,
                            bool stepSuccessful, int &iter,
                            double projSteepestDescentInfNorm,
                            bool hasFiniteBounds);

int checkStoppingCriteria(const double gradf[2], double relFactor,
                          int funcCount, double projSteepestDescentInfNorm,
                          bool hasFiniteBounds);

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for checkStoppingCriteria.h
//
// [EOF]
//
