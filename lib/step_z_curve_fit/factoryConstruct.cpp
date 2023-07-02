//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: factoryConstruct.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "factoryConstruct.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "step_z_curve_fit_internal_types.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const b_anonymous_function &nonlin
//                int mCeq
//                const double lb[2]
//                const double ub[2]
//                c_struct_T &obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const b_anonymous_function &nonlin, int mCeq,
                      const double lb[2], const double ub[2], c_struct_T &obj)
{
  int idx;
  bool b;
  obj.nonlin = nonlin;
  obj.f_1 = 0.0;
  obj.cEq_1.set_size(mCeq);
  obj.f_2 = 0.0;
  obj.cEq_2.set_size(mCeq);
  obj.nVar = 2;
  obj.mIneq = 0;
  obj.mEq = mCeq;
  obj.numEvals = 0;
  obj.SpecifyObjectiveGradient = false;
  obj.SpecifyConstraintGradient = false;
  obj.isEmptyNonlcon = (mCeq == 0);
  obj.FiniteDifferenceType = 0;
  b = false;
  idx = 0;
  while ((!b) && (idx + 1 <= 2)) {
    obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    if (obj.hasLB[idx] || obj.hasUB[idx]) {
      b = true;
    }
    idx++;
  }
  while (idx + 1 <= 2) {
    obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    idx++;
  }
  obj.hasBounds = b;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factoryConstruct.cpp
//
// [EOF]
//
