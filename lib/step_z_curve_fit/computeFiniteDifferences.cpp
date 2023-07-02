//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: computeFiniteDifferences.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "computeFiniteDifferences.h"
#include "anonymous_function.h"
#include "anonymous_function1.h"
#include "driver.h"
#include "rt_nonfinite.h"
#include "step_z_curve_fit_internal_types.h"
#include "step_z_curve_fit_internal_types1.h"
#include "step_z_curve_fit_internal_types11.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : c_struct_T &obj
//                const ::coder::array<double, 2U> &cEqCurrent
//                double xk[2]
//                ::coder::array<double, 2U> &JacCeqTrans
//                const double lb[2]
//                const double ub[2]
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
bool computeFiniteDifferences(c_struct_T &obj,
                              const ::coder::array<double, 2U> &cEqCurrent,
                              double xk[2],
                              ::coder::array<double, 2U> &JacCeqTrans,
                              const double lb[2], const double ub[2])
{
  array<double, 2U> y;
  bool evalOK;
  if (obj.isEmptyNonlcon) {
    evalOK = true;
  } else {
    int idx;
    bool exitg1;
    evalOK = true;
    obj.numEvals = 0;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx < 2)) {
      double deltaX;
      double lbDiff;
      double ubDiff;
      int k;
      int nx;
      bool guard1{false};
      bool modifiedStep;
      modifiedStep = false;
      deltaX = 1.4901161193847656E-8 *
               (1.0 - 2.0 * static_cast<double>(xk[idx] < 0.0)) *
               std::fmax(std::abs(xk[idx]), 1.0);
      if (obj.hasLB[idx] || obj.hasUB[idx]) {
        if (obj.hasLB[idx] && obj.hasUB[idx]) {
          lbDiff = deltaX;
          if ((lb[idx] != ub[idx]) && (xk[idx] >= lb[idx]) &&
              (xk[idx] <= ub[idx])) {
            ubDiff = xk[idx] + deltaX;
            if ((ubDiff > ub[idx]) || (ubDiff < lb[idx])) {
              lbDiff = -deltaX;
              modifiedStep = true;
              ubDiff = xk[idx] - deltaX;
              if ((ubDiff > ub[idx]) || (ubDiff < lb[idx])) {
                lbDiff = xk[idx] - lb[idx];
                ubDiff = ub[idx] - xk[idx];
                if (lbDiff <= ubDiff) {
                  lbDiff = -lbDiff;
                } else {
                  lbDiff = ubDiff;
                }
              }
            }
          }
          deltaX = lbDiff;
        } else if (obj.hasUB[idx]) {
          if ((xk[idx] <= ub[idx]) && (xk[idx] + deltaX > ub[idx])) {
            deltaX = -deltaX;
            modifiedStep = true;
          }
        } else if ((xk[idx] >= lb[idx]) && (xk[idx] + deltaX < lb[idx])) {
          deltaX = -deltaX;
          modifiedStep = true;
        }
      }
      evalOK = true;
      lbDiff = xk[idx];
      xk[idx] += deltaX;
      //  Function to fit the curve
      y.set_size(1, obj.nonlin.workspace.fun.workspace.xdata.size(1));
      ubDiff = xk[1];
      nx = obj.nonlin.workspace.fun.workspace.xdata.size(1);
      for (k = 0; k < nx; k++) {
        y[k] = 6.2831853071795862 *
               (obj.nonlin.workspace.fun.workspace.xdata[k] / ubDiff);
      }
      nx = y.size(1);
      for (k = 0; k < nx; k++) {
        y[k] = std::cos(y[k]);
      }
      if (y.size(1) == obj.nonlin.workspace.fun.workspace.ydata.size(1)) {
        ubDiff = xk[0];
        if (y.size(1) == 1) {
          nx = obj.nonlin.workspace.fun.workspace.ydata.size(1);
        } else {
          nx = y.size(1);
        }
        if (y.size(1) == 1) {
          k = obj.nonlin.workspace.fun.workspace.ydata.size(1);
        } else {
          k = y.size(1);
        }
        obj.cEq_1.set_size(k);
        for (k = 0; k < nx; k++) {
          obj.cEq_1[k] = (0.0 - 0.5 * (ubDiff * (y[k] - 1.0))) -
                         obj.nonlin.workspace.fun.workspace.ydata[k];
        }
      } else {
        binary_expand_op(obj, xk, y);
      }
      nx = 0;
      while (evalOK && (nx + 1 <= obj.mEq)) {
        evalOK = ((!std::isinf(obj.cEq_1[nx])) && (!std::isnan(obj.cEq_1[nx])));
        nx++;
      }
      xk[idx] = lbDiff;
      obj.f_1 = 0.0;
      obj.numEvals++;
      guard1 = false;
      if (!evalOK) {
        if (!modifiedStep) {
          deltaX = -deltaX;
          if (obj.hasLB[idx]) {
            ubDiff = xk[idx] + deltaX;
            if ((ubDiff >= lb[idx]) && obj.hasUB[idx] && (ubDiff <= ub[idx])) {
              modifiedStep = true;
            } else {
              modifiedStep = false;
            }
          } else {
            modifiedStep = false;
          }
          if ((!obj.hasBounds) || modifiedStep) {
            evalOK = true;
            lbDiff = xk[idx];
            xk[idx] += deltaX;
            //  Function to fit the curve
            y.set_size(1, obj.nonlin.workspace.fun.workspace.xdata.size(1));
            ubDiff = xk[1];
            nx = obj.nonlin.workspace.fun.workspace.xdata.size(1);
            for (k = 0; k < nx; k++) {
              y[k] = 6.2831853071795862 *
                     (obj.nonlin.workspace.fun.workspace.xdata[k] / ubDiff);
            }
            nx = y.size(1);
            for (k = 0; k < nx; k++) {
              y[k] = std::cos(y[k]);
            }
            if (y.size(1) == obj.nonlin.workspace.fun.workspace.ydata.size(1)) {
              ubDiff = xk[0];
              if (y.size(1) == 1) {
                nx = obj.nonlin.workspace.fun.workspace.ydata.size(1);
              } else {
                nx = y.size(1);
              }
              if (y.size(1) == 1) {
                k = obj.nonlin.workspace.fun.workspace.ydata.size(1);
              } else {
                k = y.size(1);
              }
              obj.cEq_1.set_size(k);
              for (k = 0; k < nx; k++) {
                obj.cEq_1[k] = (0.0 - 0.5 * (ubDiff * (y[k] - 1.0))) -
                               obj.nonlin.workspace.fun.workspace.ydata[k];
              }
            } else {
              binary_expand_op(obj, xk, y);
            }
            nx = 0;
            while (evalOK && (nx + 1 <= obj.mEq)) {
              evalOK = ((!std::isinf(obj.cEq_1[nx])) &&
                        (!std::isnan(obj.cEq_1[nx])));
              nx++;
            }
            xk[idx] = lbDiff;
            obj.f_1 = 0.0;
            obj.numEvals++;
          }
        }
        if (!evalOK) {
          exitg1 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        k = obj.mEq;
        for (nx = 0; nx < k; nx++) {
          JacCeqTrans[idx + (nx << 1)] =
              (obj.cEq_1[nx] - cEqCurrent[nx]) / deltaX;
        }
        idx++;
      }
    }
  }
  return evalOK;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFiniteDifferences.cpp
//
// [EOF]
//
