//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: step_z_curve_fit.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "step_z_curve_fit.h"
#include "anonymous_function.h"
#include "anonymous_function1.h"
#include "checkStoppingCriteria.h"
#include "computeFiniteDifferences.h"
#include "curve_function.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "hasFiniteBounds.h"
#include "linearLeastSquares.h"
#include "lsqcurvefit.h"
#include "projectBox.h"
#include "rt_nonfinite.h"
#include "step_z_curve_fit_internal_types.h"
#include "step_z_curve_fit_internal_types1.h"
#include "step_z_curve_fit_internal_types11.h"
#include "xgemv.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const coder::array<double, 2U> &xdata
//                const coder::array<double, 2U> &ydata
//                const double x0[2]
//                const double lb[2]
//                const double ub[2]
//                double *AH
//                double *end_time
// Return Type  : void
//
void step_z_curve_fit(const coder::array<double, 2U> &xdata,
                      const coder::array<double, 2U> &ydata, const double x0[2],
                      const double lb[2], const double ub[2], double *AH,
                      double *end_time)
{
  coder::b_anonymous_function b_this;
  coder::array<double, 2U> JacCeqTrans;
  coder::array<double, 2U> augJacobian;
  coder::array<double, 2U> b_varargout_f1;
  coder::array<double, 2U> fNew;
  coder::array<double, 2U> jacobian;
  coder::array<double, 2U> residual;
  coder::array<double, 2U> varargout_f1;
  coder::array<double, 2U> x;
  coder::array<double, 1U> rhs;
  c_struct_T FiniteDifferences;
  c_struct_T b_FiniteDifferences;
  double dx[2];
  double gradf[2];
  double popt[2];
  double varargin_1[2];
  double xp[2];
  double b_gamma;
  double b_x;
  double funDiff;
  double minWidth;
  double relFactor;
  double resnorm;
  double tolActive;
  int aIdx;
  int bIdx;
  int f_temp_size_idx_1;
  int funcCount;
  int i;
  int iter;
  int iy0_tmp;
  int m;
  int m_temp;
  bool hasLB[2];
  bool hasUB[2];
  bool indActive[2];
  bool exitg1;
  bool hasFiniteBounds;
  bool stepSuccessful;
  b_this.workspace.fun.workspace.xdata.set_size(1, xdata.size(1));
  aIdx = xdata.size(1);
  for (i = 0; i < aIdx; i++) {
    b_this.workspace.fun.workspace.xdata[i] = xdata[i];
  }
  b_this.workspace.fun.workspace.ydata.set_size(1, ydata.size(1));
  aIdx = ydata.size(1);
  for (i = 0; i < aIdx; i++) {
    b_this.workspace.fun.workspace.ydata[i] = ydata[i];
  }
  funDiff = rtInf;
  iter = 0;
  popt[0] = x0[0];
  indActive[0] = false;
  dx[0] = rtInf;
  varargin_1[0] = ub[0] - lb[0];
  popt[1] = x0[1];
  indActive[1] = false;
  dx[1] = rtInf;
  varargin_1[1] = ub[1] - lb[1];
  if ((varargin_1[0] > varargin_1[1]) ||
      (std::isnan(varargin_1[0]) && (!std::isnan(varargin_1[1])))) {
    minWidth = varargin_1[1];
  } else {
    minWidth = varargin_1[0];
  }
  hasFiniteBounds =
      coder::optim::coder::utils::hasFiniteBounds(hasLB, hasUB, lb, ub);
  if (hasFiniteBounds && (!(minWidth < 0.0))) {
    if (hasLB[0]) {
      popt[0] = std::fmax(lb[0], x0[0]);
    }
    if (hasUB[0]) {
      popt[0] = std::fmin(ub[0], popt[0]);
    }
    if (hasLB[1]) {
      popt[1] = std::fmax(lb[1], x0[1]);
    }
    if (hasUB[1]) {
      popt[1] = std::fmin(ub[1], popt[1]);
    }
  }
  curve_function(popt, xdata, x);
  if (x.size(1) == ydata.size(1)) {
    varargout_f1.set_size(1, x.size(1));
    aIdx = x.size(1);
    for (i = 0; i < aIdx; i++) {
      varargout_f1[i] = x[i] - ydata[i];
    }
  } else {
    minus(varargout_f1, x, ydata);
  }
  curve_function(popt, xdata, x);
  if (x.size(1) == 1) {
    f_temp_size_idx_1 = ydata.size(1);
  } else {
    f_temp_size_idx_1 = x.size(1);
  }
  m_temp = varargout_f1.size(1);
  jacobian.set_size(f_temp_size_idx_1, 2);
  m = f_temp_size_idx_1 - 1;
  residual.set_size(1, varargout_f1.size(1));
  fNew.set_size(1, varargout_f1.size(1));
  for (int b_i{0}; b_i <= m; b_i++) {
    residual[b_i] = varargout_f1[b_i];
  }
  augJacobian.set_size(varargout_f1.size(1) + 2, 2);
  rhs.set_size(varargout_f1.size(1) + 2);
  i = varargout_f1.size(1) - 1;
  for (int j{0}; j < 2; j++) {
    aIdx = j * m_temp;
    bIdx = j * (f_temp_size_idx_1 + 2);
    for (int b_i{0}; b_i <= i; b_i++) {
      augJacobian[bIdx + b_i] = jacobian[aIdx + b_i];
    }
  }
  resnorm = 0.0;
  if (varargout_f1.size(1) >= 1) {
    for (bIdx = 0; bIdx < f_temp_size_idx_1; bIdx++) {
      tolActive = residual[bIdx];
      resnorm += tolActive * tolActive;
    }
  }
  JacCeqTrans.set_size(2, residual.size(1));
  coder::optim::coder::utils::FiniteDifferences::factoryConstruct(
      b_this, f_temp_size_idx_1, lb, ub, FiniteDifferences);
  varargin_1[0] = popt[0];
  varargin_1[1] = popt[1];
  coder::optim::coder::utils::FiniteDifferences::computeFiniteDifferences(
      FiniteDifferences, residual, varargin_1, JacCeqTrans, lb, ub);
  aIdx = JacCeqTrans.size(1);
  for (bIdx = 0; bIdx < 2; bIdx++) {
    for (int j{0}; j < aIdx; j++) {
      augJacobian[j + augJacobian.size(0) * bIdx] = JacCeqTrans[bIdx + 2 * j];
    }
  }
  funcCount = FiniteDifferences.numEvals + 1;
  b_gamma = 0.01;
  augJacobian[varargout_f1.size(1)] = 0.0;
  augJacobian[varargout_f1.size(1) + 1] = 0.0;
  augJacobian[varargout_f1.size(1)] = 0.1;
  iy0_tmp = (varargout_f1.size(1) + 2) << 1;
  augJacobian[iy0_tmp - 2] = 0.0;
  augJacobian[iy0_tmp - 1] = 0.0;
  augJacobian[(varargout_f1.size(1) + augJacobian.size(0)) + 1] = 0.1;
  for (int j{0}; j < 2; j++) {
    aIdx = j * (f_temp_size_idx_1 + 2);
    bIdx = j * m_temp;
    for (int b_i{0}; b_i <= i; b_i++) {
      jacobian[bIdx + b_i] = augJacobian[aIdx + b_i];
    }
  }
  coder::internal::blas::xgemv(varargout_f1.size(1), jacobian,
                               varargout_f1.size(1), residual, gradf);
  varargin_1[0] = -gradf[0];
  varargin_1[1] = -gradf[1];
  tolActive = coder::optim::coder::levenbergMarquardt::projectBox(
      popt, varargin_1, lb, ub, hasLB, hasUB);
  b_x = coder::optim::coder::levenbergMarquardt::computeFirstOrderOpt(
      gradf, hasFiniteBounds, tolActive);
  relFactor = std::fmax(b_x, 1.0);
  stepSuccessful = true;
  if (minWidth < 0.0) {
    aIdx = -2;
  } else {
    aIdx = coder::optim::coder::levenbergMarquardt::checkStoppingCriteria(
        gradf, relFactor, FiniteDifferences.numEvals + 1, tolActive,
        hasFiniteBounds);
  }
  exitg1 = false;
  while ((!exitg1) && (aIdx == -5)) {
    double resnormNew;
    bool evalOK;
    bool guard1{false};
    x.set_size(1, residual.size(1));
    aIdx = residual.size(1);
    for (bIdx = 0; bIdx < aIdx; bIdx++) {
      x[bIdx] = -residual[bIdx];
    }
    for (bIdx = 0; bIdx <= m; bIdx++) {
      rhs[bIdx] = x[bIdx];
    }
    rhs[f_temp_size_idx_1] = 0.0;
    rhs[f_temp_size_idx_1 + 1] = 0.0;
    if (hasFiniteBounds) {
      varargin_1[0] = -gradf[0] / (b_gamma + 1.0);
      varargin_1[1] = -gradf[1] / (b_gamma + 1.0);
      tolActive = coder::optim::coder::levenbergMarquardt::projectBox(
          popt, varargin_1, lb, ub, hasLB, hasUB);
      tolActive = std::fmin(tolActive, minWidth / 2.0);
      for (int b_i{0}; b_i < 2; b_i++) {
        if (hasLB[b_i]) {
          if ((popt[b_i] - lb[b_i] <= tolActive) && (gradf[b_i] > 0.0)) {
            indActive[b_i] = true;
          } else {
            indActive[b_i] = false;
          }
        }
        if (hasUB[b_i]) {
          if (indActive[b_i] ||
              ((ub[b_i] - popt[b_i] <= tolActive) && (gradf[b_i] < 0.0))) {
            indActive[b_i] = true;
          } else {
            indActive[b_i] = false;
          }
        }
        if (indActive[b_i]) {
          aIdx = (m_temp + 2) * b_i;
          for (bIdx = 0; bIdx <= m; bIdx++) {
            augJacobian[aIdx + bIdx] = 0.0;
          }
        }
      }
    }
    coder::optim::coder::levenbergMarquardt::linearLeastSquares(
        augJacobian, rhs, dx, m_temp + 2);
    if (hasFiniteBounds) {
      if (indActive[0]) {
        dx[0] = -gradf[0] / (b_gamma + 1.0);
      }
      tolActive = popt[0] + dx[0];
      xp[0] = tolActive;
      if (hasLB[0]) {
        tolActive = std::fmax(lb[0], tolActive);
        xp[0] = tolActive;
      }
      if (hasUB[0]) {
        tolActive = std::fmin(ub[0], tolActive);
        xp[0] = tolActive;
      }
      if (indActive[1]) {
        dx[1] = -gradf[1] / (b_gamma + 1.0);
      }
      tolActive = popt[1] + dx[1];
      xp[1] = tolActive;
      if (hasLB[1]) {
        tolActive = std::fmax(lb[1], tolActive);
        xp[1] = tolActive;
      }
      if (hasUB[1]) {
        tolActive = std::fmin(ub[1], tolActive);
        xp[1] = tolActive;
      }
    } else {
      xp[0] = popt[0] + dx[0];
      xp[1] = popt[1] + dx[1];
    }
    curve_function(xp, xdata, x);
    if (x.size(1) == ydata.size(1)) {
      b_varargout_f1.set_size(1, x.size(1));
      aIdx = x.size(1);
      for (bIdx = 0; bIdx < aIdx; bIdx++) {
        b_varargout_f1[bIdx] = x[bIdx] - ydata[bIdx];
      }
    } else {
      minus(b_varargout_f1, x, ydata);
    }
    for (int b_i{0}; b_i <= m; b_i++) {
      fNew[b_i] = b_varargout_f1[b_i];
    }
    resnormNew = 0.0;
    if (m_temp >= 1) {
      for (bIdx = 0; bIdx <= m; bIdx++) {
        tolActive = fNew[bIdx];
        resnormNew += tolActive * tolActive;
      }
    }
    evalOK = true;
    for (int b_i{0}; b_i < m_temp; b_i++) {
      if (evalOK) {
        b_x = fNew[b_i];
        if (std::isinf(b_x) || std::isnan(b_x)) {
          evalOK = false;
        }
      } else {
        evalOK = false;
      }
    }
    funcCount++;
    guard1 = false;
    if ((resnormNew < resnorm) && evalOK) {
      iter++;
      funDiff = std::abs(resnormNew - resnorm) / resnorm;
      resnorm = resnormNew;
      residual.set_size(1, fNew.size(1));
      aIdx = fNew.size(1);
      for (bIdx = 0; bIdx < aIdx; bIdx++) {
        residual[bIdx] = fNew[bIdx];
      }
      JacCeqTrans.set_size(2, fNew.size(1));
      varargin_1[0] = xp[0];
      varargin_1[1] = xp[1];
      b_FiniteDifferences = FiniteDifferences;
      evalOK = coder::optim::coder::utils::FiniteDifferences::
          computeFiniteDifferences(b_FiniteDifferences, fNew, varargin_1,
                                   JacCeqTrans, lb, ub);
      funcCount += b_FiniteDifferences.numEvals;
      aIdx = JacCeqTrans.size(1);
      for (bIdx = 0; bIdx < 2; bIdx++) {
        for (int j{0}; j < aIdx; j++) {
          augJacobian[j + augJacobian.size(0) * bIdx] =
              JacCeqTrans[bIdx + 2 * j];
        }
      }
      for (int j{0}; j < 2; j++) {
        aIdx = j * (f_temp_size_idx_1 + 2);
        bIdx = j * m_temp;
        for (int b_i{0}; b_i <= i; b_i++) {
          jacobian[bIdx + b_i] = augJacobian[aIdx + b_i];
        }
      }
      if (evalOK) {
        popt[0] = xp[0];
        popt[1] = xp[1];
        if (stepSuccessful) {
          b_gamma *= 0.1;
        }
        stepSuccessful = true;
        guard1 = true;
      } else {
        exitg1 = true;
      }
    } else {
      b_gamma *= 10.0;
      stepSuccessful = false;
      for (int j{0}; j < 2; j++) {
        aIdx = j * m_temp;
        bIdx = j * (f_temp_size_idx_1 + 2);
        for (int b_i{0}; b_i <= i; b_i++) {
          augJacobian[bIdx + b_i] = jacobian[aIdx + b_i];
        }
      }
      guard1 = true;
    }
    if (guard1) {
      tolActive = std::sqrt(b_gamma);
      augJacobian[m_temp] = 0.0;
      augJacobian[m_temp + 1] = 0.0;
      augJacobian[m_temp] = tolActive;
      augJacobian[iy0_tmp - 2] = 0.0;
      augJacobian[iy0_tmp - 1] = 0.0;
      augJacobian[(m_temp + augJacobian.size(0)) + 1] = tolActive;
      coder::internal::blas::xgemv(m_temp, jacobian, m_temp, residual, gradf);
      varargin_1[0] = -gradf[0];
      varargin_1[1] = -gradf[1];
      tolActive = coder::optim::coder::levenbergMarquardt::projectBox(
          popt, varargin_1, lb, ub, hasLB, hasUB);
      aIdx = coder::optim::coder::levenbergMarquardt::b_checkStoppingCriteria(
          gradf, relFactor, funDiff, popt, dx, funcCount, stepSuccessful, iter,
          tolActive, hasFiniteBounds);
      if (aIdx != -5) {
        exitg1 = true;
      }
    }
  }
  *AH = popt[0];
  *end_time = popt[1];
}

//
// File trailer for step_z_curve_fit.cpp
//
// [EOF]
//
