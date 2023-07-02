//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: linearLeastSquares.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "linearLeastSquares.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : ::coder::array<double, 2U> &lhs
//                ::coder::array<double, 1U> &rhs
//                double dx[2]
//                int m
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
void linearLeastSquares(::coder::array<double, 2U> &lhs,
                        ::coder::array<double, 1U> &rhs, double dx[2], int m)
{
  double jpvt[2];
  double tau_data[2];
  double work[2];
  double temp;
  int i;
  int ii;
  int ix;
  int iy;
  int j;
  int ma;
  int mmi;
  int nfxd;
  int temp_tmp;
  ma = lhs.size(0);
  jpvt[0] = 0.0;
  tau_data[0] = 0.0;
  jpvt[1] = 0.0;
  tau_data[1] = 0.0;
  nfxd = 0;
  for (j = 0; j < 2; j++) {
    if (jpvt[j] != 0.0) {
      nfxd++;
      if (j + 1 != nfxd) {
        ix = j * ma;
        iy = (nfxd - 1) * ma;
        for (int k{0}; k < m; k++) {
          temp_tmp = ix + k;
          temp = lhs[temp_tmp];
          i = iy + k;
          lhs[temp_tmp] = lhs[i];
          lhs[i] = temp;
        }
        jpvt[j] = jpvt[nfxd - 1];
        jpvt[nfxd - 1] = static_cast<double>(j) + 1.0;
      } else {
        jpvt[j] = static_cast<double>(j) + 1.0;
      }
    } else {
      jpvt[j] = static_cast<double>(j) + 1.0;
    }
    work[j] = 0.0;
  }
  iy = lhs.size(0);
  i = static_cast<unsigned char>(nfxd);
  for (int b_i{0}; b_i < i; b_i++) {
    ii = b_i * iy + b_i;
    mmi = m - b_i;
    if (b_i + 1 < m) {
      temp = lhs[ii];
      tau_data[b_i] = internal::reflapack::xzlarfg(mmi, temp, lhs, ii + 2);
      lhs[ii] = temp;
    } else {
      tau_data[1] = 0.0;
    }
    if (b_i + 1 < 2) {
      temp = lhs[ii];
      lhs[ii] = 1.0;
      internal::reflapack::xzlarf(mmi, 1, ii + 1, tau_data[0], lhs,
                                  (ii + iy) + 1, iy, work);
      lhs[ii] = temp;
    }
  }
  if (nfxd < 2) {
    double vn1[2];
    double vn2[2];
    ma = lhs.size(0);
    work[0] = 0.0;
    vn1[0] = 0.0;
    vn2[0] = 0.0;
    work[1] = 0.0;
    vn1[1] = 0.0;
    vn2[1] = 0.0;
    i = nfxd + 1;
    for (j = i; j < 3; j++) {
      temp = internal::blas::xnrm2(m - nfxd, lhs, (nfxd + (j - 1) * ma) + 1);
      vn1[j - 1] = temp;
      vn2[j - 1] = temp;
    }
    for (int b_i{i}; b_i < 3; b_i++) {
      int ip1;
      ip1 = b_i + 1;
      j = (b_i - 1) * ma;
      ii = (j + b_i) - 1;
      mmi = m - b_i;
      iy = -1;
      if ((3 - b_i > 1) && (std::abs(vn1[1]) > std::abs(vn1[b_i - 1]))) {
        iy = 0;
      }
      iy += b_i;
      if (iy + 1 != b_i) {
        ix = iy * ma;
        for (int k{0}; k < m; k++) {
          temp_tmp = ix + k;
          temp = lhs[temp_tmp];
          nfxd = j + k;
          lhs[temp_tmp] = lhs[nfxd];
          lhs[nfxd] = temp;
        }
        temp = jpvt[iy];
        jpvt[iy] = jpvt[b_i - 1];
        jpvt[b_i - 1] = temp;
        vn1[iy] = vn1[b_i - 1];
        vn2[iy] = vn2[b_i - 1];
      }
      if (b_i < m) {
        temp = lhs[ii];
        tau_data[b_i - 1] =
            internal::reflapack::xzlarfg(mmi + 1, temp, lhs, ii + 2);
        lhs[ii] = temp;
      } else {
        tau_data[1] = 0.0;
      }
      if (b_i < 2) {
        temp = lhs[ii];
        lhs[ii] = 1.0;
        internal::reflapack::xzlarf(mmi + 1, 2 - b_i, ii + 1, tau_data[0], lhs,
                                    (ii + ma) + 1, ma, work);
        lhs[ii] = temp;
      }
      for (j = ip1; j < 3; j++) {
        iy = b_i + ma;
        if (vn1[1] != 0.0) {
          double temp2;
          temp = std::abs(lhs[iy - 1]) / vn1[1];
          temp = 1.0 - temp * temp;
          if (temp < 0.0) {
            temp = 0.0;
          }
          temp2 = vn1[1] / vn2[1];
          temp2 = temp * (temp2 * temp2);
          if (temp2 <= 1.4901161193847656E-8) {
            if (b_i < m) {
              temp = internal::blas::xnrm2(mmi, lhs, iy + 1);
              vn1[1] = temp;
              vn2[1] = temp;
            } else {
              vn1[1] = 0.0;
              vn2[1] = 0.0;
            }
          } else {
            vn1[1] *= std::sqrt(temp);
          }
        }
      }
    }
  }
  iy = lhs.size(0);
  for (j = 0; j < 2; j++) {
    if (tau_data[j] != 0.0) {
      temp = rhs[j];
      i = j + 2;
      for (int b_i{i}; b_i <= iy; b_i++) {
        temp += lhs[(b_i + lhs.size(0) * j) - 1] * rhs[b_i - 1];
      }
      temp *= tau_data[j];
      if (temp != 0.0) {
        rhs[j] = rhs[j] - temp;
        for (int b_i{i}; b_i <= iy; b_i++) {
          rhs[b_i - 1] = rhs[b_i - 1] - lhs[(b_i + lhs.size(0) * j) - 1] * temp;
        }
      }
    }
  }
  dx[0] = rhs[0];
  dx[1] = rhs[1];
  for (j = 1; j >= 0; j--) {
    iy = j + j * m;
    dx[j] /= lhs[iy];
    for (int b_i{0}; b_i < j; b_i++) {
      dx[0] -= dx[j] * lhs[iy - 1];
    }
  }
  work[1] = dx[1];
  dx[static_cast<int>(jpvt[0]) - 1] = dx[0];
  dx[static_cast<int>(jpvt[1]) - 1] = work[1];
}

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for linearLeastSquares.cpp
//
// [EOF]
//
