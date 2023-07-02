//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: xzlarf.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "xzlarf.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                ::coder::array<double, 2U> &C
//                int ic0
//                int ldc
//                double work[2]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int m, int n, int iv0, double tau, ::coder::array<double, 2U> &C,
            int ic0, int ldc, double work[2])
{
  int i;
  int jA;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int exitg1;
      jA = ic0;
      do {
        exitg1 = 0;
        if (jA <= (ic0 + lastv) - 1) {
          if (C[jA - 1] != 0.0) {
            exitg1 = 1;
          } else {
            jA++;
          }
        } else {
          lastc = 0;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }
  if (lastv > 0) {
    double c;
    int b_i;
    if (lastc != 0) {
      work[0] = 0.0;
      i = 0;
      for (int iac{ic0}; ldc < 0 ? iac >= ic0 : iac <= ic0; iac += ldc) {
        c = 0.0;
        b_i = (iac + lastv) - 1;
        for (jA = iac; jA <= b_i; jA++) {
          c += C[jA - 1] * C[((iv0 + jA) - iac) - 1];
        }
        work[i] += c;
        i++;
      }
    }
    if (!(-tau == 0.0)) {
      jA = ic0;
      b_i = static_cast<unsigned char>(lastc);
      for (lastc = 0; lastc < b_i; lastc++) {
        c = work[lastc];
        if (c != 0.0) {
          c *= -tau;
          i = lastv + jA;
          for (int iac{jA}; iac < i; iac++) {
            C[iac - 1] = C[iac - 1] + C[((iv0 + iac) - jA) - 1] * c;
          }
        }
        jA += ldc;
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzlarf.cpp
//
// [EOF]
//
