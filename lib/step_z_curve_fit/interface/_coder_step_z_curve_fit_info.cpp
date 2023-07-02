//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: _coder_step_z_curve_fit_info.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

// Include Files
#include "_coder_step_z_curve_fit_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
//
// Arguments    : void
// Return Type  : const mxArray *
//
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5]{
      "789ced54cb4ac340149d481541acd9b81517dd095db9d19dd4168b582a2d58da949aa453"
      "3375261327939abaf20ffc0dc17ff03ffc1bc9ab4d0686041f15a177"
      "737372e6de73e6925ca034af1400401944d1db89f26e8cd5386f806c88bc229c53b2c7c1"
      "262865ea12fe25ce26b539f479046c9dc045e5981264eb36efce1d08",
      "1874299ec171c84c10865d4460270d5a01228d14b50001153cd72c68de773c0298e52e1d"
      "e23458cce35972df52c1791c4ae6a10afca03e3c3fd5da14cf1b886b"
      "2e87cee869647a6c064713c4ab24f1e37cd34f59c0a29f848fa53ddbe488dacbf7b75fd4"
      "1743a69fc44fcdff20472fe1b3f3cfdebe4af2efbf57d08f9897e7b7",
      "c33c7dfb08a955e9a947effd55ea25f1577abea45fd1ef695fa2a70afcb533355b3777b8"
      "73c9fcfefcecb86d9db08b948f768e4e9e0f20c1abeaff2aa92f3ac7"
      "9aa4bf2af08366af3eac109d63dd6094f28ac629c506f535d7d2191c6bd4e18860646890"
      "600dbb0fe1af1bac4c477964bae34096f1fd5ff767a2b725d58f9831",
      "f50c0cd7fb73bd3f7f476fbd3fbfd7ff131f1e99f6", ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 2928U, &nameCaptureInfo);
  return nameCaptureInfo;
}

//
// Arguments    : void
// Return Type  : mxArray *
//
mxArray *emlrtMexFcnProperties()
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[7]{
      "Version",      "ResolvedFunctions", "Checksum",    "EntryPoints",
      "CoverageInfo", "IsPolymorphic",     "PropertyList"};
  const char_T *epFieldName[6]{
      "Name",           "NumberOfInputs", "NumberOfOutputs",
      "ConstantInputs", "FullPath",       "TimeStamp"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 5);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("step_z_curve_fit"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(5.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString("D:\\PolyFit\\step_z_curve_fit.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739067.74359953706));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("9.14.0.2206163 (R2023a)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("iJi8fcEot0E9JwnKeo0OWB"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_step_z_curve_fit_info.cpp
//
// [EOF]
//
