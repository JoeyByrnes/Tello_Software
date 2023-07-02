/* Copyright 2020 The MathWorks, Inc. */
#ifndef MAVLINKCODEGEN_UTIL_HPP
#define MAVLINKCODEGEN_UTIL_HPP

#if defined(BUILDING_LIBMWMAVLINKCODEGEN)
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
/* For uint32_T, boolean_T, etc */
#include "tmwtypes.h"

#define MAVLINK_CODEGEN_API DLL_EXPORT_SYM

#else

#if defined(MATLAB_MEX_FILE) || defined(BUILDING_UNITTEST) || defined(MATLAB_BUILTINS)
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C" /* sbcheck:ok:extern_c needed because of LIBMWMAVLINKCODEGEN_API*/
#else
#define EXTERN_C extern
#endif
#endif

#ifndef MAVLINK_CODEGEN_API
#define MAVLINK_CODEGEN_API
#endif

#endif

#endif
