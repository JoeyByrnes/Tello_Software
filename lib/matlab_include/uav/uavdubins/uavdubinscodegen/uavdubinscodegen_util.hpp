/* Copyright 2020 The MathWorks, Inc. */

/**
 * @file : uavDubinsCodegen.hpp
 * Defines DLL import/export macros.
 */

#ifndef UAVDUBINSCODEGEN_UTIL_HPP_
#define UAVDUBINSCODEGEN_UTIL_HPP_

#if defined(BUILDING_LIBMWUAVDUBINSCODEGEN)

#include "package.h" /* For DLL_EXPORT_SYM and EXTERN_C */
#include "tmwtypes.h" /* For uint32_T, boolean_T, etc */
#define UAVDUBINSCODEGEN_API DLL_EXPORT_SYM

#else

#if defined(MATLAB_MEX_FILE) || defined(BUILDING_UNITTEST) || defined(MATLAB_BUILTINS)
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif
#endif

#ifndef UAVDUBINSCODEGEN_API
#define UAVDUBINSCODEGEN_API
#endif

#endif

#endif /* UAVDUBINSCODEGEN_UTIL_HPP_ */
