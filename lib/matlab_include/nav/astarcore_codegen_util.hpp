/**
 * @file astarcore_codegen_util.hpp
 * @brief Utility functions for AStarCore functionality in codegen.
*/

/* Copyright 2022 The MathWorks, Inc. */


#ifndef ASTARCORE_CODEGEN_UTIL_HPP
#define ASTARCORE_CODEGEN_UTIL_HPP

#if defined(BUILDING_LIBMWASTARCODEGEN)  
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
/* For uint32_T, boolean_T, etc */
#include "tmwtypes.h"

#define ASTARCORE_CODEGEN_API DLL_EXPORT_SYM 

#else

#if defined(MATLAB_MEX_FILE) || defined(BUILDING_UNITTEST) || defined(MATLAB_BUILTINS)
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C" /* sbcheck:ok:extern_c needed because of LIBMWASTARCORE_CODEGEN_API*/
#else
#define EXTERN_C extern
#endif
#endif

#ifndef ASTARCORE_CODEGEN_API
#define ASTARCORE_CODEGEN_API
#endif

#endif

#endif
