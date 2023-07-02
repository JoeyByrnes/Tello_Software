/* Copyright 2019 The MathWorks, Inc. */

/**
 * @file
 * Utility functions for octomap 3D functionality in codegen.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C++98 standard.
 */

#ifndef OCTOMAPCODEGEN_UTIL_HPP_
#define OCTOMAPCODEGEN_UTIL_HPP_

#if defined(BUILDING_LIBMWOCTOMAPCODEGEN)
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
/* For uint32_T, boolean_T, etc*/
#include "tmwtypes.h"

#define OCTOMAP_CODEGEN_API DLL_EXPORT_SYM

#else

/* For uint32_T, boolean_T, etc */
/* Consuming MATLAB C++ module should define MATLAB_BUILTINS token in its makefile */
#if defined(MATLAB_MEX_FILE) || defined(BUILDING_UNITTEST) || defined(MATLAB_BUILTINS)
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif
#endif

#ifndef OCTOMAP_CODEGEN_API
#define OCTOMAP_CODEGEN_API
#endif

#endif /* else */

#endif /* OCTOMAPCODEGEN_UTIL_HPP_ */
