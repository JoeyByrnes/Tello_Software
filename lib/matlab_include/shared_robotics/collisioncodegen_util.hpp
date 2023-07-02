// Copyright 2018-2022 The MathWorks, Inc.

/**
 * @file
 * @brief Provide additional typedefs needed for collisioncodegen
 */

#ifndef COLLISIONCODEGEN_UTIL_HPP
#define COLLISIONCODEGEN_UTIL_HPP

#if defined(BUILDING_LIBMWCOLLISIONCODEGEN) // should be defined by the mw build
                                            // infrastructure
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
#include "tmwtypes.h"

#define COLLISIONCODEGEN_API DLL_EXPORT_SYM

#else

/* For uint32_T, boolean_T, etc */
/* Consuming MATLAB C++ module should define MATLAB_BUILTINS token in its
 * makefile */
#if defined(MATLAB_MEX_FILE) || defined(BUILDING_UNITTEST) ||                  \
    defined(MATLAB_BUILTINS)
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef COLLISIONCODEGEN_API
#define COLLISIONCODEGEN_API
#endif

#endif /* else */

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif
#endif

#endif /* COLLISIONCODEGEN_UTIL_HPP_ */
