// Copyright 2022 The MathWorks, Inc.

#ifndef SPEC_HPP
#define SPEC_HPP

#if defined(BUILDING_RBTCODEGEN) // should be defined by the mw build
                                 // infrastructure
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
#include "tmwtypes.h"

#define RBTCODEGEN_API DLL_EXPORT_SYM

#else

/* For uint32_T, boolean_T, etc */
/* Consuming MATLAB C++ module should define MATLAB_BUILTINS token in its
 * makefile */
#if defined(MATLAB_MEX_FILE) || defined(BUILDING_UNITTEST) ||                  \
    defined(BUILDING_RBTBUILTINS)
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef RBTCODEGEN_API
#define RBTCODEGEN_API
#endif

#endif /* else */

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif
#endif

#endif /* SPEC_HPP_ */
