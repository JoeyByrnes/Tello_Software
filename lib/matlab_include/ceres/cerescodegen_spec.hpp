/* Copyright 2021-2022 The MathWorks, Inc. */

/**
 * @file
 * @brief Provide necessary typedefs for cerescodegen
 */

#ifndef CERESCODEGEN_SPEC_HPP
#define CERESCODEGEN_SPEC_HPP

    

    #ifdef BUILDING_LIBMWCERESCODEGEN // should be defined by the mw build infrastructure

        /* This header is being included by files inside this module */
        #include "package.h" /* For DLL_EXPORT_SYM, DLL_EXPORT_SYM */

        #include "tmwtypes.h" /* For uint32_T, boolean_T, etc */

        #define CERESCODEGEN_API  DLL_EXPORT_SYM

    #else
        /* Consuming internal MATLAB C++ module should define MATLAB_BUILTINS token in its makefile */
        #if defined(MATLAB_MEX_FILE) || defined(MATLAB_BUILTINS)
            #include "tmwtypes.h"
        #else
            #include "rtwtypes.h"
        #endif

        #ifndef CERESCODEGEN_API
            #define CERESCODEGEN_API
        #endif

    #endif

    #ifndef EXTERN_C
        #ifdef __cplusplus
            #define EXTERN_C extern "C"
        #else
            #define EXTERN_C extern
        #endif
    #endif

    #ifndef M_PI
        #define M_PI 3.14159265358979323846
    #endif

#endif // CERESCODEGEN_SPEC_HPP


