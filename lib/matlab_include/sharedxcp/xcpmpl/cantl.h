/* Copyright 2012-2021 The MathWorks, Inc. */

#ifndef CANTL_H
#define CANTL_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#if defined(MATLAB_MEX_FILE)
    #if defined (_WIN32)
        #define CANAPI __declspec(dllimport)
    #elif defined (__linux)
        #define CANAPI
    #endif
#elif defined(XCP_DLL_EXPORT)
    #if defined (_WIN32)
        #define CANAPI __declspec(dllexport)
    #elif defined (__linux)
        #define CANAPI __attribute__((visibility("default")))
    #endif
#else
    #define CANAPI
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void * HCTL;
typedef void * HCFDTL;

CANAPI HCTL     ctlGetHandle(const char* tag);
CANAPI void     ctlReleaseHandle(const char* tag);
CANAPI int32_t  ctlExecuteTx(HCTL handle, void* out_buf, uint32_t buf_size);
CANAPI int32_t  ctlExecuteRx(HCTL handle, void* in_buf,  uint32_t rx_count);

CANAPI HCFDTL    cfdtlGetHandle(const char* tag);
CANAPI void     cfdtlReleaseHandle(const char* tag);
CANAPI int32_t  cfdtlExecuteTx(HCTL handle, void* out_buf, uint32_t buf_size);
CANAPI int32_t  cfdtlExecuteRx(HCTL handle, void* in_buf,  uint32_t rx_count);

#ifdef __cplusplus
}
#endif

#endif

