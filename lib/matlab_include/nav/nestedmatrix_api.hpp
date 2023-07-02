/* Copyright 2022 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for NestedMatrix helper.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifndef NESTEDMATRIX_CODEGEN_API_H_
#define NESTEDMATRIX_CODEGEN_API_H_

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomapcodegen/octomapcodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "octomapcodegen_util.hpp"
#endif

/**
 * @brief Retrieve number of elements in array stored by NestedMatrixVoidWrapperBase pointer
 *
 * @param voidWrapper void*-cast ptr to NestedMatrixVoidWrapperBase*
 * @return numel Total number of elements in stored matrix
 */
EXTERN_C OCTOMAP_CODEGEN_API uint64_T nestedmatrixcodegen_getNumel(void* voidWrapper);

/**
 * @brief Retrieve number of dimensions in array stored by NestedMatrixVoidWrapperBase pointer
 *
 * @param voidWrapper void*-cast ptr to NestedMatrixVoidWrapperBase*
 * @return numDim Dimensions of stored matrix in MATLAB
 */
EXTERN_C OCTOMAP_CODEGEN_API uint64_T nestedmatrixcodegen_getNumDimensions(void* voidWrapper);

/**
 * @brief Retrieve size of array stored by NestedMatrixVoidWrapperBase pointer
 *
 * @param voidWrapper void*-cast ptr to NestedMatrixVoidWrapperBase*
 * @param[out] sz A 1xN array storing the size of the data matrix stored by the NestedArray
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_getMATLABSize(void* voidWrapper,
                                                                    uint64_T* sz);

/**
 * @brief Delete pointer to NestedMatrixVoidWrapperBase
 *
 * @param voidWrapper void*-cast ptr to NestedMatrixVoidWrapperBase*
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_destruct(void* voidWrapper);

/**
 * The following API will copy the data out of the nested container
 * and cast it to the type defined by the function.
 *
 *      NOTE: The API used MUST MATCH the type used to construct the
 *            NestedMatrixVoidWrapper<type>* (passed here as void*)
 */

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<double>*
 *
 * @param voidWrapper void*-cast ptr to NestedMatrixVoidWrapper<double>*
 * @param[out] dest Allocated array of DOUBLES which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_REAL64(void* voidWrapper,
                                                                      real64_T* dest);

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<float>*
 *
 * @param voidWrapper void*-casted ptr to NestedMatrixVoidWrapper<float>*
 * @param[out] dest Allocated array of SINGLES which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_REAL32(void* voidWrapper,
                                                                      real32_T* dest);

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<uint64>*
 *
 * @param voidWrapper void*-casted ptr to NestedMatrixVoidWrapper<uint64>*
 * @param[out] dest Allocated array of UINT64 which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_UINT64(void* voidWrapper,
                                                                      uint64_T* dest);

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<uint32>*
 *
 * @param voidWrapper void*-casted ptr to NestedMatrixVoidWrapper<uint32>*
 * @param[out] dest Allocated array of UINT32 which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_UINT32(void* voidWrapper,
                                                                      uint32_T* dest);

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<int64>*
 *
 * @param voidWrapper void*-casted ptr to NestedMatrixVoidWrapper<int64>*
 * @param[out] dest Allocated array of INT64 which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_INT64(void* voidWrapper,
                                                                     int64_T* dest);

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<int32>*
 *
 * @param voidWrapper void*-casted ptr to NestedMatrixVoidWrapper<int32>*
 * @param[out] dest Allocated array of INT32 which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_INT32(void* voidWrapper,
                                                                     int32_T* dest);

/**
 * @brief Retrieve data stored by NestedMatrixVoidWrapper<bool>*
 *
 * @param voidWrapper void*-casted ptr to NestedMatrixVoidWrapper<bool>*
 * @param[out] dest Allocated array of BOOLEANS which will receive the values stored in voidWrapper
 */
EXTERN_C OCTOMAP_CODEGEN_API void nestedmatrixcodegen_retrieve_BOOLEAN(void* voidWrapper,
                                                                       boolean_T* dest);

#endif /* NESTEDMATRIX_CODEGEN_API_H_ */
