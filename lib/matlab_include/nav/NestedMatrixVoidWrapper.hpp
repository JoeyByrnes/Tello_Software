/* Copyright 2022 The MathWorks, Inc. */

#ifndef VOID_WRAPPER_H
#define VOID_WRAPPER_H

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomapcodegen/nestedmatrix_api.hpp"
#include "octomapcodegen/NestedMatrix.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "nestedmatrix_api.hpp"
#include "NestedMatrix.hpp"
#endif

#include <functional>

namespace nav {
// Retrieve #elements void-cast NestedMatrix*
//      Function must cast to appropriate NestedMatrix<T,N,DEALLOC> before calling getNumel
using NumelFcn = std::function<uint64_T(void*)>;

// Retrieve #dimensions void-cast NestedMatrix*
//      Function should cast to appropriate NestedMatrix<T,N,DEALLOC> before calling getMATLABDims
using NumDimFcn = std::function<uint64_T(void*)>;

// Copy size matrix from void-cast NestedMatrix* into array passed from MATLAB
//      Function should cast to appropriate NestedMatrix<T,N,DEALLOC> before calling copySizeMatrix
using CopySizeFcn = std::function<void(void*, uint64_T*)>;

// Release the nested matrix and clean up allocated memory contained within
//      Function should cast to appropriate NestedMatrix<T,N,DEALLOC> before calling delete
using CleanupFcn = std::function<void(void*)>;

// Copy data matrix from void-cast NestedMatrix* into matrix passed from MATLAB
//      Function should cast to appropriate NestedMatrix<T,N,DEALLOC> before calling copyData
template <typename Tout>
using copyDataFcn = std::function<void(void*, Tout*)>;

/**
 * @brief A base-class which provides an interface to a type-erased nav::NestedMatrix object
 *      This object manages a dynamically allocated, type-erased "delegate" - specifically,
 *      it manages a pointer to a NestedMatrix that has been cast to void.
 *
 *      Handles to NestedMatrix member functions generated before the type was erased allow this
 * object to redirect calls to the managed pointer without knowledge of the underlying type.
 *
 *      Use nav::createWrapper to convert a nav::NestedMatrix* to a NestedMatrixVoidWrapperBase
 *
 *      Example:
 *          MATLAB:
 *              ...
 *              % Create container for dynamically allocated matrix
 *              matWrapper = nav.algs.internal.coder.dynamicMatrixBuildable([inf 3]); % Voxel
 * center(s)
 *
 *              % The C++ function "foo" returns a wrapper around some dynamically allocated C++
 * object representing [inf 3] matrix coder.ceval('foo', ..., coder.ref(matWrapper.Ptr));
 *
 *              % Copy data back to matlab
 *              mat = matWrapper.getData();
 *
 *              % C++ object will get cleaned up automatically when matWrapper goes out of scope
 *
 *          C++:
 *              void foo(..., void** outputPtr)
 *              {
 *                  // Generate a dynamically allocated resource
 *                  auto* data = new std::vector<std::array<double,3>>();
 *
 *                  // Allocate/define populate the data
 *                  ...
 *
 *                  // Wrap the resource in the NestedMatrix marshaller
 *                  auto* ctrMat = new nav::NestedMatrix<std::vector<std::array<double, 3>>,
 * 2>(std::move(nav::raw2unique(ctrs)), { ctrs->size(), 3});
 *
 *                  // Generate a new NestedMatrixVoidWrapperBase and point the void** to it
 *                  nav::createWrapper<double>(ctrMat,*ctrPtr);
 *              }
 */
class NestedMatrixVoidWrapperBase {
  protected:
    void* m_nestedMatrix;    // Pointer to a nav::NestedMatrix that has been cast to void*
    NumelFcn m_numelFcn;     // Handle for calling nav::NestedMatrix::getNumel()
    NumDimFcn m_numDimFcn;   // Handle for calling nav::NestedMatrix::getNumDim()
    CopySizeFcn m_sizeFcn;   // Handle for calling nav::NestedMatrix::copySize()
    CleanupFcn m_CleanupFcn; // Handle for deleting the nav::NestedMatrix
  public:
    /// @brief Constructor
    /// @param nestedMat Pointer to a nav::NestedMatrix
    /// @param fNumel Handle for calling nav::NestedMatrix::getNumel()
    /// @param fNumDim Handle for calling nav::NestedMatrix::getNumDim()
    /// @param fSize Handle for calling nav::NestedMatrix::copySize()
    /// @param fCleanup Handle for deleting the nav::NestedMatrix
    NestedMatrixVoidWrapperBase(void* nestedMat,
                                NumelFcn&& fNumel,
                                NumDimFcn&& fNumDim,
                                CopySizeFcn&& fSize,
                                CleanupFcn&& fCleanup)
        : m_nestedMatrix(nestedMat)
        , m_numelFcn(fNumel)
        , m_numDimFcn(fNumDim)
        , m_sizeFcn(fSize)
        , m_CleanupFcn(fCleanup) {}

    /// @brief Virtual destructor called when MATLAB object goes out of scope
    virtual ~NestedMatrixVoidWrapperBase() {
        m_CleanupFcn(m_nestedMatrix);
    }

    /// @brief Retrieve number of elements in NestedMatrix
    /// @return Number of elements
    uint64_T numel(void) const {
        return m_numelFcn(m_nestedMatrix);
    }

    /// @brief Retrieve number of dimensions in NestedMatrix
    /// @return Number of dimensions
    uint64_T numDim(void) const {
        return m_numDimFcn(m_nestedMatrix);
    }

    /// @brief Copy the size of MATLAB-side matrix stored by C++ object
    /// @param szOut Size of matrix in MATLAB
    void size(uint64_T* szOut) const {
        m_sizeFcn(m_nestedMatrix, szOut);
    }
};

/// @brief A derived version of NestedMatrixVoidWrapperBase needed for
///     copying data from the NestedMatrix to linear array of specific datatype
template <typename Tout>
class NestedMatrixVoidWrapper : public NestedMatrixVoidWrapperBase {
  protected:
    copyDataFcn<Tout>
        m_dataFcn; // Handle for copying data out of the nav::NestedMatrix to a linear array
  public:
    /// @brief Constructor
    /// @param nestedMat Pointer to a nav::NestedMatrix
    /// @param fNumel Handle for calling nav::NestedMatrix::getNumel()
    /// @param fNumDim Handle for calling nav::NestedMatrix::getNumDim()
    /// @param fSize Handle for calling nav::NestedMatrix::copySize()
    /// @param fCleanup Handle for deleting the nav::NestedMatrix
    /// @param fData Handle for copying data to linear array
    NestedMatrixVoidWrapper(void* nestedMat,
                            NumelFcn&& fNumel,
                            NumDimFcn&& fNumDim,
                            CopySizeFcn&& fSize,
                            CleanupFcn&& fCleanup,
                            copyDataFcn<Tout>&& fData)
        : NestedMatrixVoidWrapperBase(std::move(nestedMat),
                                      std::move(fNumel),
                                      std::move(fNumDim),
                                      std::move(fSize),
                                      std::move(fCleanup))
        , m_dataFcn(fData) {}

    virtual ~NestedMatrixVoidWrapper() {
        // Nothing more to clean
    }

    /// @brief Copy the data from the C++ object to a linear array
    /// @param data Pointer to head of linear array
    void data(Tout* data) const {
        m_dataFcn(m_nestedMatrix, data);
    }
};

/// @brief Wraps nav::NestedMatrix in a NestedMatrixVoidWrapper
/// @param nestedMatrix Object managing some nested/dynamically-allocated C++ matrix
/// @param[out] wrapper NestedMatrixVoidWrapper cast to void*
template <typename Tout, typename T, std::size_t N, typename DEALLOC>
void createWrapper(nav::NestedMatrix<T, N, DEALLOC>*& nestedMatrix, void*& wrapper) {
    // Create function handles to wrapped object's methods
    NumelFcn f0 = [](void* voidObj) -> uint64_T {
        auto* objPtr = static_cast<nav::NestedMatrix<T, N, DEALLOC>*>(voidObj);
        return static_cast<uint64_T>(objPtr->getNumel());
    };
    NumDimFcn f1 = [](void* voidObj) -> uint64_T {
        auto* objPtr = static_cast<nav::NestedMatrix<T, N, DEALLOC>*>(voidObj);
        return static_cast<uint64_T>(objPtr->getNumDim());
    };
    CopySizeFcn f2 = [](void* voidObj, uint64_T* out) {
        auto* objPtr = static_cast<nav::NestedMatrix<T, N, DEALLOC>*>(voidObj);
        objPtr->copySizeMatrix(out);
    };
    CleanupFcn f3 = [](void* voidObj) {
        if (voidObj != nullptr) {
            auto* objPtr = static_cast<nav::NestedMatrix<T, N, DEALLOC>*>(voidObj);
            delete objPtr; // sbcheck:ok:allocterms
            objPtr = nullptr;
            voidObj = nullptr;
        }
    };
    copyDataFcn<Tout> f4 = [](void* voidObj, Tout* out) {
        auto* objPtr = static_cast<nav::NestedMatrix<T, N, DEALLOC>*>(voidObj);
        objPtr->copyData(out);
    };

    // Create typed wrapper
    auto* pDerived = new NestedMatrixVoidWrapper<Tout>(nestedMatrix, std::move(f0), std::move(f1),
                                                       std::move(f2), std::move(f3), std::move(f4));

    // Convert to base pointer
    auto* pBase = static_cast<NestedMatrixVoidWrapperBase*>(pDerived);

    // Return as void pointer
    wrapper = static_cast<void*>(pBase);
}
} // namespace nav

#endif /* VOID_WRAPPER_H */
