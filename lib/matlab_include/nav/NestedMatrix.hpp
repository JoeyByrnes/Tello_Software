/* Copyright 2022 The MathWorks, Inc. */

#ifndef NESTED_NAV_MATRIX_H
#define NESTED_NAV_MATRIX_H

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomapcodegen/octomapcodegen_util.hpp"
#include "octomapcodegen/IndexSequenceNAV.hpp"
#else
#include "octomapcodegen_util.hpp"
#include "IndexSequenceNAV.hpp"
#endif

#include <memory>
#include <vector>
#include <numeric>
#include <algorithm>
#include <array>

namespace nav {
/// @brief Convenience function for wrapping a raw pointer inside a unique_ptr
///     NOTE: This does not create a new object like std::make_unique. This function
///           trusts the caller to hand lifetime management to the unique_ptr.
/// @param rawPtr Raw pointer to an object
/// @return A unique_ptr to the same object
template <typename T>
auto raw2unique(T*& rawPtr) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(rawPtr);
}

/// @brief A convenience class which packages arbitrarily nested C-style arrays with a custom
/// deallocator
template <typename T, size_t N>
class CWrapper {
  protected:
    T m_arr;                      // Dynamically allocated (potentially nested) C-style array
    std::array<size_t, N> m_dims; // Number of dimensions in each layer of the nested array
  public:
    /// @brief Package arbitrarily nested C-style arrays with a custom deallocator
    /// @param arr Dynamically allocated (potentially nested) C-style array
    /// @param dims Number of dimensions in each layer of the nested array
    CWrapper(T arr, size_t const (&dims)[N])
        : m_arr(arr) // Capture address of array
    {
        std::copy(&(dims[0]), &(dims[0]) + N, m_dims.begin());
    }

    /// @brief Package arbitrarily nested C-style arrays with a custom deallocator
    /// @param arr Dynamically allocated (potentially nested) C-style array
    /// @param dims Number of dimensions in each layer of the nested array
    CWrapper(T arr, std::array<size_t, N> dims)
        : m_arr(arr)
        , // Capture address of array
        m_dims(dims) {}

    /// @brief Recurses through the nested array and releases memory
    ~CWrapper() {
        if (m_arr != nullptr) {
            nav::deallocateCArray(m_arr, m_dims);
            m_arr = nullptr;
        }
    }

    /// @brief Returns pointer to underlying C-style array
    /// @return C-style array
    T operator*(void) {
        return m_arr;
    }

    /// @brief Returns i'th element of underlying C-style array
    /// @return i'th element of underlying C-style array
    auto operator[](std::size_t idx) -> decltype(m_arr[idx]) {
        return m_arr[idx];
    }
};

/// @brief Wraps a pointer to a (potentially nested) C-Array in a unique_ptr
/// @param arr Pointer to a C-style array
/// @param dims Dimensions of each layer in nested array
/// @return A unique_ptr with custom deallocator for all arrays in the nested hierarchy
template <typename T, size_t N>
std::unique_ptr<CWrapper<T, N>> wrap_unique_CArray(T arr, size_t const (&dims)[N]) {
    std::unique_ptr<CWrapper<T, N>> ptr(new CWrapper<T, N>(arr, dims));
    return ptr;
}

/// @brief Wraps a pointer to a (potentially nested) C-Array in a unique_ptr
/// @param arr Pointer to a C-style array
/// @param dims Dimensions of each layer in nested array
/// @return A unique_ptr with custom deallocator for all arrays in the nested hierarchy
template <typename T, size_t N>
std::unique_ptr<CWrapper<T, N>> wrap_unique_CArray(T arr, std::array<size_t, N>& dims) {
    std::unique_ptr<CWrapper<T, N>> ptr(new CWrapper<T, N>(arr, dims));
    return ptr;
}

/**
 * @brief Wraps a shared_ptr to an arbitrary "matrix-style" C++ object
 *      This object is constructed using a unique_ptr pointing to a dynamically
 *      allocated resource satisfying the following requirements:
 *          1) The resource lifetime is managed by this object
 *          2) The resource represents a non-jagged matrix
 *          3) The number of dimensions is fixed/known at compile time
 *
 *      Size and dimension order (unpackOrder) provided at construction
 *      determine how the data in the nested container gets copied out to a
 *      linear array.
 *
 *      This helper is used in conjuction with nav::NestedMatrixVoidWrapper and
 *      the C-compliant API in nestedmatrix_api.hpp to return dynamically-allocated
 *      matrices to MATLAB during codegen.
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
template <typename T, size_t N, typename DEALLOC = std::default_delete<T>>
class NestedMatrix {
  protected:
    // Pointer to managed data structure
    std::unique_ptr<T, DEALLOC> m_data;

    // Size of each dimensions in C++ structure
    std::array<size_t, N> m_sizes;

    // Mapping between each layer of C++ structure and corresponding dimension of MATLAB matrix
    //  Example 1:
    //      Assume m_data points to a 2D array, with m_dataSize = [3 10] (e.g: double[3][10])
    //      Assume each row contains all X values for a set of 10 [x y z] points
    //          - Specify m_unpackOrder = [0 1] if the data corresponds to a 3x10 matrix in MATLAB
    //          (default)
    //          - Specify m_unpackOrder = [1 0] if the data corresponds to a 10x3 matrix in MATLAB
    //  Example 2:
    //      Assume m_data points to a 3-layer object, with m_dataSize = [10 3 3] (e.g:
    //      std::vector<std::array<std::array<T,3>>>) Assume the innermost container
    //      (std::array<T,3>) represents an [x y z] point
    //          - Specify m_unpackOrder = [2 0 1] to map the C++ data to a 3x3x10 matrix in MATLAB,
    //          where
    //              - Each 3x3 "page" in the matrix contains an element of the std::vector
    //              - Each row in the 3x3 matrix corresponds to an [x y z] std::array<T,3>
    std::array<size_t, N> m_unpackOrder;

  public:
    /// @brief Creates a wrapper capable of mashaling data from a nested container to linear array
    /// @param data A unique_ptr pointing to a dynamically allocated container or nested C-array
    /// @param dataSize An N-element initializer representing the size of each nested container
    /// @param unpackOrder An N-element std::array that maps the layers in the container to
    /// dimensions of a matrix in MATLAB
    NestedMatrix(
        std::unique_ptr<T, DEALLOC>&& data,
        size_t const (&dataSize)[N],
        std::array<size_t, N> unpackOrder = sequence2array<N>(nav::make_index_sequence<N>{}))
        : m_data(std::move(data))
        , m_unpackOrder(unpackOrder) {
        std::copy(&(dataSize[0]), &(dataSize[0]) + N, m_sizes.begin());
    }

    /// @brief Creates a wrapper capable of mashaling data from a nested container to linear array
    /// @param data A unique_ptr pointing to a dynamically allocated container or nested C-array
    /// @param dataSize An N-element std::array representing the size of each nested container
    /// @param unpackOrder An N-element std::array that maps the layers in the container to
    /// dimensions of a matrix in MATLAB
    NestedMatrix(
        std::unique_ptr<T, DEALLOC>&& data,
        std::array<size_t, N> dataSize,
        std::array<size_t, N> unpackOrder = sequence2array<N>(nav::make_index_sequence<N>{}))
        : m_data(std::move(data))
        , m_sizes(dataSize)
        , m_unpackOrder(unpackOrder) {}

    /// @brief Copies data from the nested resource to a (pre-allocated) linear array
    /// @tparam Tout The type of the output array
    /// @param[out] out Pointer to head of output array
    template <typename Tout>
    void copyData(Tout* out) const;

    /// @brief Returns number of dimensions/containers in managed pointer
    /// @return Number of layers/dimensions in nested container
    const std::size_t getNumDim(void) const {
        return N;
    }

    /// @brief Returns number of elements in stored container
    /// @return Total number of elements in nested container
    std::size_t getNumel(void) const {
        return std::accumulate(m_sizes.begin(), m_sizes.end(), size_t(1),
                               std::multiplies<size_t>());
    }

    /// @brief Copy size of MATLAB matrix represented by the C++ structure to a (pre-allocated)
    /// linear array
    /// @tparam Tout Type of destination array
    /// @param[out] out Pointer to head of output array
    template <typename Tout>
    const void copySizeMatrix(Tout* out) const {
        // Convert C++ to MATLAB dimensions
        auto cpp2matlabMap = createOrderedSizeAndStride();
        auto mSize = std::get<0>(cpp2matlabMap); // Dimensions of MATLAB matrix

        // Copy dimensions to destination pointer
        std::transform(mSize.begin(), mSize.end(), out,
                       [](decltype(*mSize.begin()) e) { return static_cast<Tout>(e); });
    }

    /// @brief Get dimensions of C++ structure as std::array
    /// @return Dimensions of resource managed by this object
    std::array<std::size_t, N> getCppDims(void) const {
        return m_sizes;
    }

    /// @brief Get dimensions of MATLAB structure as std::array
    /// @return Dimensions of the C++ object as a MATLAB matrix
    std::array<std::size_t, N> getMATLABDims(void) const {
        auto cpp2matlabMap = createOrderedSizeAndStride();
        return std::get<0>(cpp2matlabMap);
    }

  protected:
    // Expand template parameter pack and begin recursive copy
    template <typename Tout, std::size_t... Ts>
    void copyDataExpansion(Tout* out,
                           std::array<std::size_t, 2 * N> SizeStridePairs,
                           nav::index_sequence<Ts...>) const;

    // Recursively traverses the stored/output data structures with appropriate stride until base
    // case is hit
    template <typename Tout,
              typename C,
              typename CurSize,
              typename CurStride,
              typename... REST> // REST contains pairs of dimension size and stride
    void copyElement(Tout* dest, C& elem, CurSize cSize, CurStride stride, REST... rest) const;

    // Base case, copying actually happens here using the appropriate stride
    template <typename Tout, typename C, typename CurSize, typename CurStride>
    void copyElement(Tout* dest, C& elem, CurSize cSize, CurStride stride) const;

    /// @brief Calculate matrix size and stride information when copying C++ data to linear MATLAB
    /// array
    ///     This method returns a tuple containing the following:
    ///         1) matlabSize       : The C++ dimensions (m_sizes) rearranged based on the
    ///         m_unpackOrder (essentially the size of the matrix in MATLAB) 2) cppStride        :
    ///         The distance between elements in the linear output array for a step along each
    ///         dimension of the C++ data-structure
    std::tuple<std::array<size_t, N>, std::array<size_t, N>> createOrderedSizeAndStride(void) const;
};

template <typename T, size_t N, typename DEALLOC>
std::tuple<std::array<size_t, N>, std::array<size_t, N>>
nav::NestedMatrix<T, N, DEALLOC>::createOrderedSizeAndStride(void) const {
    std::array<size_t, N> matlabDims;
    std::array<size_t, N + 1> stride{1};    // 1st element initialized to 1
    std::array<size_t, N> orderedStride{1}; // 1st element initialized to 1
    size_t step = 1;
    for (size_t i = 0; i < N; i++) {
        matlabDims[m_unpackOrder[i]] = m_sizes[i];
    }
    for (size_t i = 0; i < N; i++) {
        step *= matlabDims[i];
        stride[i + 1] = step;
    }
    for (size_t i = 0; i < N; i++) {
        orderedStride[i] = stride[m_unpackOrder[i]];
    }
    std::tuple<std::array<size_t, N>, std::array<size_t, N>> cpp2matlabMap(matlabDims,
                                                                           orderedStride);
    return cpp2matlabMap;
}

template <typename T, size_t N, typename DEALLOC>
template <typename Tout>
void NestedMatrix<T, N, DEALLOC>::copyData(Tout* out) const {
    auto sizeAndStride = createOrderedSizeAndStride();
    auto strideInCpp = std::get<1>(sizeAndStride);

    // Interlace size and stride info
    std::array<size_t, 2 * N> interlacedSizeAndStride;
    for (size_t i = 0; i < N; i++) {
        interlacedSizeAndStride[2 * i] = m_sizes[i];
        interlacedSizeAndStride[2 * i + 1] = strideInCpp[i];
    }

    // Expand and extract
    copyDataExpansion(out, interlacedSizeAndStride, nav::make_index_sequence<2 * N>{});
}

// Expand template parameter pack and begin recursive copy
template <typename T, size_t N, typename DEALLOC>
template <typename Tout, std::size_t... Ts>
void NestedMatrix<T, N, DEALLOC>::copyDataExpansion(Tout* out,
                                                    std::array<std::size_t, 2 * N> SizeStridePairs,
                                                    nav::index_sequence<Ts...>) const {
    copyElement(out, *m_data, (SizeStridePairs[Ts])...);
}

// Recursively traverses the stored/output data structures with appropriate stride until base case
// is hit
template <typename T, size_t N, typename DEALLOC>
template <typename Tout, typename C, typename CurSize, typename CurStride, typename... REST>
void nav::NestedMatrix<T, N, DEALLOC>::copyElement(Tout* dest,
                                                   C& elem,
                                                   CurSize cSize,
                                                   CurStride stride,
                                                   REST... rest) const {
    for (size_t i = 0; i < cSize; i++) {
        copyElement(dest, elem[i], rest...);
        dest += stride;
    }
}

// Base case, copying actually happens here using the appropriate stride
template <typename T, size_t N, typename DEALLOC>
template <typename Tout, typename C, typename CurSize, typename CurStride>
void nav::NestedMatrix<T, N, DEALLOC>::copyElement(Tout* dest,
                                                   C& elem,
                                                   CurSize cSize,
                                                   CurStride stride) const {
    for (size_t i = 0; i < cSize; i++) {
        *dest = static_cast<Tout>(elem[i]);
        dest += stride;
    }
}
} // namespace nav
#endif /* NESTED_NAV_MATRIX_H */
