/* Copyright 2022 The MathWorks, Inc. */

#ifndef INDEX_SEQUENCE_H
#define INDEX_SEQUENCE_H

#include <functional>
#include <iterator>
#include <memory>

namespace nav {
// Generate expandable sequence of indices for variadic templates in C++11
// https://stackoverflow.com/questions/34929856/unpack-an-array-to-call-a-function-with-variadic-template
template <std::size_t... I>
class index_sequence {};

template <std::size_t N, std::size_t... I>
struct make_index_sequence : make_index_sequence<N - 1, N - 1, I...> {};

template <std::size_t... I>
struct make_index_sequence<0, I...> : index_sequence<I...> {};

template <std::size_t N, std::size_t... Ts>
std::array<std::size_t, N> sequence2array(nav::index_sequence<Ts...>,
                                          std::size_t offset = std::size_t(0)) {
    std::array<std::size_t, N> a{(offset + Ts)...};
    return a;
}

/// @brief Deallocates the innermost C-array (base case)
/// @tparam T Type of array
/// @tparam REMSIZE Template parameter pack containing size of array
/// @param arr Array of type T
template <typename T, typename... REMSIZE>
void deallocateArr(T arr, REMSIZE...) {
    if (arr != nullptr) {
        // Free self
        delete[] arr; // sbcheck:ok:allocterms
        arr = nullptr;
    }
}

/// @brief Recursively deallocates a nested C-style array (recursive step)
/// @param arr Nested C-style array
/// @param cSize Dimension of current array
/// @param sizes Parameter pack containing dimensions of arrays below current array
template <typename T, typename CURSIZE, typename... REMSIZE>
void deallocateArr(T arr, CURSIZE cSize, REMSIZE... sizes) {
    if (arr != nullptr) {
        for (size_t i = 0; i < cSize; i++) {
            if (arr[i] != nullptr) {
                deallocateArr(arr[i], sizes...);
            }
        }
        // Free self
        delete[] arr; // sbcheck:ok:allocterms
        arr = nullptr;
    }
}

/// @brief Expands parameter packs and recursively deallocates C-style array
/// @param arr (Potentially nested) C-style array
/// @param sizes Dimensions of nested array
template <size_t NDIM, typename T, size_t... I>
void deallocateCArrayExpand(T arr, std::array<size_t, NDIM> sizes, nav::index_sequence<I...>) {
    deallocateArr(arr, sizes[I]...);
}

/// @brief Recursive deallocate C-style array
/// @param arr (Potentially nested) C-style array
/// @param sizes Dimensions of nested array
template <size_t NDIM, typename T>
void deallocateCArray(T arr, std::array<size_t, NDIM> sizes) {
    deallocateCArrayExpand(arr, sizes, nav::make_index_sequence<NDIM - 1>{});
}

/// @brief Convenience function for wrapping a raw pointer to an object inside a unique_ptr
///     NOTE: This does not create a new object like std::make_unique. This function expects
///           that the user relinquishes deallocation responsibilities to the unique_ptr.
/// @tparam T Type of new object constructed with input arguments
/// @tparam Args Template parameter pack of argument input types
/// @param args Template paramter pack of arguments
/// @return unique_ptr to object of type T
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(
        new T(std::forward<Args>(args)...)); // All arguments are forwarded to T-style constructor
}
} // namespace nav

#endif /* INDEX_SEQUENCE_H */
