/* Copyright 2022 The MathWorks, Inc. */

#ifndef NAV_TRAITS_H
#define NAV_TRAITS_H

#include <iterator>
#include <functional>
#include <type_traits>

/* https://stackoverflow.com/questions/13830158/check-if-a-variable-type-is-iterable/53967057#53967057
 */

namespace detail {
using std::begin;
using std::end;

template <typename T>
auto is_iterable_impl(int a) // Automatically attempt this implementation first
    -> std::decay_t<
        decltype(void(*begin(std::declval<T&>())), // Has a begin iterator that can be dereferenced
                 begin(std::declval<T&>()) != end(std::declval<T&>()),   // input is not empty
                 void(),                                                 // Handle evil operator
                 ++std::declval<decltype(begin(std::declval<T&>()))&>(), // operator++ is defined
                 std::true_type{})>; // If all previous conditions do not error during inference,
                                     // this type is true

// Any input which does not pass conditions of above specialization will show as false
template <typename T>
std::false_type is_iterable_impl(...) {
    return std::false_type();
}
} // namespace detail

namespace nav {
// is_iterable A template specialization trait for determining whether input of type T is iterable
template <typename T>
using is_iterable = decltype(detail::is_iterable_impl<T>(0));

template <typename T>
constexpr inline bool is_iterable_v = is_iterable<T>::value;

// element_type_t retrieve type of element contained by the template object
template <typename T, size_t N, typename DEALLOC>
using element_type_t = std::remove_reference_t<decltype(*std::begin(std::declval<T&>()))>;
} // namespace nav

#endif /* NAV_TRAITS_H */
