/* Copyright 2019 The MathWorks, Inc. */

/**
 * @file
 * Constants used in shared autonomous features.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C++98 standard.
 */

#ifndef AUTONOMOUSCODEGEN_QUERIES_H_
#define AUTONOMOUSCODEGEN_QUERIES_H_

#ifdef BUILDING_LIBMWAUTONOMOUSCODEGEN
#include "autonomouscodegen/autonomouscodegen_util.hpp"
#else
#include "autonomouscodegen_util.hpp"
#endif

namespace autonomous {
namespace queries {

inline bool isNaN(real64_T x){
    return x != x;
}

} // namespace queries
} // namespace autonomous

#endif // AUTONOMOUSCODEGEN_QUERIES_H_
