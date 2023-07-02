// Copyright 2019 The MathWorks, Inc.

/**
 * @file
 * @brief Provide additional typedefs needed for collisioncodegen
 */

#ifndef COLLISIONCODEGEN_HELPER_HPP
#define COLLISIONCODEGEN_HELPER_HPP

#include "collisioncodegen_util.hpp"
#include <cstddef>
#ifdef BUILDING_LIBMWCOLLISIONCODEGEN
#include <ccd/ccd_compiler.h>
#else
#include <ccd_compiler.h>
#endif

namespace shared_robotics {
struct _idx_vec3_t {
    std::size_t v[3];
};
typedef struct _idx_vec3_t idx_vec3_t;

_ccd_inline void idxVec3Set(idx_vec3_t* v, std::size_t x, std::size_t y, std::size_t z) {
    v->v[0] = x;
    v->v[1] = y;
    v->v[2] = z;
}

} // namespace shared_robotics

#endif /*  COLLISIONCODEGEN_HELPER_HPP */
