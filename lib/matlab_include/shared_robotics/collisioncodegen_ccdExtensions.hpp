// Copyright 2018-2019 The MathWorks, Inc.

/**
 * @file
 * @brief Extension on existing libccd functions to support finding distance and witness points 
 */

#ifndef COLLISIONCODEGEN_CCDEXTENSIONS_HPP
#define COLLISIONCODEGEN_CCDEXTENSIONS_HPP

#ifdef BUILDING_LIBMWCOLLISIONCODEGEN
#include <ccd/ccd_ccd.h>
#else
#include <ccd_ccd.h>
#endif
#include "collisioncodegen_helper.hpp"
#include <iostream>

namespace shared_robotics 
{
    /**
     * @brief Compute the minimal distance between @p obj1 and @p obj2 if they are not in collision, also computes the witness points
     * @param[in] obj1 Pointer to @c CollisionGeometry object 1
     * @param[in] obj2 Pointer to @c CollisionGeometry object 2
     * @param[in] ccd Pointer to a @c ccd_t object
     * @param[out] p1 Pointer to witness point on @p obj1
     * @param[out] p2 Pointer to witness point on @p obj2
     * @return The minimal distance between @p obj1 and @p obj2
     */
    ccd_real_t ccdDistance(const void *obj1, const void *obj2, const ccd_t *ccd,
                           ccd_vec3_t* p1, ccd_vec3_t* p2);

    /// compute minimal distance from the given simplex to origin (internal use only).
    ccd_real_t distanceToOrigin(ccd_simplex_t * simplex, ccd_vec3_t *& closestPoint);

    /// extract the witness points from the closest point found on a simplex (internal use only).
    void extractWitnessPoints(const ccd_simplex_t * simplex, const ccd_vec3_t * closestPoint, ccd_vec3_t *p1, ccd_vec3_t *p2);

    /// interpolate between two support points (internal use only).
    void interpolateBetweenTwoSupportPoints(const ccd_simplex_t *& simplex, const ccd_vec3_t *& closestPoint, ccd_vec3_t *& p1, ccd_vec3_t *& p2);

    /// interpolate between three support points (internal use only).
    void interpolateAmongThreeSupportPoints(const ccd_simplex_t *& simplex, const ccd_vec3_t *& closestPoint, ccd_vec3_t *& p1, ccd_vec3_t *& p2);


} // namespace shared_robotics   
#endif
