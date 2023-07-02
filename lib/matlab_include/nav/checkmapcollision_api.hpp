#pragma once
/* Copyright 2022 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for Octomap/CollisionGeometry collision-check.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifndef OCTOGEOM_COLLISION_FUNCTIONS_CODEGEN_API_H_
#define OCTOGEOM_COLLISION_FUNCTIONS_CODEGEN_API_H_

#ifdef BUILDING_LIBMWCOLLISIONMAPCODEGEN
#include "collisionmapcodegen/collisionmapcodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "collisionmapcodegen_util.hpp"
#endif

/// @brief Checks for collision between occupancyMap3D and collisionGeometry C++ wrapper objects
/// @param map A pointer to the nav::octomapcodegen-wrapped octomap::OcTree class
/// @param geom A pointer to a shared_robotics::CollisionGeometry object
/// @param[out] distance Return value for distance between the map/geometry. Setting this to 0 will
/// skip the distance check
/// @param exhaustive A boolean dictating whether the routine should continue looking for collisions
/// after the first has been encountered
/// @param narrowPhase A boolean dictating whether broad-phase collisions should trigger a
/// narrow-phase collision between the colliding voxel and underlying geometry
/// @param broadPhase A boolean dictating whether the broad-phase pre-check should be performed
/// @param maxQueryDepth An unsigned int specifying the maximum depth allowed during the search
/// (unsigned int between [0 16])
/// @param[out] p1Vec Closest point on the voxel nearest to the checked geometry
/// @param[out] p2Vec Closest point on the checked geometry to the nearest voxel
/// @param[out] centerPtr A NestedMatrixVoidWrapper containing an Nx3 set of voxel-centers that are
/// in collision with the geometry
/// @param[out] sizePtr A NestedMatrixVoidWrapper containing an Nx1 sizes of voxel which are in
/// collision
/// @param[out] center Center of AABB bounding the input geometry (Debugging only)
/// @param[out] pMin Minimum point in world coordinates of AABB bounding the input geometry
/// (Debugging only)
/// @param[out] pMax Maximum point in world coordinates of AABB bounding the input geometry
/// (Debugging only)
/// @return Returns true when a collision has been found, false otherwise
EXTERN_C COLLISIONMAP_CODEGEN_API boolean_T
checkmapcollisioncodegen_checkCollision(void* map,
                                        void* geom,
                                        const double* pos1d,
                                        const double* quat1d,
                                        real64_T* distance,
                                        const boolean_T exhaustive,
                                        const boolean_T narrowPhase,
                                        const boolean_T broadPhase,
                                        const uint32_T maxQueryDepth,
                                        real64_T p1Vec[3],
                                        real64_T p2Vec[3],
                                        void** centerPtr,
                                        void** sizePtr,
                                        real64_T center[3],
                                        real64_T pMin[3],
                                        real64_T pMax[3]);

#endif /* OCTOGEOM_COLLISION_FUNCTIONS_CODEGEN_API_H_ */
