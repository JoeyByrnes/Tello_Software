/* Copyright 2022 The MathWorks, Inc. */
#ifndef COLLISIONFUNCTIONS_CODEGEN_HPP
#define COLLISIONFUNCTIONS_CODEGEN_HPP

#ifdef BUILDING_LIBMWCOLLISIONMAPCODEGEN
// Local dependencies
#include "collisionmapcodegen/CollisionMap.hpp"
#include "collisionmapcodegen/collisionmapcodegen_types.hpp"
// External dependencies
#include "octomapcodegen/octomapcodegen.hpp"
#include "collisioncodegen/collisioncodegen_CollisionGeometry.hpp"
#include "collisioncodegen/collisioncodegen_checkCollision_api.hpp"
#include "collisioncodegen/collisioncodegen_helper.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
// Local dependencies
#include "CollisionMap.hpp"
#include "collisionmapcodegen_types.hpp"
// External dependencies
#include "octomapcodegen.hpp"
#include "collisioncodegen_CollisionGeometry.hpp"
#include "collisioncodegen_checkCollision_api.hpp"
#include "collisioncodegen_helper.hpp"
#endif

namespace nav {
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
/// @param[out] collisionCenters An Nx3 set of voxel-centers that are in collision with the geometry
/// @param[out] edgeLength An Nx1 set of lengths corresponding to voxels in collision
/// @param[out] boxCtr Center of AABB bounding the input geometry (Debugging only)
/// @param[out] pMin Minimum point in world coordinates of AABB bounding the input geometry
/// (Debugging only)
/// @param[out] pMax Maximum point in world coordinates of AABB bounding the input geometry
/// (Debugging only)
/// @return Returns true when a collision has been found, false otherwise
COLLISIONMAP_CODEGEN_API bool octomapCheckCollision_impl(
    void* map,
    const void* geom,
    double& distance,
    const bool exhaustive,
    const bool narrowPhase,
    const bool broadPhase,
    const depth_t maxQueryDepth,
    double* p1Vec,
    double* p2Vec,
    std::vector<std::array<double, 3>>& collisionCenters,
    std::vector<double>& edgeLength,
    double* boxCtr,
    double* pMin,
    double* pMax);

/// @brief Checks for collision between a geometry and the map, exiting immediately upon the first
/// collision
/// @param map A pointer to the nav::octomapcodegen-wrapped octomap::OcTree class
/// @param geom A pointer to a shared_robotics::CollisionGeometry object
/// @param geomAABB Global AABB for geom object
/// @param broadPhase A boolean dictating whether the broad-phase pre-check should be performed
/// @param computeDistance Flag dictating whether distance should be returned. Setting this to 0
/// will skip the distance check
/// @param[out] p1Vec Closest point on the voxel nearest to the checked geometry
/// @param[out] p2Vec Closest point on the checked geometry to the nearest voxel
/// @param[out] key octomap::OcTreeKey corresponding to voxel in collision with geometry
/// @param[out] collDepth Depth corresponding to colliding voxel
/// @param[out] distance Distance between closest points on voxel and geometry
/// @return Returns true when a collision has been found, false otherwise
bool checkCollision_FullCheck_EarlyExit(void* map,
                                        const void* geom,
                                        const AABB& geomAABB,
                                        const bool broadPhase,
                                        const int& computeDistance,
                                        double* p1Vec,
                                        double* p2Vec,
                                        std::vector<OcKey>& key,
                                        std::vector<depth_t>& collDepth,
                                        double* distance);

/// @brief Checks for collision up to the specified depth, exiting immediately upon the first
/// collision
/// @param map A pointer to the nav::octomapcodegen-wrapped octomap::OcTree class
/// @param geom A pointer to a shared_robotics::CollisionGeometry object
/// @param geomAABB Global AABB for geom object
/// @param broadPhase A boolean dictating whether the broad-phase pre-check should be performed
/// @param computeDistance Flag dictating whether distance should be returned. Setting this to 0
/// will skip the distance check
/// @param[out] p1Vec Closest point on the voxel nearest to the checked geometry
/// @param[out] p2Vec Closest point on the checked geometry to the nearest voxel
/// @param[out] key octomap::OcTreeKey corresponding to voxel in collision with geometry
/// @param[out] collDepth Depth corresponding to colliding voxel
/// @param maxQueryDepth Maximum depth allowed during traversal of tree
/// @param[out] distance Distance between closest points on voxel and geometry
/// @return Returns true when a collision has been found, false otherwise
bool checkCollision_FullCheck_EarlyExitAtDepth(void* map,
                                               const void* geom,
                                               const AABB& geomAABB,
                                               const bool broadPhase,
                                               const int& computeDistance,
                                               double* p1Vec,
                                               double* p2Vec,
                                               std::vector<OcKey>& key,
                                               std::vector<depth_t>& collDepth,
                                               const depth_t maxQueryDepth,
                                               double* distance);

/// @brief Exhaustively checks for collision, and will return all voxels that are in collision with
/// the input geometry
/// @param map A pointer to the nav::octomapcodegen-wrapped octomap::OcTree class
/// @param geom A pointer to a shared_robotics::CollisionGeometry object
/// @param geomAABB Global AABB for geom object
/// @param broadPhase A boolean dictating whether the broad-phase pre-check should be performed
/// @param computeDistance Flag dictating whether distance should be returned. Setting this to 0
/// will skip the distance check
/// @param[out] p1Vec Closest point on the voxel nearest to the checked geometry
/// @param[out] p2Vec Closest point on the checked geometry to the nearest voxel
/// @param[out] keys octomap::OcTreeKey corresponding to voxel in collision with geometry
/// @param[out] collDepth Depth corresponding to colliding voxel
/// @param[out] distance Distance between closest points on voxel and geometry
/// @return Returns true when a collision has been found, false otherwise
bool checkCollision_FullCheck_Exhaustive(void* map,
                                         const void* geom,
                                         const AABB& geomAABB,
                                         const bool broadPhase,
                                         const int& computeDistance,
                                         double* p1Vec,
                                         double* p2Vec,
                                         std::vector<OcKey>& keys,
                                         std::vector<depth_t>& collDepth,
                                         double* distance);

/// @brief Exhaustively checks for collision up to the specified depth
/// @param map A pointer to the nav::octomapcodegen-wrapped octomap::OcTree class
/// @param geom A pointer to a shared_robotics::CollisionGeometry object
/// @param geomAABB Global AABB for geom object
/// @param broadPhase A boolean dictating whether the broad-phase pre-check should be performed
/// @param computeDistance Flag dictating whether distance should be returned. Setting this to 0
/// will skip the distance check
/// @param[out] p1Vec Closest point on the voxel nearest to the checked geometry
/// @param[out] p2Vec Closest point on the checked geometry to the nearest voxel
/// @param[out] keys octomap::OcTreeKey corresponding to voxel in collision with geometry
/// @param[out] collDepth Depth corresponding to colliding voxel
/// @param maxQueryDepth Maximum depth allowed during traversal of tree
/// @param[out] distance Distance between closest points on voxel and geometry
/// @return Returns true when a collision has been found, false otherwise
bool checkCollision_FullCheck_ExhaustiveAtDepth(void* map,
                                                const void* geom,
                                                const AABB& geomAABB,
                                                const bool broadPhase,
                                                const int& computeDistance,
                                                double* p1Vec,
                                                double* p2Vec,
                                                std::vector<OcKey>& keys,
                                                std::vector<depth_t>& collDepth,
                                                const depth_t maxQueryDepth,
                                                double* distance);

/// @brief Computes the nav::AABB of a given shared_robotics::CollisionGeometry object in the
/// object's locate frame
/// @param geom A shared_robotics::CollisionGeometry object
/// @param[out] bbox A nav::AABB object
void computeBV(const shared_robotics::CollisionGeometry* geom, nav::AABB& bbox);

/// @brief Computes the nav::AABB of a given shared_robotics::CollisionGeometry object in the world
/// frame
/// @param geom A shared_robotics::CollisionGeometry object
/// @param[out] bbox A nav::AABB object
/// encapsulates the geometry
void computeGlobalBV(const shared_robotics::CollisionGeometry* geom,
                     nav::AABB& bbox);

/// @brief Calculates the local AABB for a rotated collision geometry
/// @param geom Ptr to a shared_robotics::CollisionGeometry object
/// @param[out] bbox An axially aligned bounding box object
void computeRotatedBV(const shared_robotics::CollisionGeometry* geom, AABB& bbox);

/// @brief Derives AABB min/max corners via projection of parametric 3D circle to each 2D plane.
///
/// A parametric circle can be defined as P(t) = C + r*cos(t)*U + r*sin(t)*V, where
/// V = (U x N), s.t. U,V,N are orthonormal vectors, with N normal to the circle's face.
/// Applying a rotation, R, to the circle is equivalent to Np = R*N, and since the original
/// face of the cylinder uses N = [0 0 1] it follows that R = [Up(:) Vp(:) Np(:)].
/// 
/// Taking the derivative of rotated P(t) gives R*P'(t) = r*sin(t)*Up - r*cos(t)*Vp, which can
/// be set to 0 and solve for t in each dimension separately. Each t correspond to an extrema
/// in the corresponding dimension. Adding the absolute value of these extrema to the absolute
/// value face-center point gives pMax, with pMin = -pMax due to reflective symmetry about origin.
///
/// @param cyl Ptr to cylinder collision geometry
/// @return <pMin, pMax>
std::pair<ccd_vec3_t, ccd_vec3_t> cylBounds(const shared_robotics::CollisionGeometry* cyl);

/// @brief Calculates AABB corners for origin-centered capsule obtained by adding radius to
/// rotated cylinder-face centerpoint in each dimension. Min point is reflected max point.
/// @param cap Ptr to capsule collision geometry
/// @return <pMin, pMax>
std::pair<ccd_vec3_t, ccd_vec3_t> capBounds(const shared_robotics::CollisionGeometry* cap);

/// @brief Calculates AABB corners for origin-centered box
/// @param box Ptr to box collision geometry
/// @return <pMin, pMax>
std::pair<ccd_vec3_t, ccd_vec3_t> boxBounds(const shared_robotics::CollisionGeometry* box);

/// @brief Applies rotation to set of points and tracks min/max
/// @param v libCCD vector of xyz vertices
/// @param q libCCD vector of xyzw quaternion values
/// @return <pMin, pMax>
std::pair<ccd_vec3_t, ccd_vec3_t> boundRotatedPoints(const std::vector<ccd_vec3_t>& v, const ccd_quat_t* q);

} // namespace nav

#endif /* COLLISIONFUNCTIONS_CODEGEN_HPP */
