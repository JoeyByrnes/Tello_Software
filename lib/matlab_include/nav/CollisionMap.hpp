/* Copyright 2022 The MathWorks, Inc. */
#ifndef COLLISIONMAP_CODEGEN_HPP
#define COLLISIONMAP_CODEGEN_HPP

#ifdef BUILDING_LIBMWCOLLISIONMAPCODEGEN
#include "collisionmapcodegen/AABB.hpp"
#include "collisionmapcodegen/CollisionMap.hpp"
#include "collisionmapcodegen/collisionmapcodegen_util.hpp"
#include "collisionmapcodegen/collisionmapcodegen_types.hpp"
#include "octomap/OcTree.h"
#else
#include "AABB.hpp"
#include "CollisionMap.hpp"
#include "collisionmapcodegen_util.hpp"
#include "collisionmapcodegen_types.hpp"
#include "OcTree.h"
#endif

#include <type_traits>
#include <iterator>
#include <memory>
#include <algorithm>

using OcNode = octomap::OcTreeNode;
using OcMapPtr = std::shared_ptr<octomap::OcTree>;
using OT = octomap::OcTree;
using OcKey = octomap::OcTreeKey;

namespace nav {
/// @class CollisionMap
/// @brief The @c CollisionMap class
/// @details A wrapper class for the octomap::OcTree, which provides an interface for collision
///         checking against nav::AABB and shared_robotics::CollisionGeometry objects.
class COLLISIONMAP_CODEGEN_API CollisionMap {
  public:
    /// @brief Creates a lightweight collision-checking wrapper around an OcTree object
    /// @param ocMapRoot An octomap::OcTree object
    CollisionMap(OcMapPtr ocMapRoot);

    /// @brief Returns whether the specified node in an OcTree is occupied or not
    /// @param node An octomap::OcTreeNode
    /// @return Returns a boolean, true when the node is occupied, false otherwise
    bool isNodeOccupied(const OcNode& node);

    /// @brief Generates a nav::AABB from a key at the specified depth
    /// @param key An octomap::OcTreeKey object
    /// @param depth An unsigned int between [0 nav::MAX_TREE_DEPTH], specifying the depth in the
    /// tree (root = 0)
    /// @return Returns a nav::AABB object
    AABB createAABB(const OcKey& key, const depth_t depth = MAX_TREE_DEPTH);

    /// @brief Returns the length of a voxel edge at the requested depth
    /// @param depth An unsigned int between [0 nav::MAX_TREE_DEPTH], specifying the depth in the
    /// tree (root = 0)
    /// @return Length of edge
    inline double voxelSizeAtDepth(const depth_t depth) {
        return m_tree->getResolution() * (1 << (MAX_TREE_DEPTH - depth));
    }

    /// @brief Sets an axially-aligned region on the tree
    /// @details During collision checking, only voxels which fall within this AABB region
    ///             will be checked for collision
    /// @param box A nav::AABB whose volume defines the bounded region on the tree
    /// @param depth An unsigned int between [0 nav::MAX_TREE_DEPTH], specifying the depth in the
    /// tree (root = 0)
    /// @return Boolean indicating whether any portion of the BBOX falls within the limits of the map
    bool setBBXOnTree(const AABB& box, const depth_t depth = MAX_TREE_DEPTH);

    /// @brief Returns the min and max keys for a voxel containing the input key
    /// @details Takes in an OcTreeKey and a depth-offset from the base depth. The function
    ///          then floors the key by the offset, resulting in a key lying at the min corner
    ///          of a voxel located at maxQueryDepth-levelFromMaxDepth. The max corner key is
    ///          created by shifting the min key by the size of a voxel at the indicated depth.
    /// @param key The original octomap::OcTreeKey
    /// @param levelFromMaxDepth The number of levels above the base depth
    /// @return Returns a pair of octomap::OcTreeKey objects, representing the min and max keys
    inline std::tuple<OcKey, OcKey> shiftKeyToVoxelCorners(const OcKey& key,
                                                           const unsigned short levelFromMaxDepth);

    /// @brief Returns whether an octomap::OcTreeKey at a given depth lies within the AABB region
    /// @param key An octomap::OcTreeKey object
    /// @param currentDepth An unsigned int between [0 nav::MAX_TREE_DEPTH], specifying the depth in
    /// the tree (root = 0)
    /// @return True if the key touches the region, false otherwise
    bool inBaseBBX(const OcKey& key, const depth_t currentDepth);

    /// @brief Performs a broad-phase pre-check between a key and the map at a given depth
    /// @param broadPhase When true, the check is performed, otherwise the pre-check always returns
    /// true
    /// @param key An octomap::OcTreeKey object
    /// @param currentDepth An unsigned int between [0 nav::MAX_TREE_DEPTH], specifying the depth in
    /// the tree (root = 0)
    /// @return Returns true when the broad-phase check indicates a collision, false otherwise
    bool broadPhasePrecheck(const bool broadPhase, const OcKey& key, const depth_t currentDepth);

    /// @brief Performs a non-exhaustive check between the tree and a simple AABB-bounded object
    /// @param box A nav::AABB object
    /// @param[out] key An octomap::OcTreeKey object corresponding to the first voxel in collision
    /// with the nav::AABB
    /// @param[out] collDepth The depth at which the collision occurs
    /// @return Returns true when a collision has been found, false otherwise
    bool checkCollision_BroadPhase(const AABB& box,
                                   std::vector<OcKey>& key,
                                   std::vector<depth_t>& collDepth);

    /// @brief Performs a non-exhaustive AABB-only check, limited to the specified depth in the tree
    /// @param box A nav::AABB object
    /// @param[out] key An octomap::OcTreeKey object corresponding to the first voxel in collision
    /// with the nav::AABB
    /// @param[out] collDepth The depth at which the collision occurs
    /// @param maxQueryDepth The maximum depth allowed during the search (unsigned int between [0
    /// nav::MAX_TREE_DEPTH])
    /// @return Returns true when a collision has been found, false otherwise
    bool checkCollision_BroadPhaseAtDepth(const AABB& box,
                                          std::vector<OcKey>& key,
                                          std::vector<depth_t>& collDepth,
                                          const depth_t maxQueryDepth);

    /// @brief Performs an exhaustive AABB-only check
    /// @param box A nav::AABB object
    /// @param[out] keys A vector of keys corresponding to all voxels in collision with the
    /// nav::AABB
    /// @param[out] collDepth A vector of depths corresponding to colliding voxels
    /// @return Returns true when a collision has been found, false otherwise
    bool checkCollision_BroadPhaseExhaustive(const AABB& box,
                                             std::vector<OcKey>& keys,
                                             std::vector<depth_t>& collDepth);

    /// @brief Performs an exhaustive AABB-only check, limited to the specified depth in the tree
    /// @param box A nav::AABB object
    /// @param[out] keys A vector of keys corresponding to all voxels in collision with the
    /// nav::AABB
    /// @param[out] collDepth A vector of depths corresponding to colliding voxels
    /// @param maxQueryDepth The maximum depth allowed during the search (unsigned int between [0
    /// nav::MAX_TREE_DEPTH])
    /// @return Returns true when a collision has been found, false otherwise
    bool checkCollision_BroadPhaseExhaustiveAtDepth(const AABB& box,
                                                    std::vector<OcKey>& keys,
                                                    std::vector<depth_t>& collDepth,
                                                    const depth_t maxQueryDepth);

    /// @brief Generates a new key located at the center of the voxel containing the input key at
    /// the specified depth
    /// @param key An octomap::OcTreeKey object
    /// @param currentDepth The depth at which the key should be shifted
    /// @return The shifted octomap::OcTreeKey object
    OcKey convertToBaseKey(const OcKey& key, const depth_t& currentDepth);

    /// @brief Calculates the closest distance between voxels in the map and a nav::AABB object
    /// @param aabb A nav::AABB object
    /// @param[out] p1Vec Closest point on the voxel nearest to the nav::AABB
    /// @param[out] p2Vec Closest point on the nav::AABB to the nearest voxel
    /// @param maxQueryDepth The maximum depth allowed during the search (unsigned int between [0
    /// nav::MAX_TREE_DEPTH])
    /// @return Distance between closest points p1Vec/p2Vec
    double broadPhaseDistance(AABB& aabb,
                              double* p1Vec,
                              double* p2Vec,
                              const depth_t maxQueryDepth);

    /// @brief Get voxel key corresponding to min corner of active bounding box
    /// @return Min voxel key
    OcKey minKey(void) { return m_tree->coordToKey(m_tree->getBBXMin()); }

    /// @brief Get voxel key corresponding to max corner of active bounding box
    /// @return Max voxel key
    OcKey maxKey(void) { return m_tree->coordToKey(m_tree->getBBXMax()); }

  protected:
    depth_t m_boxDepth = MAX_TREE_DEPTH;
    OcKey m_minKey;
    OcKey m_maxKey;
    std::shared_ptr<OT> m_tree;
};

inline std::tuple<OcKey, OcKey> CollisionMap::shiftKeyToVoxelCorners(
    const OcKey& key,
    const unsigned short levelFromMaxDepth) {
    // Convert node key to corner keys
    OcKey minNodeKey = key;

    // Slice off offset marking node center (results in botLeft corner)
    std::for_each(minNodeKey.k, minNodeKey.k + 3u, [&](octomap::key_type& kVal) {
        kVal = static_cast<unsigned short>(static_cast<unsigned short>(kVal >> levelFromMaxDepth)
                                           << levelFromMaxDepth);
    });

    // Shift up by current level's node dimension (results in topRight corner)
    OcKey maxNodeKey = minNodeKey;
    std::for_each(maxNodeKey.k, maxNodeKey.k + 3u,
                  [&](octomap::key_type& kVal) { kVal += ((1u << levelFromMaxDepth) - 1u); });

    return std::tie(minNodeKey, maxNodeKey);
}

/// templated helper functions for casting between datatypes
template <class Tout, class Tin>
std::vector<Tout> castVec(const std::vector<Tin>& vIn) {
    std::vector<Tout> out;
    out.reserve(vIn.size());
    std::transform(vIn.begin(), vIn.end(), out.begin(), [](Tin x) { return static_cast<Tout>(x); });
    return out;
}

/// templated helper functions for casting between datatypes
template <class Container, class T = typename Container::value_type>
void castVec(const octomap::point3d& vIn, Container& out) {
    out = {vIn(1), vIn(2), vIn(3)};
}

/// templated helper functions for casting between datatypes
template <class OutContainer,
          typename Tout = typename OutContainer::value_type,
          class InContainer,
          class t_size>
OutContainer castVec(const InContainer& input, const t_size& size) {
    std::allocator<typename OutContainer::value_type> alloc;
    using traits_t = std::allocator_traits<decltype(alloc)>;
    std::vector<Tout> out(size);
    traits_t::construct(alloc, out, size);
    size_t i = 0;
    std::for_each(&input[0], &input[0] + size, [&out, &i](decltype(input[0])& x) {
        out[i] = static_cast<Tout>(x);
        i++;
    });
    return out;
}

/// templated helper functions for casting between datatypes
template <typename T>
inline octomap::point3d castVec(const std::vector<T>& vIn) {
    return octomap::point3d(static_cast<float>(vIn[0]), static_cast<float>(vIn[1]),
                            static_cast<float>(vIn[2]));
}

/// templated helper functions for casting between datatypes
template <typename T>
inline const std::vector<T> castVec(octomap::point3d vIn) {
    std::vector<T> out{vIn(0), vIn(1), vIn(2)};
    return out;
}
} // namespace nav

#endif /* COLLISIONMAP_CODEGEN_HPP */
