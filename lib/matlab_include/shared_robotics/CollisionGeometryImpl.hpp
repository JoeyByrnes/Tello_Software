// Copyright 2018-2022 The MathWorks, Inc.

/**
 * @file
 * This file contains the wrapper for the CollisionGeometry class.
 */

#ifndef COLLISION_GEOMETRY_IMPL
#define COLLISION_GEOMETRY_IMPL

#include <memory>
#include <vector>
#include <string>

#include <mex.h>
#include <matrix.h>
#include <mcos_factory/mi.hpp>          // for marshaling scalar types
#include <sl_services_mi/slsv_mcos.hpp> // for marshaling std::vector<double>
#include "collisionbuiltins/collisionbuiltins_util.hpp"

namespace shared_robotics {
/// forward declare the class that represents collision geometry
class CollisionGeometry;

/**
 * @brief Wrap the CollisionGeometry class
 */
class COLLISION_BUILTINS_API CollisionGeometryImpl {
  public:
    /**
     * @brief Creates @c CollisionGeometryImpl object through a sphere primitive
     * @param radius Radius of the sphere
     */
    CollisionGeometryImpl(double radius);

    /**
     * @brief Creates @c CollisionGeometryImpl object through a cylinder primitive
     * @param radius Radius of the cylinder
     * @param height Height of the cylinder
     */
    CollisionGeometryImpl(double radius, double height);

    /**
     * @brief Creates @c CollisionGeometryImpl object through an axis-aligned box primitive
     * @param x Length along the x axis
     * @param y Length along the y axis
     * @param z Length along the z axis
     */
    CollisionGeometryImpl(double x, double y, double z);

    /**
     * @brief Creates @c CollisionGeometryImpl object through vertices from a convex mesh
     * @param vertices Vertices of a convex mesh
     * @param numVertices Number of vertices
     */
    CollisionGeometryImpl(const mcos::factory::mxArrayProxy& vertices, double numVertices);

    /**
     * @brief Creates @c CollisionGeometryImpl object through vertices and faces from a convex mesh
     * @param vertices Vertices of a convex mesh
     * @param faces Faces of a convex mesh
     * @param numVertices Number of vertices
     * @param numFaces Number of faces
     */
    CollisionGeometryImpl(const mcos::factory::mxArrayProxy& vertices,
                          const mcos::factory::mxArrayProxy& faces,
                          double numVertices,
                          double numFaces);

    /**
     * @brief Destructor
     */
    ~CollisionGeometryImpl();

    /**
     * @brief Get collision geometry type
     * @return Type string
     */
    std::string getType() const;

    /**
     * @brief Set collision geometry type
     * @param type Type of collision geometry as a string
     */
    void setType(std::string type);


    /**
     * @brief Compute the support point for a certain support direction.
     * @param dir A 3d vector that represents the support direction
     * @param pos A 3d vector that represents the position of the geometry
     * @param quat A 4-vector that represents the orientation of the geometry (a quaternion)
     * @return The support point as a 3 vector
     */
    mcos::factory::mxArrayProxy support(mcos::factory::mxArrayProxy& dir,
                                        mcos::factory::mxArrayProxy& pos,
                                        mcos::factory::mxArrayProxy& quat);

  public:
    /// pointer to the underlying geometry representation
    std::unique_ptr<shared_robotics::CollisionGeometry> m_geometry;
};
} // namespace shared_robotics

#endif
