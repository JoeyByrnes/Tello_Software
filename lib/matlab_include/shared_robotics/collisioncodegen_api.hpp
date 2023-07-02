// Copyright 2019-2022 The MathWorks, Inc.
/**
 * @file
 * @brief This files serves as a wrapper of the
 * collisioncodegen_checkCollision_api.hpp. This file is ANSI-C compliant.
 */

#ifndef COLLISIONCODEGEN_API_HPP
#define COLLISIONCODEGEN_API_HPP
#include "collisioncodegen_util.hpp"

typedef void *CollisionGeometryVoidPtr;

/**
 * @brief Create a collision geometry as a box primitive (the caller is
 * responsible for freeing the memory pointed by the returned pointer)
 * @param x The side length of the box along x-axis
 * @param y The side length of the box along y-axis
 * @param z The side length of the box along z-axis
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeBox(real64_T x, real64_T y, real64_T z);

/**
 * @brief Create a collision geometry as a sphere primitive (the caller is
 * responsible for freeing the memory pointed by the returned pointer)
 * @param r The radius of the sphere
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeSphere(real64_T r);

/**
 * @brief Create a collision geometry as a cylinder primitive (the caller is
 * responsible for freeing the memory pointed by the returned pointer)
 * @param r The radius of the cylinder
 * @param h The length of the cylinder
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeCylinder(real64_T r, real64_T h);

/**
 * @brief Create a collision geometry as a capsule primitive (the caller is
 * responsible for freeing the memory pointed by the returned pointer)
 * @param r The radius of the capsule
 * @param h The length of the capsule
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeCapsule(real64_T r, real64_T h);

/**
 * @brief Create a full-mesh primitive with faces (the caller is responsible for
 * freeing the memory pointed by the returned pointer)
 * @param vertices The vertices of the mesh
 * @param numVertices The number of vertices of the mesh
 * @return A void pointer to an instance  of CollisionGeometry
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_makeMesh(real64_T *vertices, real64_T numVertices);

/**
 * @brief Check collision between @p obj1 and @p obj2 if they are not in
 * collision, also computes the minimal distance and witness points if
 * requested.
 * @param[in] obj1 Pointer to @c CollisionGeometry object 1
 * @param[in] obj2 Pointer to @c CollisionGeometry object 2
 * @param[in] computeDistance An integer indicating whether to compute minimal
 * distance when checking collision. 1 - yes, 0 - no
 * @param[out] p1Vec Pointer to witness point on @p obj1
 * @param[out] p2Vec Pointer to witness point on @p obj2
 * @param[out] distance Minimal distance between @p obj1 and @p obj2, if they
 * are not in collision
 * @return The minimal distance between @p obj1 and @p obj2
 */
EXTERN_C COLLISIONCODEGEN_API int
collisioncodegen_intersect(CollisionGeometryVoidPtr obj1, real64_T *pos1,
                           real64_T *quat1, CollisionGeometryVoidPtr obj2,
                           real64_T *pos2, real64_T *quat2,
                           real64_T computeDistance, real64_T *p1Vec,
                           real64_T *p2Vec, real64_T *distance);

/**
 * @brief Delete the factory generated CollisionGeometryVoidPtr
 * @param[in] objPtr Pointer to a pointer of CollisionGeometry object
 */
EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_destructGeometry(const CollisionGeometryVoidPtr *objPtr);

/**
 * @brief Copy of the factory generated CollisionGeometryVoidPtr
 * @param[in] obj Pointer of CollisionGeometry object
 */
EXTERN_C COLLISIONCODEGEN_API CollisionGeometryVoidPtr
collisioncodegen_copyGeometry(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the vertices of the underlying geometry
 * @param[in] obj Pointer to a CollisionGeometry object
 * @param[in] vertices Pointer to the memory holding an array of vertices
 */
EXTERN_C COLLISIONCODEGEN_API void
collisioncodegen_getVertices(CollisionGeometryVoidPtr obj, real64_T *vertices);

/**
 * @brief Get the number of vertices of the underlying geometry
 * @param[in] obj Pointer to a CollisionGeometry object
 * @return The number of vertices
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getNumVertices(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the X dimension of the underlying geometry
 * @param[in] obj Pointer to a CollisionGeometry object
 * @return The X dimension
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getX(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the Y dimension of the underlying geometry
 * @param[in] obj Pointer to a CollisionGeometry object
 * @return The Y dimension
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getY(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the Z dimension of the underlying geometry
 * @param[in] obj Pointer to a CollisionGeometry object
 * @return The Z dimension
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getZ(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the Radius of the underlying geometry
 * @param[in] obj Pointer to a CollisionGeometry object
 * @return The Radius
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getRadius(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the Length of the underlying geometry
 * @param[in] obj Pointer to a pointer of CollisionGeometry object
 * @return The Length
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getLength(CollisionGeometryVoidPtr obj);

/**
 * @brief Get the string type of the underlying geometry
 * @param[in] obj Pointer to a pointer of CollisionGeometry object
 * @param[in] type Pointer to the memory holding the string
 */
EXTERN_C COLLISIONCODEGEN_API real64_T
collisioncodegen_getType(CollisionGeometryVoidPtr obj, char_T *type);

#endif
