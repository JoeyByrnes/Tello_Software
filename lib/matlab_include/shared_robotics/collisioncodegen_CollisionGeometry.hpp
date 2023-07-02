// Copyright 2018-2022 The MathWorks, Inc.

/**
 * @file
 * @brief Interface for collision geometry representation based on support mapping
 */

#ifndef COLLISIONCODEGEN_COLLISIONGEOMETRY_HPP
#define COLLISIONCODEGEN_COLLISIONGEOMETRY_HPP

#ifdef BUILDING_LIBMWCOLLISIONCODEGEN
#include <ccd/ccd_vec3.h> // need to use some data type definition
#include <ccd/ccd_quat.h> // for ccd quaternion functions
#else
#include <ccd_vec3.h> // need to use some data type definition
#include <ccd_quat.h> // for ccd quaternion functions
#endif
#include "collisioncodegen_helper.hpp"
#include <iostream>
#include <vector>


namespace shared_robotics {
class CollisionGeometry;
struct CollisionGeometryDeleter {
    void operator()(CollisionGeometry* ptr);
};

/**
 * @brief The @c CollisionGeometry class.
 * @details A collision geometry can be specified as a primitive (box, cylinder or sphere)
 *          or a convex mesh (vertices only, or vertices and faces). ccd algorithm interacts
 *          with a collision geometry through its support function
 */
class COLLISIONCODEGEN_API CollisionGeometry {
  public:
    /// enum for collision geometry types
    enum Type { Box, Sphere, Cylinder, ConvexMesh, ConvexMeshFull, Capsule };

    /// destructor
    virtual ~CollisionGeometry();

    /// constructor for box primitive
    CollisionGeometry(ccd_real_t x, ccd_real_t y, ccd_real_t z)
        : m_x(x)
        , m_y(y)
        , m_z(z) {
        initializePositionAndOrientation();
        m_type = Box;
    }

    /// constructor for sphere primitive
    CollisionGeometry(ccd_real_t radius)
        : m_radius(radius) {
        initializePositionAndOrientation();
        m_type = Sphere;
    }

    /// constructor for cylinder primitive
    CollisionGeometry(ccd_real_t radius, ccd_real_t height)
        : m_radius(radius)
        , m_height(height) {
        initializePositionAndOrientation();
        m_type = Cylinder;
    }

    void setEnumType(Type type) {
        m_type = type;
    }


    /// constructor for convex mesh (vertices only)
    CollisionGeometry(ccd_real_t* vertices, int numVertices, bool isColumnMajor = true) {
        initializePositionAndOrientation();
        m_type = ConvexMesh;
        m_numVertices = static_cast<std::size_t>(numVertices);

        ccd_vec3_t vert = {{0, 0, 0}};
        std::size_t numV = m_numVertices;

        if (isColumnMajor) {
            for (std::size_t k = 0; k < numV; k++) {
                ccdVec3Set(&vert, vertices[k], vertices[numV + k], vertices[2 * numV + k]);
                m_vertices.push_back(vert);
            }
        } else {
            for (std::size_t k = 0; k < numV; k++) {
                ccdVec3Set(&vert, vertices[3 * k], vertices[3 * k + 1], vertices[3 * k + 2]);
                m_vertices.push_back(vert);
            }
        }
    }


    /// constructor for convex mesh (vertices and faces)
    CollisionGeometry(ccd_real_t* vertices,
                      int* faces,
                      int numVertices,
                      int numFaces,
                      bool isColumnMajor = true) {
        initializePositionAndOrientation();
        m_type = ConvexMeshFull;
        m_numVertices = static_cast<std::size_t>(numVertices);
        m_numFaces = static_cast<std::size_t>(numFaces);

        ccd_vec3_t vert = {{0, 0, 0}};
        idx_vec3_t face = {{0, 0, 0}};
        std::size_t numV = m_numVertices;
        std::size_t numF = m_numFaces;

        if (isColumnMajor) {
            for (std::size_t k = 0; k < numV; k++) {
                ccdVec3Set(&vert, vertices[k], vertices[numV + k], vertices[2 * numV + k]);
                m_vertices.push_back(vert);
            }

            for (std::size_t q = 0; q < numF; q++) {
                idxVec3Set(&face, static_cast<std::size_t>(faces[q]),
                           static_cast<std::size_t>(faces[numF + q]),
                           static_cast<std::size_t>(faces[2 * numF + q]));
                m_faces.push_back(face);
            }
        } else {
            for (std::size_t k = 0; k < numV; k++) {
                ccdVec3Set(&vert, vertices[3 * k], vertices[3 * k + 1], vertices[3 * k + 2]);
                m_vertices.push_back(vert);
            }

            for (std::size_t q = 0; q < numF; q++) {
                idxVec3Set(&face, static_cast<std::size_t>(faces[3 * q]),
                           static_cast<std::size_t>(faces[3 * q + 1]),
                           static_cast<std::size_t>(faces[3 * q + 2]));
                m_faces.push_back(face);
            }
        }

        constructConnectionList();
    }

    /// query the enum type of the collision geometry
    Type getEnumType() const;

    /// query the type of the collision geometry
    std::string getType() const;

    /// query the X dimension of the collision geometry
    ccd_real_t getX() const;

    /// query the Y dimension of the collision geometry
    ccd_real_t getY() const;

    /// query the Z dimension of the collision geometry
    ccd_real_t getZ() const;

    /// query the radius of the collision geometry
    ccd_real_t getRadius() const;

    /// query the height of the collision geometry
    ccd_real_t getHeight() const;

    /// query the number of vertices of the collision geometry
    std::size_t getNumVertices() const;

    /// query the number of faces of the collision geometry
    std::size_t getNumFaces() const;

    /// query the vertices of the collision geometry
    const std::vector<ccd_vec3_t>& getVertices() const;

    /// query the faces of the collision geometry
    const std::vector<idx_vec3_t>& getFaces() const;

    /// query the connection list of the collision geometry
    const std::vector<std::vector<std::size_t>>& getConnectionList() const;


    /// support function for libccd interface
    void support(const ccd_vec3_t* dir, ccd_vec3_t* supportVertex) const;

  protected:
    /// sign function
    int sign(ccd_real_t val) const;

    /// must only be invoked after m_vertices and m_faces are initialized
    void constructConnectionList();


    /// type of the geometry
    Type m_type;


    /// dimension x of the box, if the geometry is a box primitive
    ccd_real_t m_x;

    /// dimension x of the box, if the geometry is a box primitive
    ccd_real_t m_y;

    /// dimension x of the box, if the geometry is a box primitive
    ccd_real_t m_z;


    /// radius of the sphere, if the geometry is a sphere primitive, or radius of the bottom of a
    /// cylinder, if the geometry is a cylinder primitive
    ccd_real_t m_radius;


    /// height of the cylinder, if the geometry is a cylinder primitive
    ccd_real_t m_height;


    /// number of vertices in the convex mesh, if the geometry is a convex primitive
    std::size_t m_numVertices;

    /// number of faces of the convex mesh, if the geometry is a convex primitive
    std::size_t m_numFaces;

    /// vertices of the mesh (if the geometry is a mesh primitive)
    /// vertices is saved as a vector of ccd_vec3_t structs with m_numVertices elements
    std::vector<ccd_vec3_t> m_vertices;

    /// faces of the convex mesh (if the geometry is a mesh primitive)
    /// faces is save as an (m_numFaces)-by-1 vector of idx_vec3_t structs consisting of node
    /// indices
    std::vector<idx_vec3_t> m_faces;

    /// connection list for each node int the mesh
    std::vector<std::vector<std::size_t>> m_connectionList;

  public:
    /// position of the geometry
    ccd_vec3_t m_pos;

    /// orientation of the geometry (ccd assuming [x, y, z, w] sequence)
    ccd_quat_t m_quat;

  private:
    void initializePositionAndOrientation() {
        /// position of the geometry which is an array of size 3
        for (int i = 0; i < 3; i++) {
            m_pos.v[i] = 0;
        }

        /// orientation of the geometry which is an array of size 4 (ccd assuming [x, y, z, w]
        /// sequence)
        for (int i = 0; i < 3; i++) {
            m_quat.q[i] = 0;
        }
        m_quat.q[3] = 1;
    }
};

/// sign function
inline int shared_robotics::CollisionGeometry::sign(ccd_real_t val) const {

    if (CCD_FABS(val) < CCD_EPS) {
        return 0;
    } else if (val < 0.0) {
        return -1;
    }
    return 1;
}

inline shared_robotics::CollisionGeometry::Type shared_robotics::CollisionGeometry::getEnumType() const {
    return m_type;
}

inline std::string shared_robotics::CollisionGeometry::getType() const {
    switch (m_type) {
    case Box:
        return "Box";
    case Sphere:
        return "Sphere";
    case Cylinder:
        return "Cylinder";
    case ConvexMeshFull:
        return "ConvexMeshFull";
    case Capsule:
        return "Capsule";
    default:
        return "ConvexMesh";
    }
}

inline ccd_real_t shared_robotics::CollisionGeometry::getX() const {
    return m_x;
}
inline ccd_real_t shared_robotics::CollisionGeometry::getY() const {
    return m_y;
}
inline ccd_real_t shared_robotics::CollisionGeometry::getZ() const {
    return m_z;
}
inline ccd_real_t shared_robotics::CollisionGeometry::getRadius() const {
    return m_radius;
}
inline ccd_real_t shared_robotics::CollisionGeometry::getHeight() const {
    return m_height;
}
inline std::size_t shared_robotics::CollisionGeometry::getNumVertices() const {
    return m_numVertices;
}
inline std::size_t shared_robotics::CollisionGeometry::getNumFaces() const {
    return m_numFaces;
}
inline const std::vector<ccd_vec3_t>& shared_robotics::CollisionGeometry::getVertices() const {
    return m_vertices;
}
inline const std::vector<idx_vec3_t>& shared_robotics::CollisionGeometry::getFaces() const {
    return m_faces;
}

inline const std::vector<std::vector<std::size_t>>&
shared_robotics::CollisionGeometry::getConnectionList() const {
    return m_connectionList;
}


} // namespace shared_robotics
#endif
