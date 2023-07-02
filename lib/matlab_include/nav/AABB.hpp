/* Copyright 2022 The MathWorks, Inc. */
#ifndef AABB_CODEGEN_HPP
#define AABB_CODEGEN_HPP
#include <algorithm> // for std::min and std::max
#include <vector>
#include <cmath>

namespace nav {
/**
 * @brief The @c AABB class
 * @details An axially-aligned bounding box with helper utilities used for simple
 * collision-checking. This class is primarily used by nav::CollisionMap to support geometry vs
 * octomap collision-checks.
 */
class AABB {
  public:
    /// minimum point in the local grid frame [xMin yMin zMin]
    std::vector<double> m_localMin;

    /// maximum point in the local grid frame [xMax yMax zMax]
    std::vector<double> m_localMax;

    /// position of local frame relative to world frame [x y z]
    std::vector<double> m_localOrigin;

    /// default constructor (creates 1x1x1 box centered at [0 0 0])
    AABB()
        : m_localMin({-0.5, -0.5, -0.5})
        , m_localMax({0.5, 0.5, 0.5})
        , m_localOrigin({0, 0, 0}) {}

    /// @brief Constructs a nav::AABB with all sides of equal length (2*halfLength), centered at
    /// location
    /// @param halfLength Distance of local min/max from center point in all 3 dimensions
    /// @param location Location of AABB local frame [x y z]
    AABB(const double halfLength, const double location[])
        : m_localMin({-halfLength, -halfLength, -halfLength})
        , m_localMax({halfLength, halfLength, halfLength})
        , m_localOrigin({location[0], location[1], location[2]}) {}

    /// @brief Construct a nav::AABB using local min/max coordinates. Box is centered at [0,0,0]
    /// @param xMin Min corner in x-dimension
    /// @param yMin Min corner in y-dimension
    /// @param zMin Min corner in z-dimension
    /// @param xMax Max corner in x-dimension
    /// @param yMax Max corner in y-dimension
    /// @param zMax Max corner in z-dimension
    AABB(const double xMin,
         const double yMin,
         const double zMin,
         const double xMax,
         const double yMax,
         const double zMax)
        : m_localMin({xMin, yMin, zMin})
        , m_localMax({xMax, yMax, zMax})
        , m_localOrigin({0, 0, 0}) {}

    /// @brief Constructs a nav::AABB with all sides of equal length (2*halfLength), centered at
    /// location
    /// @param halfLength Distance of local min/max from center point in all 3 dimensions
    /// @param center Location of AABB local frame [x y z]
    AABB(double halfLength, const std::vector<double>& center) {
        AABB aabb(halfLength, center.data());
        *this = aabb;
    }

    /// @brief Construct a nav::AABB by specifying min, max, and center location.
    /// @param pMin Local min [xMin, yMin, zMin]
    /// @param pMax Local max [xMax, yMax, zMax]
    /// @param center Location of AABB local frame [x y z]
    AABB(const double* pMin, const double* pMax, const double* center)
        : m_localMin({pMin[0], pMin[1], pMin[2]})
        , m_localMax({pMax[0], pMax[1], pMax[2]})
        , m_localOrigin({center[0], center[1], center[2]}) {}

    /// Returns the min corner of AABB in world coordinates
    inline std::vector<double> worldMin(void) const {
        return AABB::shiftToWorld(m_localMin);
    }

    /// Returns the max corner of AABB in world coordinates
    inline std::vector<double> worldMax(void) const {
        return AABB::shiftToWorld(m_localMax);
    }

    /// Equal operator for test verification
    friend bool operator==(const AABB& obj, const AABB& other) {
        return (&obj == &other ||
                (obj.m_localMin == other.m_localMin && obj.m_localMax == other.m_localMax &&
                 obj.m_localOrigin == other.m_localOrigin));
    }

    /// @brief Checks whether two AABB overlap
    /// @param other The second AABB object
    /// @return Returns true if the AABB overlap, false otherwise
    bool overlap(const AABB& other) {
        for (size_t i = 0; i < 3; i++) {
            // If any dim does not overlap, the AABB volumes do not intersect
            if (!overlapInDim(other, i)) {
                return false;
            }
        }
        return true;
    }

    /// @brief Checks whether two AABB overlap
    /// @param other The second AABB object
    /// @param[out] p1 Returns min corner of overlap region
    /// @param[out] p2 Returns max corner of overlap region
    /// @return Returns true if the AABB overlap, false otherwise
    bool overlap(const AABB& other, double p1[3], double p2[3]) {
        bool out = true;
        for (size_t i = 0; i < 3; i++) {
            // If any dim does not overlap, the AABB volumes do not intersect
            out &= overlapInDim(other, i, p1, p2);
        }
        return out;
    }

    /// @brief Returns the distance between two AABB
    /// @param other The second AABB object
    /// @return Distance between the two AABB, returns 0 when overlapping
    inline double distance(const AABB& other) {
        double dist = 0, d;
        for (size_t i = 0; i < 3; i++) {
            d = distInDim(other, i);
            dist += d * d;
        }
        return std::sqrt(dist);
    }

    /// @brief Returns the distance and witness points between two AABB
    /// @param other The second AABB object
    /// @param[out] p1 Nearest point on the first AABB object
    /// @param[out] p2 Nearest point on the second AABB object
    /// @return Distance between the two AABB, returns 0 when overlapping
    inline double distance(const AABB& other, double* p1, double* p2) {
        double dist = 0, d;
        for (size_t i = 0; i < 3; i++) {
            d = distInDim(other, i);
            if (d != 0) {
                p1[i] = (d > 0) ? worldMax(i) : worldMin(i);
                p2[i] = (d > 0) ? other.worldMin(i) : other.worldMax(i);
            } else {
                p1[i] = (std::max)(worldMin(i), other.worldMin(i)); // parenthesis surrounding std::min/max to prevent macro replacement)
                p1[i] += ((std::min)(worldMax(i), other.worldMax(i)) - p1[i]) * 0.5;
                p2[i] = p1[i];
            }
            dist += d * d;
        }
        return std::sqrt(dist);
    }

  protected:
    // Returns whether two cuboids overlap in given dimension
    inline bool overlapInDim(const AABB& other, size_t dim) {
        return !(worldMax(dim) < other.worldMin(dim) || worldMin(dim) > other.worldMax(dim));
    }

    // Returns min/max corner of overlapping cuboid region in given dimension
    inline bool overlapInDim(const AABB& other, size_t dim, double p1[], double p2[]) {
        bool overlaps = !(worldMax(dim) < other.worldMin(dim) || worldMin(dim) > other.worldMax(dim));
        if (overlaps)
        {
            p1[dim] = (std::max)(worldMin(dim), other.worldMin(dim));
            p2[dim] = (std::min)(worldMax(dim), other.worldMax(dim));
        }
        return overlaps;
    }

    inline double worldMin(size_t dim) const {
        return m_localOrigin[dim] + m_localMin[dim];
    }

    inline double worldMax(size_t dim) const {
        return m_localOrigin[dim] + m_localMax[dim];
    }

    inline double distInDim(const AABB& other, size_t i) const {
        double d = other.worldMin(i) - worldMax(i);
        if (d > 0) {
            return d;
        } else {
            d = other.worldMax(i) - worldMin(i);
            return (d < 0) ? d : 0;
        }
    }

    inline std::vector<double> shiftToWorld(const std::vector<double>& in) const {
        std::vector<double> world(in);
        for (size_t i = 0; i < 3; i++) {
            world[i] += m_localOrigin[i];
        }
        return world;
    }
};
} // namespace nav
#endif /* AABB_CODEGEN_HPP */
