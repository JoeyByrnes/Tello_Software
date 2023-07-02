// Copyright 2019-2021 The MathWorks, Inc.

/**
 * @file
 * @brief Reeds-Shepp distance metric
 */

#ifndef PLANNINGCODEGEN_REEDSSHEPPMETRIC_HPP
#define PLANNINGCODEGEN_REEDSSHEPPMETRIC_HPP

#include "planningcodegen_DistanceMetric.hpp"
#ifdef IS_NOT_MATLAB_HOST
#include "autonomouscodegen_reeds_shepp_api.hpp"
#else
#include "autonomouscodegen_reeds_shepp_tbb_api.hpp"
#endif


namespace nav {
/// template class for Reeds-Shepp distance metric
/**
 * @tparam T    Data type
 *
 */
template <typename T>
class ReedsSheppMetric : public DistanceMetric<T> {
  public:
    /**
     * @brief Compute distance between two vehicle states using Reeds-Shepp path
     * @param[in] state1 [x1, y1, theta1] as an std::vector
     * @param[in] state2 [x2, y2, theta2] as an std::vector
     * @return The distance between state1 and state2
     */
    T distance(const std::vector<T>& state1, const std::vector<T>& state2) {
        T result = static_cast<T>(0.0);
        (void)state1;
        (void)state2;
        return result;
    }
};

/// Specialization for double precision data type.
template <>
class ReedsSheppMetric<real64_T> : public DistanceMetric<real64_T> {
  public:
    ReedsSheppMetric(real64_T turningRadius, real64_T reverseCost, boolean_T isReversed = false) {
        this->m_dim = 3;
        this->m_reverseDirection = isReversed;
        m_minTurningRadius = turningRadius;
        m_reverseCost = reverseCost;
    }



    /// Reeds-Shepp distance between two states
    // When invoked by a sampling-based planner during NN search:
    // A) if m_reverseDirection is false, the distance function computes RS distances from
    //    treeStates (N) to queryStates (1)
    // B) if m_reverseDirection is true, the distance function computes RS distances from
    //    queryStates (1) to treeStates (N)
    std::vector<real64_T> distance(const std::vector<real64_T>& treeStates,
                                   const std::vector<real64_T>& queryStates) override{

        const std::vector<real64_T>& states1 = this->m_reverseDirection ? queryStates : treeStates;
        const std::vector<real64_T>& states2 = this->m_reverseDirection ? treeStates : queryStates;

        // reorder just to be reordered again (convert from row-major order to column-major order)
        // See enhancement g2415010.
        std::vector<real64_T> states1Reordered = rowMajorToColumnMajor(states1, this->m_dim);
        std::vector<real64_T> states2Reordered = rowMajorToColumnMajor(states2, this->m_dim);

        std::size_t numStates1 = states1.size() / this->m_dim;
        std::size_t numStates2 = states2.size() / this->m_dim;
        std::vector<real64_T> dists(numStates1 * numStates2, 0.0);

// unfortunately, this method below is designed to work with mxArray from MATLAB ONLY, which assumes
// column-major matrices We have to reorder "states" before we send in the data, which requires an
// copying overhead. better way is to update the TBB functor to provide a row-major API.
#ifdef IS_NOT_MATLAB_HOST
        autonomousReedsSheppDistanceCodegen_real64(
            states1Reordered.data(), static_cast<uint32_T>(numStates1), states2Reordered.data(),
            static_cast<uint32_T>(numStates2), m_minTurningRadius, m_reverseCost, dists.data());
#else
        autonomousReedsSheppDistanceCodegen_tbb_real64(
            states1Reordered.data(), static_cast<uint32_T>(numStates1), states2Reordered.data(),
            static_cast<uint32_T>(numStates2), m_minTurningRadius, m_reverseCost, dists.data());
#endif
        return dists;
    }

  protected:
    /// minimum turning radius
    real64_T m_minTurningRadius;

    /// reverse cost
    real64_T m_reverseCost;
};


} // namespace nav

#endif
