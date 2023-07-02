// Copyright 2019-2021 The MathWorks, Inc.

/**
 * @file
 * @brief Metric used for common configuration space (CS) in motion planing
 */

#ifndef PLANNINGCODEGEN_COMMONCSMETRIC_HPP
#define PLANNINGCODEGEN_COMMONCSMETRIC_HPP

#include <cassert>
#include "planningcodegen_DistanceMetric.hpp"

namespace nav {
/// template class for common configuration space metrics
/**
 * @tparam T    Data type
 *
 */
template <typename T>
class CommonCSMetric : public DistanceMetric<T> {
  private:
    const double M_PI_ = 3.141592653589793238462643383;
  public:
    enum Topology { EUCLIDEAN = 0, CIRCLE, REALPROJECTIVE };

    CommonCSMetric(std::size_t dim) {
        this->m_dim = dim;
        m_topologies.assign(this->m_dim, Topology::EUCLIDEAN);
        m_weights.assign(this->m_dim, static_cast<T>(1.0));
    }

    CommonCSMetric(std::size_t dim,
                   const std::vector<int32_T>& topologies,
                   const std::vector<T>& weights) {
        this->m_dim = dim;
        assert(topologies.size() == this->m_dim);
        assert(weights.size() == this->m_dim);
        m_topologies = topologies;
        m_weights = weights;
    }

    void configure(const std::vector<int32_T>& topologies, const std::vector<T>& weights) {
        assert(topologies.size() == this->m_dim);
        assert(weights.size() == this->m_dim);
        m_topologies = topologies;
        m_weights = weights;
    }

    /// compute distances from a query state to a collection of states (1 or more)
    std::vector<T> distance(const std::vector<T>& states, const std::vector<T>& queryState) {
        typename std::vector<T>::const_iterator queryIt = queryState.begin();
        typename std::vector<T>::const_iterator statesIt = states.begin();

        std::size_t numStates = states.size() / this->m_dim;
        std::vector<T> output;
        for (std::size_t j = 0; j < numStates; j++) {
            output.push_back(distanceInternal(queryIt, statesIt));
            statesIt += this->m_dim;
        }
        return output;
    }


  protected:
    std::vector<int32_T> m_topologies;
    std::vector<T> m_weights;

    /// distance between two states as the Cartesian product of the three metric spaces
    T distanceInternal(const typename std::vector<T>::const_iterator& state1It,
                       const typename std::vector<T>::const_iterator& state2It) {
        T result = static_cast<T>(0.0);
        T dist = static_cast<T>(0.0);
        T t = static_cast<T>(0.0);
        T dotProd = static_cast<T>(0.0);
        for (std::size_t k = 0; k < this->m_dim; k++) {
            switch (m_topologies[k]) {
            case CIRCLE: /*S1 circle, i.e. 2D rotation*/
                t = std::fabs(wrapToPi(*(state2It + k)) - wrapToPi(*(state1It + k)));
                dist = std::fmin(t, 2 * M_PI_ - t);
                break;

            case REALPROJECTIVE: /* quaternion, [k, k+3] */
                dotProd = unitQuatDotProduct(state1It, state2It, k);
                dist = std::fmin(std::acos(dotProd),
                                 std::acos(-dotProd)); // angle between two unit quaternions
                k += 3; // RP3 (quaternion) always has 4 (3+1) elements, another k addition will
                        // happen as part of the for-loop
                break;

            case EUCLIDEAN: /*Euclidean 1-space*/
            default:
                dist = *(state2It + k) - *(state1It + k);
            }
            result += m_weights[k] * std::pow(dist, 2.0);
        }


        return std::sqrt(result);
    }


    /// dot product of two unit quaternions
    T unitQuatDotProduct(const typename std::vector<T>::const_iterator& state1It,
                         const typename std::vector<T>::const_iterator& state2It,
                         std::size_t idxStart) {
        T dotProd = static_cast<T>(0.0);
        T norm1 = static_cast<T>(0.0);
        T norm2 = static_cast<T>(0.0);
        for (std::size_t k = idxStart; k < idxStart + 4; k++) {
            norm1 += (*(state1It + k)) * (*(state1It + k));
            norm2 += (*(state2It + k)) * (*(state2It + k));
            dotProd += (*(state1It + k)) * (*(state2It + k));
        }
        return dotProd / (norm1 * norm2);
    }

    /// wrap to pi
    T wrapToPi(T inputRadians) {
        T x = std::fmod(inputRadians + M_PI_, 2 * M_PI_);
        if (!std::signbit(x) /* true if negative*/ &&
            (x == 0)) // positive multiples of 2 * pi map to 2 * pi
        {
            x = 2 * M_PI_;
        }
        return x - M_PI_;
    }
};


} // namespace nav

#endif
