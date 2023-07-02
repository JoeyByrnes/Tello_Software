// Copyright 2019-2021 The MathWorks, Inc.

/**
* @file
* @brief Distance metric between two states
*/


#ifndef PLANNINGCODEGEN_DISTANCEMETRIC_HPP
#define PLANNINGCODEGEN_DISTANCEMETRIC_HPP

#include <vector>
#include <iostream>
#include <cmath>

namespace nav
{
    template <typename T>
    class DistanceMetric
    {
    public:
        DistanceMetric() {}
        virtual ~DistanceMetric() {}

        virtual std::vector<T> distance(const std::vector<T>& treeStates, const std::vector<T>& queryState) = 0;
        
    protected:
        /// convert row major matrix to column major matrix
        static std::vector<real64_T> rowMajorToColumnMajor(const std::vector<real64_T>& states, std::size_t dim) {
            std::size_t numStates = states.size() / dim;
            std::vector<real64_T> statesReordered(states.size(), 0.0);
            for (std::size_t i = 0; i < dim; i++) {
                for (std::size_t j = 0; j < numStates; j++) {
                    statesReordered[j + i * numStates] = states[i + j * dim];
                }
            }
            return statesReordered;
        }

    protected:
        /// dimension of the state
        std::size_t m_dim;

        /// direction for computing distance
        /**
         * from treeStates to queryState [false, outward] or the other way around [true, inward]
         */
        boolean_T m_reverseDirection;

    };

    template <typename T>
    class EuclideanMetric : public DistanceMetric<T>
    {
    public:
        EuclideanMetric(std::size_t dim)
        {
            this->m_dim = dim;
        }

        std::vector<T> distance(const std::vector<T>& states, const std::vector<T>& queryState)
        {
            std::vector<T> dists;
            
            std::size_t numStates = states.size() / this->m_dim;
            typename std::vector<T>::const_iterator queryIt = queryState.begin();
            typename std::vector<T>::const_iterator statesIt = states.begin();

            for (std::size_t i = 0; i < numStates; i++)
            {
                dists.push_back(distanceInternal(statesIt, queryIt));
                statesIt += this->m_dim;
            }
            return dists;
        }
    protected:
        T distanceInternal(const typename std::vector<T>::const_iterator& state1It, const typename std::vector<T>::const_iterator& state2It)
        {
            T sum = static_cast<T>(0.0);
            for (size_t i = 0; i < this->m_dim; i++)
            {
                sum += ( *(state2It + i) - *(state1It + i)) * ( *(state2It + i) - *(state1It + i));
            }
            return std::sqrt(sum);
        }
    };
}

#endif
