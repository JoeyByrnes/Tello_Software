// Copyright 2019-2021 The MathWorks, Inc.

/**
* @file
* @brief Utilities to find nearest neighbor in the tree for a given state
*/


#ifndef PLANNINGCODEGEN_NEARESTNEIGHBOR_HPP
#define PLANNINGCODEGEN_NEARESTNEIGHBOR_HPP

#include <vector>
#include <queue>
#include <algorithm>
#include <utility>        // for std::pair
#include "planningcodegen_TreeNode.hpp"
#include "planningcodegen_DistanceMetric.hpp"
#include "planningcodegen_CommonCSMetric.hpp"

namespace nav
{
    template <typename T>
    class DistanceRadiusFunctor {

public:
    DistanceRadiusFunctor() {}

    bool operator()(const std::pair<T, TreeNode<T>*> &a, T val) const {
        return a.first < val;
    }
};

template <typename T>
    class AscendingNodeFunctor {

public:
    AscendingNodeFunctor() {}

    bool operator()(const std::pair<T, TreeNode<T>*> &a,
                const std::pair<T, TreeNode<T>*> &b) {
        return a.first < b.first;
    }
};
    
    /// template class for nearest neighbor finding algorithms
    /**
     * @tparam T               Data type
     */
    template <typename T>
    class NearestNeighborFinder
    {
        // tree is a friend
        template <typename>
        friend class SearchTree;

    public:
        /// NearestNeighborFinder constructor
        NearestNeighborFinder(std::size_t nodeDim)
        {
            m_buildMetric = new CommonCSMetric<T>(nodeDim);
            m_queryMetric = new CommonCSMetric<T>(nodeDim);
        }

        virtual ~NearestNeighborFinder() 
        {
            purgeMetrics();
        }

        void purgeMetrics()
        {
            if (m_queryMetric != nullptr)
            {
                delete m_queryMetric;
                m_queryMetric = nullptr;
            }
            
            if (m_buildMetric != nullptr)
            {
                delete m_buildMetric;
                m_buildMetric = nullptr;
            }
        }

        void setBuildMetric(DistanceMetric<T> * bm)
        {
            m_buildMetric = bm;
        }

        void setQueryMetric(DistanceMetric<T> * qm)
        {
            m_queryMetric = qm;
        }

        virtual void insert(TreeNode<T>* nodePtr) = 0;
        virtual TreeNode<T>* nearestNeighbor(const std::vector<T>& state, T &distNN) = 0;
        virtual std::vector<TreeNode<T>*> nearK(const std::vector<T>& state, std::size_t num) = 0;
        virtual std::vector<TreeNode<T>*> near(const std::vector<T>& state, T dist) = 0;

        DistanceMetric<T> * getBuildMetric() const
        {
            return m_buildMetric;
        }
        
        DistanceMetric<T> * getQueryMetric() const
        {
            return m_queryMetric;
        }

    protected:
        DistanceMetric<T>* m_buildMetric;
        DistanceMetric<T>* m_queryMetric;
    };

    template <typename T>
    class ExhaustiveNN : public NearestNeighborFinder<T>
    {
    public:

        ExhaustiveNN(std::size_t dim) : NearestNeighborFinder<T>(dim) { /*ExhaustiveNN constructor*/ }
        void insert(TreeNode<T>* nodePtr) override
        {
            m_nodePtrs.push_back(nodePtr);
            m_currentNodeStates.insert(m_currentNodeStates.end(), nodePtr->getState().begin(),
                nodePtr->getState().end());
        }

        struct NaNDistanceComparator
        {
            bool operator()(const T& lhs, const T& rhs)
            {
                // any finite, infinite, or NaN distance is less than NaN
                return (lhs < rhs) || std::isnan(rhs);
            }
        };

        /// return the nearest node (as node pointer) to a given state
        TreeNode<T>* nearestNeighbor(const std::vector<T>& state, T &distNN) override
        {
            std::vector<T> dists = this->m_queryMetric->distance(m_currentNodeStates, state);
            auto minIter = std::min_element(dists.begin(), dists.end(), NaNDistanceComparator());
            
            distNN = *minIter;
            auto nodePtrIter = m_nodePtrs.begin() + std::distance(dists.begin(), minIter);
            return *nodePtrIter;
        }

        /// return the nearest node (as node pointer) to a given state, alternative implementation
        TreeNode<T>* nearestNeighbor_alternative(const std::vector<T>& state, T &distNN)
        {
            T d = static_cast<T>(0.0);
            T dNN = std::numeric_limits<T>::max();
            TreeNode<T>* out = nullptr;
            for (auto&& nodePtr : m_nodePtrs)
            {
                std::vector<T> dists = this->m_queryMetric->distance(nodePtr->getState(), state);
                d = dists[0];
                if (d < dNN)
                {
                    dNN = d;
                    out = nodePtr;
                }
            }
            distNN = dNN;
            return out;
        }


        /// return the nearest K nodes (as node pointers) around the given state
        std::vector<TreeNode<T>*> nearK(const std::vector<T>& state, std::size_t num) override
        {
            std::vector<TreeNode<T>*> out;
            
            //TODO
            auto comparator = [this, &state](TreeNode<T> * node1, TreeNode<T> * node2)->bool
            {
                std::vector<T> d1 = this->m_queryMetric->distance(state, node1->getState());
                std::vector<T> d2 = this->m_queryMetric->distance(state, node2->getState());
                return d1[0] > d2[0];
            };

            std::priority_queue<TreeNode<T> *, std::vector<TreeNode<T> *>, decltype(comparator) > pq(comparator);
            for (auto&& nodePtr : m_nodePtrs)
            {
                pq.push(nodePtr);
            }

            for (std::size_t k = 0; k < std::min(num, m_nodePtrs.size()); k++)
            {
                out.push_back(pq.top());
                pq.pop();
            }
            return out;
        }

        /// return the list of nodes (as node pointers) within a radius of a given state
        std::vector<TreeNode<T>*> near(const std::vector<T>& state, T radius) override
        {
            // compute distance to all the existing nodes
            std::vector<T> dists = this->m_queryMetric->distance(m_currentNodeStates, state );
            std::size_t count = 0;
            std::vector<std::pair<T, TreeNode<T>*>> distPairs{};

            // convert to vector of (distance, node_pointer) pairs
            
            //TODO
            std::transform(dists.begin(), dists.end(), std::back_inserter(distPairs),
                [this, &count](T d) -> std::pair<T, TreeNode<T>*> { 
                count++;
                return std::make_pair(d, this->m_nodePtrs[count - 1]); });

            // sort vector of pairs by distance (ascending)
            std::sort(distPairs.begin(), distPairs.end(), nav::AscendingNodeFunctor<T>());

            // retrieve those with distance smaller than radius
            auto lbIt = std::lower_bound(distPairs.begin(), distPairs.end(), radius,
                nav::DistanceRadiusFunctor<T>());

            std::vector<TreeNode<T>*> out;
            for (auto it = distPairs.begin(); it < lbIt; it++)
            {
                out.push_back((*it).second);
            }
            return out;
        }

        /// return the list of nodes (as node pointers) within a radius of a given state (alternative implementation)
        std::vector<TreeNode<T>*> near_alternative(const std::vector<T>& state, T radius)
        {
            std::vector<TreeNode<T>*> out;
            T defaultDist = static_cast<T>(-1);
            //TODO
            auto comparator = [this, &state](std::pair<TreeNode<T>*, T>& pair1, std::pair<TreeNode<T>*, T>& pair2)->bool
            {
                if (pair1.second < 0) // compute distance if it has not been done yet
                {
                    std::vector<T> d1 = this->m_queryMetric->distance(pair1.first->getState(), state);
                    pair1.second = d1[0];
                }
                if (pair2.second < 0)
                {
                    std::vector<T>  d2 = this->m_queryMetric->distance(pair2.first->getState(), state);
                    pair2.second = d2[0];
                }
                return pair1.second > pair2.second;
            };

            // each pair contains the tree node pointer and the node's distance to given state
            std::priority_queue<std::pair<TreeNode<T>*, T>, std::vector<std::pair<TreeNode<T>*, T>>, decltype(comparator) > pq(comparator);
            for (auto&& nodePtr : m_nodePtrs)
            {
                pq.push(std::make_pair(nodePtr, defaultDist) );
            }

            while (!pq.empty() && pq.top().second < radius)
            {
                out.push_back(pq.top().first);
                pq.pop();
            }
            return out;
        }

        /// find the ID of the nearest neighbor node, used for test only
        std::size_t nearestNeighborIdx(const std::vector<T>& state, T& distNN)
        {
            T d{};
            T dNN = std::numeric_limits<T>::max();
            std::size_t idxNN = 0;
            for (std::size_t i = 0; i < m_nodePtrs.size(); i++)
            {
                std::vector<T> dists = this->m_queryMetric->distance(m_nodePtrs[i]->getState(), state);
                d = dists[0];
                if (d < dNN)
                {
                    dNN = d;
                    idxNN = i;
                }
            }
            distNN = dNN;
            return idxNN;
        }


        /// find the IDs of the near K neighboring nodes for a given state, used for test only
        std::vector<std::size_t> nearKIdx(const std::vector<T>& state, const std::size_t& num)
        {
            std::vector<std::size_t> indices{};

            //TODO
            auto comparator = [this, &state](std::size_t id1, std::size_t id2)->bool
            {
                auto node1 = this->m_nodePtrs[id1];
                auto node2 = this->m_nodePtrs[id2];
                std::vector<T> d1 = this->m_queryMetric->distance(state, node1->getState());
                std::vector<T> d2 = this->m_queryMetric->distance(state, node2->getState());
                return d1[0] > d2[0];
            };

            std::priority_queue<std::size_t, std::vector<std::size_t>, decltype(comparator) > pq(comparator);
            for (std::size_t i = 0; i < m_nodePtrs.size(); i++)
            {
                pq.push(i);
            }
            for (std::size_t k = 0; k < std::min(num, m_nodePtrs.size()); k++)
            {
                indices.push_back(pq.top());
                pq.pop();
            }
            return indices;
        }


    protected:
        /// list of node pointers to facilitate nearest neighbor search
        std::vector<TreeNode<T>*> m_nodePtrs;

        /// states from all the nodes known by NN flattened into a 1-d vector
        std::vector<T> m_currentNodeStates;

    };
}


#endif
