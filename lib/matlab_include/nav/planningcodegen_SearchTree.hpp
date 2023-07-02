// Copyright 2019-2021 The MathWorks, Inc.

/**
 * @file
 * @brief Search tree data structure to support RRT-like planning algorithms
 */


#ifndef PLANNINGCODEGEN_SEARCHTREE_HPP
#define PLANNINGCODEGEN_SEARCHTREE_HPP

#include <vector>
#include <stack>
#include <deque>
#include <queue>
#include <iostream>
#include <string>
#include <cstddef>
#include <limits>
#include "planningcodegen_TreeNode.hpp"
#include "planningcodegen_DistanceMetric.hpp"
#include "planningcodegen_NearestNeighbor.hpp"


namespace nav {
/// template class that represents the underlying tree data structure for RRT-like planning
/// algorithms
/**
 * @tparam T               Data type
 */
template <typename T>
class SearchTree {
  public:
    template <typename Derived>
    class Iterator {
      public:
        Iterator(TreeNode<T>* ptr) {
            m_ptr = ptr;
        }

        /// copy constructor
        Iterator(const Iterator& it) {
            m_ptr = it.m_ptr;
        }

        virtual ~Iterator() {
        }

        virtual Derived& operator++(int32_T x) = 0;

        TreeNode<T>* operator->() {
            return m_ptr;
        }

        TreeNode<T>& operator*() {
            return *m_ptr;
        }

        TreeNode<T>*& operator()() {
            return m_ptr;
        }

        virtual boolean_T isAtEnd() = 0;

      protected:
        TreeNode<T>* m_ptr;
    };


    /// Breadth-first iterator
    class BFSIterator : public Iterator<BFSIterator> {
      public:
        BFSIterator(TreeNode<T>* ptr)
            : Iterator<BFSIterator>(ptr) {
            m_queue.push_front(ptr);
        }

        BFSIterator(const BFSIterator& bit)
            : Iterator<BFSIterator>(bit) {
            m_queue = bit.m_queue;
        }

        /// postfix ++ breadth-first search iterator
        BFSIterator& operator++(int32_T x) override {
            // Avoid unused parameter compiler warning
            (void)x;

            auto ptr = m_queue.front();
            m_queue.pop_front();
            for (auto& p : ptr->m_children) { //
                m_queue.push_back(p);
            }
            if (!m_queue.empty()) {
                this->m_ptr = m_queue.front();
            }
            return *this;
        }

        boolean_T isAtEnd() override {
            return m_queue.empty();
        }

      protected:
        std::deque<TreeNode<T>*> m_queue;
    };


    /// Depth-first iterator
    class DFSIterator : public Iterator<DFSIterator> {
      public:
        DFSIterator(TreeNode<T>* ptr)
            : Iterator<DFSIterator>(ptr) {
            m_stack.push(ptr);
        }

        DFSIterator(const DFSIterator& dit)
            : Iterator<DFSIterator>(dit) {
            m_stack = dit.m_stack;
        }

        /// postfix ++ depth-first search iterator
        DFSIterator& operator++(int32_T x) override {
            // Avoid unused parameter compiler warning
            (void)x;

            auto ptr = m_stack.top();
            m_stack.pop();
            for (auto& p : ptr->m_children) { //
                m_stack.push(p);
            }
            if (!m_stack.empty()) {
                this->m_ptr = m_stack.top();
            }
            return *this;
        }

        boolean_T isAtEnd() override {
            return m_stack.empty();
        }

      protected:
        std::stack<TreeNode<T>*> m_stack;
    };

  protected:
    std::size_t m_nodeDim;
    TreeNode<T>* m_root;
    NearestNeighborFinder<T>* m_nnFinder;
    std::vector<TreeNode<T>*> m_nodePtrs;
    T m_ballRadiusConstant;
    T m_maxConnectionDistance;

  public:
    SearchTree(const std::vector<T>& state) {
        m_nodeDim = state.size();

        m_ballRadiusConstant = static_cast<T>(1);
        m_maxConnectionDistance = static_cast<T>(0.1);

        m_root = new TreeNode<T>(m_nodeDim);
        m_root->setState(state);

        m_nnFinder = new ExhaustiveNN<T>(m_nodeDim);

        m_nnFinder->insert(m_root);
        m_nodePtrs.push_back(m_root);
        m_root->setNodeID(0);
    }

    /// destructor
    ~SearchTree() {
        std::stack<TreeNode<T>*> stackOfNodePtrs;

        for (BFSIterator it(getRoot()); !it.isAtEnd(); it++) {
            stackOfNodePtrs.push(it()); // i.e. &*it
        }

        while (!stackOfNodePtrs.empty()) {
            delete stackOfNodePtrs.top();
            stackOfNodePtrs.pop();
        }
        delete m_nnFinder;
    }

    void setBallRadiusConstant(T rc) {
        m_ballRadiusConstant = rc;
    }

    void setMaxConnectionDistance(T dist) {
        m_maxConnectionDistance = dist;
    }

    T getBallRadiusConstant() {
        return m_ballRadiusConstant;
    }

    T getMaxConnectionDistance() {
        return m_maxConnectionDistance;
    }

    TreeNode<T>* getRoot() {
        return m_root;
    }

    TreeNode<T>* getNode(std::size_t idx) {
        if (idx < m_nodePtrs.size()) {
            return m_nodePtrs[idx];
        } else {
            return nullptr;
        }
    }

    std::size_t getNodeDim() {
        return m_nodeDim;
    }

    std::size_t getNumNodes() {
        return m_nodePtrs.size();
    }

    NearestNeighborFinder<T>* getNNFinder() {
        return m_nnFinder;
    }

    TreeNode<T>* insertNode(TreeNode<T>* parent, const std::vector<T>& state) {
        std::vector<T> dist = m_nnFinder->m_queryMetric->distance(
            parent->getState(), state); // compute distance first, if it throws exception, the rest
                                        // will not be executed.
        T costFromParent = dist[0];

        auto pNode = new TreeNode<T>(m_nodeDim);
        parent->addToChildren(pNode);
        pNode->setState(state);

        m_nodePtrs.push_back(pNode);

        pNode->setNodeID(m_nodePtrs.size() - 1);
        m_nnFinder->insert(pNode);

        pNode->m_costFromParent = costFromParent;
        pNode->m_costFromRoot = parent->m_costFromRoot + pNode->m_costFromParent;

        return pNode;
    }

    boolean_T insertNodeByID(std::size_t idx, const std::vector<T>& state, std::size_t& idxNew) {
        idxNew = 0;
        if (idx < m_nodePtrs.size()) {
            auto parent = m_nodePtrs[idx];

            if (state.size() != m_nodeDim) { // mismatched state size
                return false;
            }

            std::vector<T> dist = m_nnFinder->m_queryMetric->distance(
                parent->getState(), state); // compute distance first, if it throws exception, the
                                            // rest will not be executed.
            T costFromParent = dist[0];

            auto pNode = new TreeNode<T>(m_nodeDim);

            parent->addToChildren(pNode);
            pNode->setState(state);

            m_nodePtrs.push_back(pNode);

            pNode->setNodeID(m_nodePtrs.size() - 1);
            m_nnFinder->insert(pNode);

            pNode->m_costFromParent = costFromParent;
            pNode->m_costFromRoot = parent->m_costFromRoot + pNode->m_costFromParent;

            idxNew = pNode->getNodeID();
            return true;
        } else {
            return false;
        }
    }

    boolean_T insertNodeByIDWithPrecomputedCost(std::size_t idx,
                                                const std::vector<T>& state,
                                                T precomputedCost,
                                                std::size_t& idxNew) {
        idxNew = 0;
        if (idx < m_nodePtrs.size()) {
            auto parent = m_nodePtrs[idx];
            T costFromParent = precomputedCost;

            auto pNode = new TreeNode<T>(m_nodeDim);

            parent->addToChildren(pNode);
            pNode->setState(state);

            m_nodePtrs.push_back(pNode);

            pNode->setNodeID(m_nodePtrs.size() - 1);
            m_nnFinder->insert(pNode);

            pNode->m_costFromParent = costFromParent;
            pNode->m_costFromRoot = parent->m_costFromRoot + pNode->m_costFromParent;

            idxNew = pNode->getNodeID();
            return true;
        } else {
            return false;
        }
    }

    /// rewire one node and its descendants under a new parent node
    int32_T rewireNodeByID(std::size_t nodeID,
                           std::size_t newParentNodeID,
                           T distanceBetweenNodes = -99) {
        if (nodeID < m_nodePtrs.size() && newParentNodeID < m_nodePtrs.size()) {
            auto pNode = m_nodePtrs[nodeID];
            auto pNewParentNode = m_nodePtrs[newParentNodeID];

            // if node's parent is newParentNode, do nothing
            if (pNode->m_parent == pNewParentNode) {
                return 2;
            }

            // if node is newParentNode
            if (pNode == pNewParentNode) {
                return 3;
            }

            // if newParentNode is in fact a descendant of node, abort
            if (isDescendantOf(pNewParentNode, pNode)) {
                return 4;
            }

            // normal case: node's parent is not newParentNode, and newParentNode is not a
            // descendant of node.
            pNode->m_parent->removeChild(pNode);
            pNewParentNode->addToChildren(pNode);

            // propagate cost changes downstream
            T newDist = static_cast<T>(0.0);
            if (distanceBetweenNodes < 0) {
                newDist = computeDistanceBetweenNodes(pNewParentNode, pNode);
            } else {
                newDist = distanceBetweenNodes;
            }

            auto distDiff = pNewParentNode->m_costFromRoot + newDist - pNode->m_costFromRoot;
            pNode->m_costFromParent = newDist;
            for (BFSIterator it(pNode); !it.isAtEnd(); it++) {
                it->m_costFromRoot += distDiff;
            }
            return 0; // success
        } else {
            return 1; // node ID out of range
        }
    }

    /// check if node 1 is a descendant of node 2
    boolean_T isDescendantOf(TreeNode<T>* pNode1, TreeNode<T>* pNode2) {
        boolean_T result = false;
        TreeNode<T>* ptr = pNode1;
        if (ptr->m_parent == nullptr) { // if pNode1 is already the root
            return false;
        }

        while (ptr != nullptr) {
            if (ptr->m_parent == pNode2) {
                result = true;
                break;
            }
            ptr = ptr->m_parent;
        }
        return result;
    }

    /// trace back to root node from a given node id
    std::vector<T> tracebackToRoot(std::size_t idx) {
        std::vector<T> route;
        std::vector<T> currState;
        if (idx < m_nodePtrs.size()) {
            TreeNode<T>* currentNode = m_nodePtrs[idx];
            currState = currentNode->getState();
            route.insert(route.end(), currState.begin(), currState.end());
            while (currentNode->m_parent) {
                currentNode = currentNode->m_parent;
                currState = currentNode->getState();
                route.insert(route.end(), currState.begin(), currState.end());
            }
        }

        return route;
    }

    T computeDistanceBetweenNodes(TreeNode<T>* node1, TreeNode<T>* node2) {
        std::vector<T> dist =
            m_nnFinder->m_queryMetric->distance(node1->getState(), node2->getState());
        return dist[0];
    }


    std::size_t nearestNeighborID(const std::vector<T>& state, T& distNN) {
        TreeNode<T>* nodePtr = m_nnFinder->nearestNeighbor(state, distNN);
        return nodePtr->getNodeID();
    }

    std::vector<std::size_t> nearKNeighborIDs(const std::vector<T>& state, const std::size_t& num) {
        std::vector<std::size_t> indices;

        std::vector<TreeNode<T>*> nodePtrs = m_nnFinder->nearK(state, num);

        for (auto&& pn : nodePtrs) {
            indices.push_back(pn->getNodeID());
        }
        return indices;
    }

    /// Returns the IDs of the nodes within the closed ball of radius r centered at the given state.
    /// r is computed adaptively.
    std::vector<std::size_t> nearNeighborIDs(const std::vector<T>& state) {
        std::vector<std::size_t> indices;

        std::vector<TreeNode<T>*> nodePtrs = m_nnFinder->near(state, computeBallRadius());

        for (auto&& pn : nodePtrs) {
            indices.push_back(pn->getNodeID());
        }
        return indices;
    }

    /// compute ball radius
    T computeBallRadius() {
        std::size_t numNodes = m_nodePtrs.size();
        T d = std::pow(m_ballRadiusConstant * std::log(numNodes) / numNodes,
                       static_cast<T>(1.0) / m_nodeDim);

        T radius = std::fmin(d, m_maxConnectionDistance);
        return radius;
    }

    std::vector<T> inspect() {
        std::vector<T> output;
        std::vector<T> nanState(m_nodeDim, std::numeric_limits<T>::quiet_NaN());
        for (BFSIterator it(getRoot()); !it.isAtEnd(); it++) {
            if (it->m_parent) {
                auto p = it->m_parent->getState();
                output.insert(output.end(), p.begin(), p.end());
            }
            auto curr = it->getState();
            output.insert(output.end(), curr.begin(), curr.end());
            output.insert(output.end(), nanState.begin(), nanState.end());
        }
        return output;
    }
};

} // namespace nav

#endif
