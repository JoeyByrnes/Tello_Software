/* Copyright 2019-2021 The MathWorks, Inc. */

// PriorityQueueImpl Data structures to support A* algorithm and its variants

#ifndef PRIORITYQUEUE_INTERFACE_IMPL
#define PRIORITYQUEUE_INTERFACE_IMPL

#include <vector>
#include <queue>
#include <utility>
#include <unordered_map>
#include <assert.h>
#include <limits>

namespace nav {
    /// Node is an std::pair of ID and data vector)
    typedef std::pair<int, std::vector<double>> Node; // <id, data>

    /**
     * @brief This class is a simple wrapper of std::unordered_map<int, std::vector<double>>
     */
    class SimpleMap {
    private:
        /// Id --> Data
        std::unordered_map<int, std::vector<double>> IdDataMap;

        /// Size of the data vector
        const int DataDim;
    
    public:

        SimpleMap(int dataDim) : DataDim(dataDim) {
            assert(dataDim >= 0);
            IdDataMap = std::unordered_map<int, std::vector<double>>();
        }

        /// Associates data vector with the given id
        void insertData(int id, const double* data) {
            IdDataMap[id] = std::vector<double>(data, data + DataDim);
        }

        /// Extracts the data vector associated with the given id
        std::vector<double> getData(int id) {
            if (IdDataMap.find(id) == IdDataMap.end() ) {
                return std::vector<double>(DataDim, std::numeric_limits<double>::quiet_NaN());
            }
            return IdDataMap[id];
        }

        /// Returns the data dimension of the simpleMap
        int getDataDim() {
            return DataDim;
        }

        /// Gets the number of entries in the simpleMap
        size_t getSize() {
            return IdDataMap.size();
        }
    };

    /**
     * @brief This class represents the map between nodeIds and the map between a nodeId and its data
     */
    class NodeMap {

    private:
        /// Map: currenNodeId --> parentNodeId
        std::unordered_map<int, int> CameFrom;

        /// Map: currentNodeId --> nodeData
        std::unordered_map<int, std::vector<double>> NodeDataMap;
        
        /// Size of the data
        const int DataDim;

        /// Unique identifier
        int UniqueId;

    public:

        NodeMap(int dataDim) : DataDim(dataDim) {
            assert(dataDim >= 0);
            CameFrom = std::unordered_map<int, int>();
            NodeDataMap = std::unordered_map<int, std::vector<double>>();
            UniqueId = 0;
        }

        ///Adds a new node as the child of parentId node (i.e. append). The new nodeId is automatically assigned and returned.
        ///parentId may or may not already exist in the nodeMap.
        /**
         * @param[in] data Data of the new node
         * @param[in] parentId ID of the parent node
         * @return ID of the new node
         */
        int insertNode(const double* data, int parentId) {
            if (UniqueId == parentId) {
                UniqueId++;
            }
            int id = UniqueId++;
            CameFrom[id] = parentId;
            NodeDataMap[id] = std::vector<double>(data, data + DataDim);
            return id;
        }

        ///Extract the node data associated with the given nodeId
        /**
         * @param[in] nodeId Id of the node
         * @param[out] parentId Id of the parent node
         * @return data vector associated with the node
         */
        std::vector<double> getNodeData(int nodeId, int & parentId) {
            if (NodeDataMap.find(nodeId) == NodeDataMap.end() ) {
                parentId = nodeId; // in normal case, parentId should never be the same as nodeId
                return std::vector<double>(DataDim, std::numeric_limits<double>::quiet_NaN());
            }
            parentId = CameFrom[nodeId];
            return NodeDataMap[nodeId];
        }

        /// Trace back to root from the node of given ID
        /**
         * @return A vector of data from nodes along the path in reverse order (data of the root node last)
         */ 
        std::vector<std::vector<double>> traceBack(int idx) {
            std::vector<std::vector<double>> res;
            while (CameFrom.find(idx) != CameFrom.end()) {
                res.push_back(NodeDataMap[idx]);
                idx = CameFrom[idx];
            }
            if (res.empty()) { res.emplace_back(DataDim, std::numeric_limits<double>::quiet_NaN()); }
            return res;
        }

        /// Return the data dimension of the node map
        int getDataDim() {
            return DataDim;
        }

        /// Get the number of nodes in NodeMap
        size_t getNumNodes() {
            return NodeDataMap.size();
        }
    };

    /**
     * @brief This class represents a priority queue of "node" data structure (min heap assumed).
     * A node is a pair of nodeId (int) and nodeData (vector of double).
     * PriorityQueueImpl is a thin wraper around the std::priority_queue data structure.
     */
    class PriorityQueueImpl {
    private:
 
        /// DataDim: size of the data;
        const int DataDim;

        /// PrimeIndex: Index of the data element that determines the location in heap
        const int PrimeIndex;

        /// UniqueId
        int UniqueId;

        struct comparator {
            int idx = 0;
            bool operator() (const Node& n1, const Node& n2) const {
                if (n1.second[idx] != n2.second[idx]){
                    return n1.second[idx] > n2.second[idx];// we want a min heap
                } else {
                    return n1.first > n2.first;// if the scores are the same, the one inserted first takes priority
                }                              // the secondary comparison is NOT necessary, just for backward compatibility, but it's extra work
            }
            comparator(int primeIdx) : idx(primeIdx) {}
            comparator() {}
        };

        std::priority_queue<Node, std::vector<Node>, comparator> pq;

    public:
        /// Constructor for PriorityQueueImpl
        PriorityQueueImpl(int dataDim, int primeIdx)
            : DataDim(dataDim), PrimeIndex(primeIdx) {
            assert(DataDim > PrimeIndex);
            assert(PrimeIndex >= 0);
            UniqueId = 0;
            pq = std::priority_queue<Node, std::vector<Node>, comparator>(PrimeIndex);
        }

        /// Push the data node onto the priority queue
        /**
         * @return The unique ID of the node just pushed
         */
        int push(const double* data) {
            int idx = UniqueId;
            pq.emplace(idx, std::vector<double>(data, data + DataDim));
            UniqueId++;
            return idx;
        }

        /// Return the element with lowest cost
        /**
         * @return If the queue is empty, top() returns a dummy node {-1, [nan, ..., nan]}
         */
        Node top() {
            if (pq.empty()){
                return std::make_pair(-1, std::vector<double>(DataDim, std::numeric_limits<double>::quiet_NaN()));
            } else {
                return pq.top();
            }
        }

        /// Remove the lowest cost element from the priority queue
        void pop() {
            if (!pq.empty()) {
                pq.pop();
            }
        }

        /// Return the size of the priority queue
        int size() {
            return static_cast<int>(pq.size());
        }

        /// Check if the priority queue is empty or not.
        bool isEmpty() {
            return (pq.empty()) ? true : false;
        }

        /// Return the data dimension of the priority queue
        int getDataDim() {
            return DataDim;
        }
    };
} // namespace nav
#endif
