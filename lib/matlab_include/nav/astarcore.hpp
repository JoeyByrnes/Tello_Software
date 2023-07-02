/**
 * @file astarcore.hpp
 * @brief This file contains the declarations for AStarClass
 */

/* Copyright 2022 The MathWorks, Inc. */

#ifndef ASTARCORE_HPP_
#define ASTARCORE_HPP_

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <cmath>

#ifdef BUILDING_LIBMWASTARCODEGEN
#include "astarcodegen/astarcore_codegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "astarcore_codegen_util.hpp"
#endif


namespace nav{      

/**
 * @brief AStarCore implementation
 * 
 */
class AStarCore{
    
    private:        
        
        typedef uint32_T nodeID;
        nodeID _start {0}; // Start nodeID        
        nodeID _goal {0}; // Goal nodeID        
        nodeID _current{0}; // Current nodeID from the priority queue

        // Priority queue to store [fCost, nodeID]
        //  fCost = gCost + hCost where,    
        //  gCost is the actual cost from start to current nodeID
        //  hCost is the heuristic from current to goal nodeID
        //  fCost is used to specify the priority in the priority queue        
        typedef std::pair<real64_T, nodeID> PQPair;        
        std::priority_queue <PQPair, std::vector<PQPair>,std::greater<PQPair>> _openSet;  

        // Unordered_map to represent mapping between nodeID & parentID
        //  cameFrom[nodeID] = parentID
        //  Used for path re-construction after the search is completed        
        std::unordered_map <nodeID, nodeID> _cameFrom;

        // Unordered map to represent the mapping between nodeID and [gCost, closed]              
        //  where gCost of the cost of path from start nodeID to current nodeID
        //  closed=0 when node is in OpenSet, closed=1 when the node is in ClosedSet
        std::unordered_map <nodeID, std::pair<real64_T, boolean_T>> _nodeData;
        
        boolean_T _goalReached{false};  // Flag to know whether goal is reached


    // Public methods
    public:

        /**
         * @brief Set the Start nodeID
         * 
         * @param start Start nodeID
         */
        void setStart(const nodeID& start);

        /**
         * @brief Set the Goal nodeID
         * 
         * @param goal Goal nodeID
         */
        void setGoal(const nodeID& goal);

        /**
         * @brief Get the current node from the top of priority queue
         * 
         * @return current nodeID 
         */
        nodeID getCurrentNode();


        /**
         * @brief Loop through neighbors and update the openSet, gScore etc.
         * 
         * @param neighbors nodeIDs of the neighbors to the current nodeID
         * @param transitionCosts between current nodeID and the neighbor nodeIDs
         * @param heuristicCosts  between neighbor IDs and goal
         */
        void loopThroughNeighbors(const std::vector<nodeID>& neighbors,
                                const std::vector<real64_T>& transitionCosts,
                                const std::vector<real64_T>& heuristicCosts);

        /**
        * @brief Check if the node is in the closedSet
        * 
        * @param node nodeID input
        * @return true 
        * @return false 
        */
        boolean_T inClosedSet(const nodeID& node);

        /**
         * @brief Get the path output from the search
         * 
         * @return path vector of nodeIDs from start to goal
         */
        std::vector<nodeID> getPath();

        /**
         * @brief Get the explored nodes during the A* search
         * 
         * @return vector containing the explored nodeIDs
         */
        std::vector<nodeID> getExploredNodes();

        /**
         * @brief Get the path cost
         * 
         * @return path cost value
         */
        real64_T getPathCost();

        /**
         * @brief Get stop condition
         * 
         * @return true 
         * @return false 
         */
        boolean_T stopCondition() const;
    };
} // namespace nav


#endif
