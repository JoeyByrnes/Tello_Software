/**
 * @file astarcore_api.hpp
 * @brief This file contains declarations of external C-API interfaces for AStarCore.
 */

/* Copyright 2019-2022 The MathWorks, Inc. */

#ifndef ASTARCORE_CODEGEN_API_HPP
#define ASTARCORE_CODEGEN_API_HPP

#ifdef BUILDING_LIBMWASTARCODEGEN
#include "astarcodegen/astarcore_codegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "astarcore_codegen_util.hpp"
#endif


/**
 * @brief AStarCore constructor
 * 
 */
EXTERN_C ASTARCORE_CODEGEN_API void* astarcore_construct();


/**
 * @brief AStarCore destructor
 * 
 * @param[in] astarObj AStarCore object 
 */
EXTERN_C ASTARCORE_CODEGEN_API void astarcore_destruct(void *astarObj);


/**
 * @brief Set start nodeID
 * 
 * @param[in] astarObj AStarCore object
 * @param[in] start Start nodeID
 */
EXTERN_C ASTARCORE_CODEGEN_API void astarcore_setStart(void *astarObj, const uint32_T start);


/**
 * @brief Set the Goal nodeID
 * 
 * @param[in] astarObj AStarCore object 
 * @param[in] goal Goal nodeID
 */
EXTERN_C ASTARCORE_CODEGEN_API void astarcore_setGoal(void *astarObj, const uint32_T goal);


/**
 * @brief Get the current node from the top of priority queue
 * 
 * @param[in] astarObj AStarCore object 
 * @return current Current nodeID  
 */
EXTERN_C ASTARCORE_CODEGEN_API uint32_T astarcore_getCurrentNode(void *astarObj);


/**
 * @brief Loop through neighbors and update the openSet, gScore etc.
 * 
 * @param[in] astarObj AStarCore object 
 * @param[in] nSize Number of neighbor nodes for the current node
 * @param[in] neighbors NodeIDs of the neighbors to the current nodeID
 * @param[in] transitionCosts Transition costs between current nodeID and the neighbor nodeIDs 
 * @param[in] heuristicCosts Heuristic costs between neighbor IDs and goal 
 */
EXTERN_C ASTARCORE_CODEGEN_API void astarcore_loopThroughNeighbors(void *astarObj, const uint32_T nSize,
                                                                  const uint32_T* neighbors,
                                                                  const real64_T* transitionCosts, 
                                                                  const real64_T* heuristicCosts);


/**
 * @brief Get the path output from the search
 * 
 * @param[in] astarObj AStarCore object 
 * @param[out] path Path vector of nodeIDs from start to goal 
 */
EXTERN_C ASTARCORE_CODEGEN_API void astarcore_getPath(void* astarObj, real64_T* path);


/**
 * @brief Get the explored nodes during the A* search
 * 
 * @param[in] astarObj AStarCore object 
 * @param[out] exploredNodes Vector containing the explored nodeIDs 
 */
EXTERN_C ASTARCORE_CODEGEN_API void astarcore_getExploredNodes(void* astarObj, real64_T* exploredNodes);


/**
 * @brief Get the size of the path during the A* search
 * 
 * @param[in] astarObj AStarCore object 
 * @return pathSize Scalar containing the number of nodes on the path 
 */
EXTERN_C ASTARCORE_CODEGEN_API uint32_T astarcore_getPathSize(void* astarObj);


/**
 * @brief Get the number of explored nodes during the A* search
 * 
 * @param[in] astarObj AStarCore object 
 * @return numExploredNodes Scalar containing the number of explored nodeIDs 
 */
EXTERN_C ASTARCORE_CODEGEN_API uint32_T astarcore_getNumExploredNodes(void* astarObj);


/**
 * @brief Get the path cost
 * 
 * @param[in] astarObj AStarCore object 
 * @return patCost Path cost value 
 */
EXTERN_C ASTARCORE_CODEGEN_API real64_T astarcore_getPathCost(void* astarObj);


/**
 * @brief Get stop condition
 * 
 * @param[in] astarObj AStarCore object 
 * @return stopCondition True or False 
 */
EXTERN_C ASTARCORE_CODEGEN_API boolean_T astarcore_stopCondition(void* astarObj);

#endif




