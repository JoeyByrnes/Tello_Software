// Copyright 2019-2021 The MathWorks, Inc.

/**
 * @file
 * @brief Helper functions used only during codegen (C wrappers for nav::searchTree)
 */


#ifndef PLANNINGCODEGEN_CGHELPERS_HPP
#define PLANNINGCODEGEN_CGHELPERS_HPP

#include "planningcodegen_utils.hpp"

// namespace is NOT supported

EXTERN_C PLANNINGCODEGEN_API void* planningcodegen_createTree(real64_T* state, real64_T sz);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_destructTree(void* tree);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_configureCommonCSMetric(
    void* tree,
    const real64_T* topologies,
    const real64_T* weight,
    real64_T size);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_configureDubinsMetric(void* tree,
                                                                        real64_T turningRadius, 
                                                                        boolean_T isReversed);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_configureReedsSheppMetric(void* tree,
                                                                            real64_T turningRadius,
                                                                            real64_T reverseCost, 
                                                                            boolean_T isReversed);

EXTERN_C PLANNINGCODEGEN_API real64_T planningcodegen_getNumNodes(void* tree);

EXTERN_C PLANNINGCODEGEN_API boolean_T planningcodegen_insertNode(void* tree,
                                                                  real64_T* data,
                                                                  real64_T sz,
                                                                  real64_T parentId,
                                                                  real64_T* newIdx);

EXTERN_C PLANNINGCODEGEN_API boolean_T
planningcodegen_insertNodeWithPrecomputedCost(void* tree,
                                              real64_T* data,
                                              real64_T sz,
                                              real64_T precomputedCost,
                                              real64_T parentId,
                                              real64_T* newIdx);

EXTERN_C PLANNINGCODEGEN_API real64_T planningcodegen_nearestNeighbor(void* tree,
                                                                      real64_T* state,
                                                                      real64_T sz);

EXTERN_C PLANNINGCODEGEN_API boolean_T planningcodegen_getNodeState(void* tree,
                                                                    real64_T idx,
                                                                    real64_T* state);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_tracebackToRoot(void* tree,
                                                                  real64_T nodeId,
                                                                  real64_T* nodeStateSeq,
                                                                  real64_T* numNodes);

EXTERN_C PLANNINGCODEGEN_API real64_T planningcodegen_getNodeCostFromRoot(void* tree, real64_T idx);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_setBallRadiusConstant(void* tree, real64_T rc);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_setMaxConnectionDistance(void* tree,
                                                                           real64_T dist);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_near(void* tree,
                                                       real64_T* state,
                                                       real64_T* nearStateIds,
                                                       real64_T* numNearStates);

EXTERN_C PLANNINGCODEGEN_API void planningcodegen_inspect(void* tree, real64_T* data);

EXTERN_C PLANNINGCODEGEN_API real64_T planningcodegen_rewire(void* tree,
                                                             real64_T nodeIdx,
                                                             real64_T newParentNodeIdx);

EXTERN_C PLANNINGCODEGEN_API real64_T
planningcodegen_rewireWithPrecomputedCost(void* tree,
                                          real64_T nodeIdx,
                                          real64_T newParentNodeIdx,
                                          real64_T precomputedCost);

#endif
