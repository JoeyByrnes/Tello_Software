/* Copyright 2019-2021 The MathWorks, Inc. */
#ifndef PRIORITYQUEUE_CODEGEN_API_HPP
#define PRIORITYQUEUE_CODEGEN_API_HPP

#ifdef BUILDING_LIBMWPRIORITYQUEUECODEGEN
#include "priorityqueuecodegen/priorityqueue_codegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "priorityqueue_codegen_util.hpp"
#endif

/** Construct PriorityQueueImpl object and returns its pointer. **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void* priorityqueuecodegen_constructPQ(const real64_T dim,
    const real64_T primeIdx);

/** Destruct nav::PriorityQueueImpl **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_destructPQ(void* objPtr);

/** Push a node onto the priority queue **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_push(void* objPtr, const real64_T* newNode);

/** Return the lowest cost node in the priority queue. **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_top(void* objPtr, real64_T* topNode, real64_T* nodeId);

/** Remove the lowest cost node from the priority queue. **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_pop(void* objPtr);

/** Return the size of the priority queue. **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_size(void* objPtr);

/** Check if the priority queue is empty or not. */
EXTERN_C PRIORITYQUEUE_CODEGEN_API boolean_T priorityqueuecodegen_isEmpty(void* objPtr);

/** Get the data dimension **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_getDataDim(void* objPtr);


// nav::NodeMap

/** Construct nav::NodeMap object and returns its pointer. */
EXTERN_C PRIORITYQUEUE_CODEGEN_API void* priorityqueuecodegen_constructNodeMap(const real64_T dim);

/** Destruct nav::NodeMap **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_destructNodeMap(void* objPtr);

/** Get the data dimension **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_nodemap_getDataDim(void* objPtr);

/** Get the number of nodes **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_nodemap_getNumNodes(void* objPtr);

/** Trace back the path from a given node to the root node **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_nodemap_traceBack(void* objPtr,
    const real64_T idx,
    real64_T* pathNodeData, 
    real64_T* numPathNodes);

/** Create two new maps: 1) newNodeId --> parentId, 2) newNodeId --> newNodeData **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_nodemap_insertNode(void* objPtr,
    const real64_T* newNodeData,
    const real64_T parentId);

/** Return data associated with the given nodeId **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_nodemap_getNodeData(void* objPtr,
    const real64_T nodeId,
    real64_T* data,
    real64_T* parentId);


// nav::SimpleMap

/** Construct nav::SimpleMap and returns its pointer. **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void* priorityqueuecodegen_constructSimpleMap(const real64_T dim);

/** Destruct nav::SimpleMap **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_destructSimpleMap(void* objPtr);

/** Get data dimension **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_simplemap_getDataDim(void* objPtr);

/** Get size of the map **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_simplemap_getSize(void* objPtr);

/** Associate data with the given id **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_simplemap_insertData(void* objPtr,
    const real64_T id,
    const real64_T* neweData);

/** Return data associated with the given id **/
EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_simplemap_getData(void* objPtr,
    const real64_T nodeId,
    real64_T* data);
#endif



