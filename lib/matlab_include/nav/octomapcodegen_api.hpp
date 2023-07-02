/* Copyright 2019-2022 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for Octomap.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifndef OCTOMAPCODEGEN_API_H_
#define OCTOMAPCODEGEN_API_H_

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomapcodegen/octomapcodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "octomapcodegen_util.hpp"
#endif

/**
 * @brief Intialize algorithm with resolution
 *
 * @param resolution Resolution of the octree leaf nodes (in meters)
 * @return Octomap internal object
 */
EXTERN_C OCTOMAP_CODEGEN_API void* octomapInitialize_real64(real64_T resolution);

/**
 * @brief Free memory and reset to initial state
 * @param[in] map Octomap internal object
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapCleanup_real64(void* map);

/**
 * @brief Get resolution of octree
 *
 * @param[in] map Octomap internal object
 * @return Resolution of octree leaf nodes (in meters)
 */
EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetResolution_real64(void* map);

/**
 * @brief Get resolution of octree
 *
 * @param[in] map Octomap internal object
 * @return Number of octree leaf nodes (in meters)
 */
EXTERN_C OCTOMAP_CODEGEN_API uint32_T octomapGetNumLeafNodes_real64(void* map);

/**
 * @brief Set resolution of octree
 * @param[in] map Octomap internal object
 * @param[in] resolution Resolution of octree leaf nodes (in meters)
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapSetResolution_real64(void* map, real64_T resolution);

/**
 * @brief Get the dimension of the octree based map (with the inserted observations so far)
 * @param[in] map Octomap internal object
 * @param[out] dimensions A 3x2 matrix with [xmin xmax; ymin ymax; zmin zmax] (in meters)
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapGetMapDimensions_real64(void* map, real64_T* dimensions);

/**
 * @brief Manipulate log_odds value of a voxel directly.
 * @param[in] map Octomap internal object
 * @param[in] xyz 3d coordinate of the voxel that is to be updated
 * @param[in] occupied ProbHit/ProbMiss Value to be added (+) to existing log_odds value of node
 * after converting to logodds
 * @param[in] lazyEval Whether update of inner nodes is omitted after the update (default: false).
 *        This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapUpdateNodeBoolean_real64(void* map, real64_T* xyz, boolean_T occupied, boolean_T lazyEval);

/**
 * @brief Manipulate log_odds value of a voxel directly.
 * @param[in] map Octomap internal object
 * @param xyz 3d coordinate of the voxel that is to be updated
 * @param probUpdate Value to be added (+) to existing log_odds value of node after converting
 * to logodds
 * @param lazyEval Whether update of inner nodes is omitted after the update (default: false).
 *        This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapUpdateNodeDouble_real64(void* map, real64_T* xyz, real64_T probUpdate, boolean_T lazyEval);

/**
 * @brief Directly set the log_odds value of a voxel
 * @param[in] map Octomap internal object
 * @param xyz 3d coordinate of the voxel that is to be set
 * @param prob Probability value to be set for a node after converting to logodds
 * @param lazyEval Whether update of inner nodes is omitted after the update (default: false).
 *        This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapSetNodeValue_real64(void* map, real64_T* xyz, real64_T prob, boolean_T lazyEval);

/**
 * @brief Get occupancy value for one or more voxels
 * @param[in] map Octomap internal object
 * @param[in] xyz 3d coordinates of the voxels that are to be updated. This is an N-by-3 matrix.
 * @param[in] nRows number of xyz coordinates. 
 * @param[out] occupancyValues Occupancy value of the voxels. This is a vector of length N.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapGetOccupancy_real64(void* map, real64_T* xyz, uint32_T nRows, real64_T* occupancyValues);

/**
 * @brief Integrate a Pointcloud (in sensor coordinate frame)
 * @param[in] map Octomap internal object
 * @param[in] origin origin of sensor in global reference frame, the format being [x y z qw qx qy
 * qz]
 * @param[in] points Pointcloud (measurement endpoints), in sensor coordinate frame. Points datatype is "double"
 * @param[in] nPoints number of points.
 * @param[in] maxRange maximum range for how long individual beams are inserted (default -1:
 * complete beam)
 * @param[in] lazyEval whether update of inner nodes is omitted after the update (default: false).
 *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
 * @param[in] discretize whether the scan is discretized first into octree key cells (default:
 * false). This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a
 * potential speedup.*
 * @return true if pointCloud have inf otherwise false
 */
EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapInsertPointCloud_real64(void* map, real64_T* origin,
                          real64_T* points,
                          uint32_T nPoints,
                          real64_T maxRange,
                          boolean_T lazyEval,
                          boolean_T discretize);

/**
 * @brief Integrate a Pointcloud (in sensor coordinate frame)
 * @param[in] map Octomap internal object
 * @param[in] origin origin of sensor in global reference frame, the format being [x y z qw qx qy
 * qz]
 * @param[in] points Pointcloud (measurement endpoints), in sensor coordinate frame. Points datatype is "single"
 * @param[in] nPoints number of points.
 * @param[in] maxRange maximum range for how long individual beams are inserted (default -1:
 * complete beam)
 * @param[in] lazyEval whether update of inner nodes is omitted after the update (default: false).
 *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
 * @param[in] discretize whether the scan is discretized first into octree key cells (default:
 * false). This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a
 * potential speedup.*
 * @return true if pointCloud have inf otherwise false
 */
EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapInsertPointCloud_real32(void* map, real64_T* origin,
                          real32_T* points,
                          uint32_T nPoints,
                          real64_T maxRange,
                          boolean_T lazyEval,
                          boolean_T discretize);

/**
 * @brief Updates the occupancy of all inner nodes to reflect their children's occupancy.
 * @param[in] map Octomap internal object
 * If you performed batch-updates with lazy evaluation enabled, you must call this
 * before any queries to ensure correct multi-resolution behavior.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapUpdateInternalOccupancy_real64(void* map);

/**
 * @brief Lossless compression of the octree: A node will replace all of its eight
 * children if they have identical values. You usually don't have to call
 * prune() after a regular occupancy update, updateNode() incrementally
 * prunes all affected nodes.
 * @param[in] map Octomap internal object
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapPrune_real64(void* map);

/**
 * @brief Creates the maximum likelihood map by calling toMaxLikelihood on all
 * tree nodes, setting their occupancy to the corresponding occupancy thresholds.
 * This enables a very efficient compression if you call prune() afterwards.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapToMaxLikelihood_real64(void* map);

/**
 * @brief Extract the values needed for visualizing the octree data representation as a
 * occupancy map
 * @param[in] map Octomap internal object
 * @param[in]  maxDepth The depth at which the visualization data needs to be extracted
 * @param[out] outData a matrix of size Nx6, where N is the number of nodes and columns: x, y, z,
 * scale, occupancy value, depth
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapExtractVisualizationData_real64(void* map, real64_T maxDepth, real64_T* outData);

/**
 * @return memory usage of the complete octree in bytes (may vary between architectures)
 */
EXTERN_C OCTOMAP_CODEGEN_API uint32_T octomapMmemoryUsage_real64(void* map);

/**
 * @brief Inflate the octree leaf nodes with a radius
 * @param[in] map Octomap internal object
 * @param[in] radius This is the inflation radius
 * @param[in] occThreshold This is the occupancy threshold set by the user
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapInflate_real64(void* map, real64_T radius, real64_T occThreshold);

/**
 * @brief Set the minimum and maximum threshold for occupancy clamping
 * @param[in] map Octomap internal object
 * @param[in] clampingThresMin The minimum threshold at which the probability saturates
 * @param[in] clampingThresMax The maximum threshold at which the probability saturates
 * This should be called before inserting any observations.
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapSetClampingThreshold_real64(void* map, real64_T clampingThresMin, real64_T clampingThresMax);

/**
 * @brief Reads OcTree from a binary file.
 * @param[in] map Octomap internal object
 * Existing nodes of the tree are deleted before the tree is read.
 * @param[in] filename path to binary .bt file
 * @return true if filename path is exist otherwise false.
 */
EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapReadBinary_real64(void* map, const char* filename);

/**
 *readBinaryImpl
 */

/**
 * @brief Writes compressed maximum likelihood OcTree to a binary stream.
 * @param[in] map Octomap internal object
 * The OcTree is first converted to the maximum likelihood estimate and pruned
 * for maximum compression.
 * @param[in] filename path to binary .ot file
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapWriteBinary_real64(void* map, const char* filename);

/**
 *writeBinaryImpl
 */

/**
 * @brief Read the file header, create the appropriate class and deserialize.
 * @param[in] map Octomap internal object
 * This creates a new octree which you need to delete yourself.
 * @param[in] filename path to binary .ot file
 * @return true if filename path is valid otherwise false
 */
EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapRead_real64(void* map, const char* filename);

/**
 *readImpl
 */

/**
 * @brief Writes data to a octree stream.
 * @param[in] map Octomap internal object
 * Write file header and complete tree to file (serialization)
 * @param filename path to binary .ot file
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapWrite_real64(void* map, const char* filename);

/**
 *writeImpl
 */

/**
 * @brief Get probability of a HIT (i.e., occupied)
 * @param[in] map Octomap internal object
 * @return Hit probability (a global setting on octree)
 */
EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetProbHit_real64(void* map);

/**
 * @brief Get probability of a MISS (i.e., free)
 * @param[in] map Octomap internal object
 * @return Miss probability (a global setting on octree)
 */
EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetProbMiss_real64(void* map);

/**
 * @brief data size of _map by converting octomap data to binary char array
 * @param[in] map Octomap internal object
 * @return Data size of _map by converting octomap data to binary char array
 */
EXTERN_C OCTOMAP_CODEGEN_API uint32_T octomapGetSizeSerializationData_real64(void* map);

/**
 * @brief serialize the data in _map by converting octomap data to binary char array
 * @param[in] map Octomap internal object
 * @param[out] data A binary char array
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapSerialization_real64(void* map, char* data);

/**
 * @brief deserialize the data as binary char array into a octree data
 * @param[in] map Octomap internal object
 * @param[in,out] data A binary char array created by serialization()
 * @param[in] n size of data
 * @return true if binary char array is right otherwise false
 */
EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapDeserialization_real64(void* map, char* data, uint32_T n);

/**
 * @brief Deserialize the full Octomap ROS msg data into a octree data
 * @param[in] map Octomap internal object
 * @param[in] resolution a double value in octomap_msgs/Octomap Resolution field
 * @param[in] data a int8 array in octomap_msgs/Octomap Data field
 * @param[in] n size of data
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapDeserializationFullROSMsgData_real64(void* map, real64_T resolution, char* data, uint32_T n);

/**
 * @brief deserialize the binary Octomap ROS msg data into a octree data
 * @param[in] map Octomap internal object
 * @param[in] resolution a double value in octomap_msgs/Octomap Resolution field
 * @param[in] data a int8 array in octomap_msgs/Octomap Data field
 * @param[in] n size of data
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapDeserializationBinaryROSMsgData_real64(void* map, real64_T resolution, char* data, uint32_T n);

/**
 * @brief Performs ray-casting in 3d and find intersecting point
 * Rays are cast from 'origin' with given directions, the first non-free
 * cell is returned. This could also be the origin node if it is occupied or unknown.
 *
 * @param[in]  map Octomap internal object
 * @param[in]  ptStart starting coordinate of ray
 * @param[in]  ptDirections An Mx3 matrix. Each row is a vector pointing in the direction of the raycast (NOT a point in space). Does not need to be normalized.
 * @param[in]  nPtDirection number of point directions
 * @param[in]  occupiedThreshold Controls whether a cell is determined to be occupied during raycast
 * @param[in]  ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false.
 * @param[in]  maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
 * @param[out] outData an Mx4 matrix, first column is the occupancy probability [0, 1) for known cells and 0.5 for unknown cells, the next 3 columns contain end points of the ray
 */
EXTERN_C OCTOMAP_CODEGEN_API void octomapGetRayIntersection_real64(
        void* map,
        real64_T* ptStart,
        real64_T* ptDirections,
        uint32_T nPtDirection,
        real64_T occupiedThreshold,
        boolean_T ignoreUnknownCells,
        real64_T maxRange,
        real64_T* outData);

/// @brief Returns the probability threshold for an occupied cell
/// @return probability value ranging between [0, 1]
EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetOccupancyProbThreshold(void* map);

/// @brief Set the probability threshold for an occupied cell
/// @param[in] probability value ranging between [0, 1]
EXTERN_C OCTOMAP_CODEGEN_API void octomapSetOccupancyProbThreshold(void* map, real64_T probability);

/// @brief Returns the probability threshold for a free cell
/// @return probability value ranging between [0, 1]
EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetFreeProbThreshold(void* map);

/// @brief Set the probability threshold for a free cell
/// @param[in] probability value ranging between [0, 1]
EXTERN_C OCTOMAP_CODEGEN_API void octomapSetFreeProbThreshold(void* map, real64_T probability);

/// @brief Check locations for free, occupied, unknown values
/// @param[in] map Octomap internal object
/// @param[in] 3d coordinates of the voxels that are to be updated. This is an N-by-3 matrix.
/// @param[in] nRows number of xyz coordinates. 
/// @param[out] ternaryValues Tri-valued output (0=free, 1=occupied, -1=unknown)
EXTERN_C OCTOMAP_CODEGEN_API void octomapCheckOccupancy_real64(void* map, real64_T* xyz, uint32_T nRows, real64_T* ternaryValues);

#endif /* OCTOMAPCODEGEN_API_H_ */
