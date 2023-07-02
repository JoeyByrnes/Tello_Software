/* Copyright 2022 The MathWorks, Inc. */
#ifndef CERESCODEGEN_API_HPP
#define CERESCODEGEN_API_HPP

#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/cerescodegen_spec.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy during test */
    #include "cerescodegen_spec.hpp"
#endif

/** Construct mw_ceres::FactorIMU object and returns its pointer. **/
EXTERN_C CERESCODEGEN_API void* cerescodegen_constructIMUFactor(int32_T* ids,
                        real64_T sampleRate,
                        real64_T* gravityAcceleration,
                        real64_T* gyroBiasNoise,
                        real64_T* accelBiasNoise,
                        real64_T* gyroNoise,
                        real64_T* accelNoise,
                        real64_T* gyroReadings,
                        real64_T* accelReadings, 
                        real64_T numReadings, 
                        real64_T* sensorTransform);

/** Construct mw_ceres::FactorIMUGS object and returns its pointer. **/
EXTERN_C CERESCODEGEN_API void* cerescodegen_constructIMUGSFactor(int32_T* ids,
                        real64_T sampleRate,
                        real64_T* gravityAcceleration,
                        real64_T* gyroBiasNoise,
                        real64_T* accelBiasNoise,
                        real64_T* gyroNoise,
                        real64_T* accelNoise,
                        real64_T* gyroReadings,
                        real64_T* accelReadings, 
                        real64_T numReadings, 
                        real64_T* sensorTransform);

/** Destruct mw_ceres::FactorIMU**/
EXTERN_C CERESCODEGEN_API void cerescodegen_destructIMUFactor(void* objPtr);

/** Destruct mw_ceres::FactorIMUGS**/
EXTERN_C CERESCODEGEN_API void cerescodegen_destructIMUGSFactor(void* objPtr);

/** Predict the Pose and Velocity. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_predictIMU(void* objPtr, const real64_T* prevBias,
                                                       const real64_T* prevPose, const real64_T* prevVel,
                                                       real64_T* predictedPose, real64_T* predictedVel); 

/** Construct mw_ceres::FactorGraph object and returns its pointer. **/
EXTERN_C CERESCODEGEN_API void* cerescodegen_constructFactorGraph();

/** Destruct mw_ceres::FactorGraph**/
EXTERN_C CERESCODEGEN_API void cerescodegen_destructFactorGraph(void* objPtr);

/** Get factor graph node ids. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_getNodeIDs(void* objPtr, real64_T* output, real64_T* outputLen, const int32_T* groupID, const int32_T numgroupID,
    const char_T* nodeType, const int32_T nodeTypeLen, const char_T* factorType, const int32_T factorTypeLen);

/** Add imu factor to factor graph **/
EXTERN_C CERESCODEGEN_API real64_T cerescodegen_addFactorIMU(void* objPtr, int32_T* ids,
                        real64_T sampleRate,
                        real64_T* gravityAcceleration,
                        real64_T* gyroBiasNoise,
                        real64_T* accelBiasNoise,
                        real64_T* gyroNoise,
                        real64_T* accelNoise,
                        real64_T* gyroReadings,
                        real64_T* accelReadings, 
                        real64_T numReadings,
                        const int32_T* groupIds,
                        const int32_T numGroupIds,
                        real64_T* sensorTransform);

/** Add imu factor connecting gravity and scale nodes to factor graph **/
EXTERN_C CERESCODEGEN_API real64_T cerescodegen_addFactorIMUGS(void* objPtr, int32_T* ids,
                        real64_T sampleRate,
                        real64_T* gravityAcceleration,
                        real64_T* gyroBiasNoise,
                        real64_T* accelBiasNoise,
                        real64_T* gyroNoise,
                        real64_T* accelNoise,
                        real64_T* gyroReadings,
                        real64_T* accelReadings, 
                        real64_T numReadings,
                        const int32_T* groupIds,
                        const int32_T numGroupIds,
                        real64_T* sensorTransform);

/** Add gaussian noise model factor to factor graph**/
EXTERN_C CERESCODEGEN_API void cerescodegen_addFactorGaussianNoiseModel(void* objPtr, const char_T* factorType, const int32_T factorTypeLen,
    const int32_T* ids, const int32_T numIds, const real64_T* measurement, const int32_T numMeasurement, const real64_T* information,
    const int32_T numInformation, const int32_T numFactors, const int32_T* groupIds, const int32_T numGroupIds, real64_T* output, real64_T* outputLen);

/** Add camera projection factor to factor graph**/
EXTERN_C CERESCODEGEN_API void cerescodegen_addFactorCameraProjection(void* objPtr, const char_T* factorType, const int32_T factorTypeLen,
    const int32_T* ids, const int32_T numIds, const real64_T* measurement, const int32_T numMeasurement, const real64_T* information,
    const int32_T numInformation, const int32_T numFactors, const int32_T* groupIds, const int32_T numGroupIds, real64_T* sensorTform, real64_T* output, real64_T* outputLen);

/** Get number of nodes added to factor graph. **/
EXTERN_C CERESCODEGEN_API real64_T cerescodegen_getNumNodes(void* objPtr);

/** Get number of factors added to factor graph . **/
EXTERN_C CERESCODEGEN_API real64_T cerescodegen_getNumFactors(void* objPtr);

/** Get state of factor graph node**/
EXTERN_C CERESCODEGEN_API void cerescodegen_getNodeState(void* objPtr, int32_T* vid, int32_T vidNum, real64_T* state, real64_T* stateLen);

/** Get type of factor graph node. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_getNodeType(void* objPtr, real64_T vid, char_T* type, real64_T* typeLen);

/** Set factor graph node state. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_setNodeState(void* objPtr, int32_T* vid, int32_T vidNum, real64_T* state, int32_T stateLen,
    real64_T* output, real64_T* outputLen);

/** Fix factor graph node. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_fixNode(void* objPtr, int32_T* vid, int32_T vidNum, real64_T* output, real64_T* outputLen);

/** Free factor graph node. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_freeNode(void* objPtr, int32_T* vid, int32_T vidNum, real64_T* output, real64_T* outputLen);

/** Query if factor graph node is fixed. **/
EXTERN_C CERESCODEGEN_API real64_T cerescodegen_isNodeFixed(void* objPtr, real64_T vid);

/** Query if factor graph has the specified node**/
EXTERN_C CERESCODEGEN_API boolean_T cerescodegen_hasNode(void* objPtr, real64_T vid);

/** Query if the factor graph is connected. **/
EXTERN_C CERESCODEGEN_API boolean_T cerescodegen_isConnected(void* objPtr);

/** Optimize factor graph. **/
EXTERN_C CERESCODEGEN_API void cerescodegen_optimize(void* objPtr, real64_T* opts, real64_T* solInfo);

#endif
