/* Copyright 2020 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for UAV Dubins motion primitive calculations.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifndef UAVDUBINSCODEGEN_DUBINS_API_H_
#define UAVDUBINSCODEGEN_DUBINS_API_H_

#ifdef BUILDING_LIBMWUAVDUBINSCODEGEN
#include "uavdubinscodegen/uavdubinscodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "uavdubinscodegen_util.hpp"
#endif

EXTERN_C UAVDUBINSCODEGEN_API void* uavDubinsConnectionObj(const double airSpeed,
        const double maxRollAngle, double* flightPathAngleLimit,
        double* disabledPathTypes, uint32_T numDisabledPathTypes);

/**
 * @brief Free memory and reset to initial state
 * @param[in] connObj uavDubinsConnection internal object
 */
EXTERN_C UAVDUBINSCODEGEN_API void uavDubinsConnectionObjCleanup(void* connObj);

/**
 * @copydoc uavDubinsDistanceCodegen_real64
 */
EXTERN_C UAVDUBINSCODEGEN_API void uavDubinsDistanceCodegen_real64(
        void* connObj,
        real64_T* startPose,
        const uint32_T numStartPoses,
        real64_T* goalPose,
        const uint32_T numGoalPoses,
        const boolean_T flagOptimal,
        const real64_T minTurningRadius,
        real64_T* sp,
        real64_T* g,
        real64_T* fpa,
        real64_T* a,
        real64_T* mtr,
        real64_T* h,
        real64_T* mt,
        real64_T* ml);

#endif /* UAVDUBINSCODEGEN_DUBINS_API_H_ */
