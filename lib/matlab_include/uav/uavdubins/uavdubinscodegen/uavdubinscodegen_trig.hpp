/* Copyright 2020 The MathWorks, Inc. */

/**
 * @file
 * Trigonometric utility functions for all autonomous functionality in codegen.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C++98 standard.
 */

#ifndef UAVDUBINS_TRIG_H_
#define UAVDUBINS_TRIG_H_

#include <cmath> // for std::fabs

#include <math.h> // for fmod, sqrt, floor, atan2

#ifdef BUILDING_LIBMWAUTONOMOUSCODEGEN
#include "uavdubinscodegen/uavdubinscodegen_constants.hpp" // for pi, twoPi, tooSmall
#include "uavdubinscodegen/uavdubinscodegen_util.hpp"
#else
#include "uavdubinscodegen_constants.hpp"
#include "uavdubinscodegen_util.hpp"
#endif

/** @file
 * Declarations for trigonometric operations.
 */

namespace uav {

/// Wrap angle to interval [0, 2pi]
/**
 * @param angleRad Input angle (in radians)
 * @return Angle wrapped to interval [0, 2pi] (in radians)
 */
inline real64_T wrapTo2Pi(const real64_T angleRad) {
    // Deal with very small negative angles. And, to deal with floating point error
    // and degenerate cases (where one of the arc lengths might be around 0 or 2pi).
    // Rather than choosing a longer path, we choose a shortest path which makes
    // values really close to 0 and 2pi equal to 0.          
    
    if (angleRad < 0 && angleRad > -tooSmall) {
        return 0;
    }
    double xm = angleRad - twoPi * floor(angleRad / twoPi);
    if (twoPi - xm < 0.5 * 1e-6){
        xm = 0;
    }
    return xm;
}

/// Wrap angle to interval [-pi, pi]
/**
 * @param[in] angleRad Input angle (in radians)
 * @return Angle wrapped to interval [-pi, pi] (in radians)
 */
inline real64_T wrapToPi(const real64_T angleRad) {
    real64_T wrappedAngle = fmod(angleRad, twoPi);
    if (wrappedAngle < -getPi()) {
        wrappedAngle += twoPi;
    }
    else if (wrappedAngle > getPi()) {
        wrappedAngle -= twoPi;
    }
    return wrappedAngle;
}
} // namespace uav

#endif // UAVDUBINS_TRIG_H_
