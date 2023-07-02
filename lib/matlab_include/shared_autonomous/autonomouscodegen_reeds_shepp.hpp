/* Copyright 2017-2021 The MathWorks, Inc. */

/**
 * @file
 * Main interfaces for Reeds-Shepp motion primitive calculations.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C++98 standard.
 */

#ifndef AUTONOMOUSCODEGEN_REEDSSHEPP_H_
#define AUTONOMOUSCODEGEN_REEDSSHEPP_H_

#include <math.h>    // for sqrt, trigonometric functions
#include <algorithm> // for max
#include <vector>

#ifdef BUILDING_LIBMWAUTONOMOUSCODEGEN
#include "autonomouscodegen/autonomouscodegen_reeds_shepp_primitives.hpp"
#else
// To deal with the fact that PackNGo has no include file hierarchy during test
#include "autonomouscodegen_reeds_shepp_primitives.hpp"
#endif

namespace autonomous {

namespace reedsshepp {
/*
 * The code for ReedsShepp primitives and computation of the various
 * segment paths is inspired from Open Motion Planning Library source.
 */

/*
 * RSSegmentType - Enumeration for holding segment type
 */
enum RSSegmentType { Left = 0, Right, Straight, NOP };

const RSSegmentType pathToSegment[18][5] = {
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Right, autonomous::reedsshepp::Left,
     autonomous::reedsshepp::NOP, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Left, autonomous::reedsshepp::Right,
     autonomous::reedsshepp::NOP, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Right, autonomous::reedsshepp::Left,
     autonomous::reedsshepp::Right, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Left, autonomous::reedsshepp::Right,
     autonomous::reedsshepp::Left, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight,
     autonomous::reedsshepp::Left, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight,
     autonomous::reedsshepp::Right, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Right,
     autonomous::reedsshepp::Left, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Left,
     autonomous::reedsshepp::Right, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight,
     autonomous::reedsshepp::Right, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight,
     autonomous::reedsshepp::Left, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Right,
     autonomous::reedsshepp::Left, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Left,
     autonomous::reedsshepp::Right, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Right,
     autonomous::reedsshepp::NOP, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Left,
     autonomous::reedsshepp::NOP, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Left,
     autonomous::reedsshepp::NOP, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight, autonomous::reedsshepp::Right,
     autonomous::reedsshepp::NOP, autonomous::reedsshepp::NOP},
    {autonomous::reedsshepp::Left, autonomous::reedsshepp::Right, autonomous::reedsshepp::Straight,
     autonomous::reedsshepp::Left, autonomous::reedsshepp::Right},
    {autonomous::reedsshepp::Right, autonomous::reedsshepp::Left, autonomous::reedsshepp::Straight,
     autonomous::reedsshepp::Right, autonomous::reedsshepp::Left}};

/*
 * shortestReedsSheppPath returns shortest normalized Reeds-Shepp path.
 */
inline autonomous::reedsshepp::ReedsSheppPath shortestReedsSheppPath(real64_T x,
                                                                     real64_T y,
                                                                     real64_T phi,
                                                                     real64_T reverseCost) {
    autonomous::reedsshepp::ReedsSheppPath minPath, tempPath;
    real64_T minLength, tempLength;

    autonomous::reedsshepp::computeCSCPath(x, y, phi, reverseCost, minPath);
    minLength = minPath.length();

    autonomous::reedsshepp::computeCCCPath(x, y, phi, reverseCost, tempPath);
    tempLength = tempPath.length();

    if (tempLength < minLength) {
        minLength = tempLength;
        minPath = tempPath;
    }

    autonomous::reedsshepp::computeCCCCPath(x, y, phi, reverseCost, tempPath);
    tempLength = tempPath.length();

    if (tempLength < minLength) {
        minLength = tempLength;
        minPath = tempPath;
    }

    autonomous::reedsshepp::computeCCSCPath(x, y, phi, reverseCost, tempPath);
    tempLength = tempPath.length();

    if (tempLength < minLength) {
        minLength = tempLength;
        minPath = tempPath;
    }

    autonomous::reedsshepp::computeCCSCCPath(x, y, phi, reverseCost, tempPath);
    tempLength = tempPath.length();

    if (tempLength < minLength) {
        minLength = tempLength;
        minPath = tempPath;
    }

    return minPath;
}

/*
 * computeReedsSheppPath - compute Reeds-Shepp shortest/all path.
 */
inline autonomous::reedsshepp::ReedsSheppPath computeReedsSheppPath(
    const real64_T initialConfiguration[3],
    const real64_T finalConfiguration[3],
    const real64_T turningRadius,
    const real64_T reverseCost) {
    real64_T x1 = initialConfiguration[0];
    real64_T y1 = initialConfiguration[1];
    real64_T theta1 = initialConfiguration[2];
    real64_T x2 = finalConfiguration[0];
    real64_T y2 = finalConfiguration[1];
    real64_T theta2 = finalConfiguration[2];

    real64_T dx = x2 - x1;
    real64_T dy = y2 - y1;
    real64_T c = cos(theta1);
    real64_T s = sin(theta1);

    real64_T x = c * dx + s * dy;
    real64_T y = -s * dx + c * dy;
    real64_T phi = theta2 - theta1;

    return shortestReedsSheppPath(x / turningRadius, y / turningRadius, phi, reverseCost);
}

/*
 * AllReedsSheppPath returns all normalized Reeds-Shepp path.
 */
inline std::vector<autonomous::reedsshepp::ReedsSheppPath> AllReedsSheppPath(
    real64_T x,
    real64_T y,
    real64_T phi,
    real64_T forwardCost,
    real64_T reverseCost,
    const boolean_T isOptimal,
    const boolean_T allPathTypes[autonomous::reedsshepp::TotalNumPaths],
    const uint32_T numTotalPathTypes) {
    std::vector<autonomous::reedsshepp::ReedsSheppPath> allPaths;
    allPaths.reserve(numTotalPathTypes);

    autonomous::reedsshepp::computeCSCPathAll(x, y, phi, forwardCost, reverseCost, &allPathTypes[0],
                                              &allPaths);
    autonomous::reedsshepp::computeCCCPathAll(x, y, phi, forwardCost, reverseCost, &allPathTypes[0],
                                              &allPaths);
    autonomous::reedsshepp::computeCCCCPathAll(x, y, phi, forwardCost, reverseCost,
                                               &allPathTypes[0], &allPaths);
    autonomous::reedsshepp::computeCCSCPathAll(x, y, phi, forwardCost, reverseCost,
                                               &allPathTypes[0], &allPaths);
    autonomous::reedsshepp::computeCCSCCPathAll(x, y, phi, forwardCost, reverseCost,
                                                &allPathTypes[0], &allPaths);

    if (isOptimal) {
        real64_T optimalPath = autonomous::inf;
        uint32_T optimalInd = 0;

        // Round up to 15 decimal point to get same optimal path in
        // all system (maci, linux, windows). (Geck --> g1770360)
        real64_T round_val = pow(10.0, 15);
        for (uint32_T ind = 0; ind < numTotalPathTypes; ++ind) {
            // Round up to 15 decimal point to get same optimal path in
            // all system (maci, linux, windows . (Geck --> g1770360)
            real64_T tempcost = round(allPaths[ind].cost() * round_val) / round_val;
            if (optimalPath > tempcost) {
                optimalPath = tempcost;
                optimalInd = ind;
            }
        }

        std::vector<autonomous::reedsshepp::ReedsSheppPath> path(1);
        path[0] = allPaths[optimalInd];

        return path;
    }
    return allPaths;
}

/*
 * computeAllReedsSheppPaths - compute Reeds-Shepp shortest/all path.
 */
inline std::vector<autonomous::reedsshepp::ReedsSheppPath> computeAllReedsSheppPaths(
    const real64_T initialConfiguration[3],
    const real64_T finalConfiguration[3],
    const real64_T turningRadius,
    const real64_T forwardCost,
    const real64_T reverseCost,
    const boolean_T isOptimal,
    const boolean_T allPathTypes[autonomous::reedsshepp::TotalNumPaths],
    const uint32_T numTotalPathTypes) {
    real64_T x1 = initialConfiguration[0];
    real64_T y1 = initialConfiguration[1];
    real64_T theta1 = initialConfiguration[2];
    real64_T x2 = finalConfiguration[0];
    real64_T y2 = finalConfiguration[1];
    real64_T theta2 = finalConfiguration[2];

    real64_T dx = x2 - x1;
    real64_T dy = y2 - y1;
    real64_T c = cos(theta1);
    real64_T s = sin(theta1);

    real64_T x = c * dx + s * dy;
    real64_T y = -s * dx + c * dy;
    real64_T phi = theta2 - theta1;

    return AllReedsSheppPath(x / turningRadius, y / turningRadius, phi, forwardCost, reverseCost,
                             isOptimal, &allPathTypes[0], numTotalPathTypes);
}

/* 
 * Compute direction value for start/goal pose.
 *
 * 1) For segmentLengths input {0, 9, 2, -10, 0},
 *
 * computeDirectionAtEdges({0, 9, 2, -10, 0}, true) or
 * computeDirectionAtEdges({0, 9, 2, -10, 0}) will return 1, sign of first
 * non-zero segment (sign(9)) i.e. direction value of start pose.
 *
 * computeDirectionAtEdges({0, 9, 2, -10, 0}, false) will return -1, sign
 * of last non-zero segment (sign(-10)) i.e. direction value of goal pose.
 *
 * 2) For segmentLengths input {-8, -1, 2, 10, 0},
 *
 * computeDirectionAtEdges({-8, -1, 2, 10, 0}, true) or
 * computeDirectionAtEdges({-8, -1, 2, 10, 0}) will return -1, sign of first
 * non-zero segment (sign(-8)) i.e. direction value of start pose.
 *
 * computeDirectionAtEdges({-8, -1, 2, 10, 0}, false) will return 1, sign
 * of last non-zero segment (sign(10)) i.e. direction value of goal pose.
 *
 */
inline real64_T computeDirectionAtEdges(const real64_T* segmentLengths, boolean_T startFlag = true)
{
    real64_T direction = 1;
    for (uint32_T i = 0; i < 5; i++){
        if(segmentLengths[i] != 0){
            direction = ((segmentLengths[i]<0) ? -1 : 1);
            //Exit from loop for start pose direction computation 
            if (startFlag){
                break;
            }
        }                
    }
    return direction;
}

/*
 * interpolateAlongInitializedRSPath - interpolate points along a
 * pre-computed ReedsShepp path.
 */
inline real64_T interpolateAlongInitializedRSPath(const real64_T pathLength,
                                              const RSSegmentType* segments,
                                              const real64_T* segmentLengths,
                                              const real64_T* from,
                                              const real64_T* towards,
                                              const real64_T t,
                                              const real64_T turningRadius,
                                              real64_T* state) {
    
    real64_T direction = 1;
    
    if (t <= 0.0) {
        state[0] = from[0];
        state[1] = from[1];
        state[2] = from[2];
        
        // To ensure first pose direction is correct
        direction = computeDirectionAtEdges(segmentLengths);
    } else if (t >= 1.0) {
        state[0] = towards[0];
        state[1] = towards[1];
        state[2] = towards[2];
        
        // To ensure last pose direction is correct
        direction = computeDirectionAtEdges(segmentLengths, false);
    } else {
                
        // Initialize at [0, 0, theta]
        state[0] = 0;
        state[1] = 0;
        state[2] = from[2];

        real64_T seg = t * pathLength;
        real64_T v, phi;

        // Compute normalized update.
        for (uint32_T i = 0; i < 5 && seg > 0; ++i) {
            
            if (segmentLengths[i] < 0) {
                v = std::max(-seg, segmentLengths[i]);
                seg += v;
                direction = -1;
            } else {
                v = std::min(seg, segmentLengths[i]);
                seg -= v;
                direction = 1;
            }

            phi = state[2];

            switch (segments[i]) {
            case autonomous::reedsshepp::Left:                
                state[0] += sin(phi + v) - sin(phi);
                state[1] += -cos(phi + v) + cos(phi);
                state[2] = autonomous::wrapToTwoPi(phi + v);
                break;

            case autonomous::reedsshepp::Right:                
                state[0] += -sin(phi - v) + sin(phi);
                state[1] += cos(phi - v) - cos(phi);
                state[2] = autonomous::wrapToTwoPi(phi - v);
                break;

            case autonomous::reedsshepp::Straight:               
                state[0] += v * cos(phi);
                state[1] += v * sin(phi);
                break;

            case autonomous::reedsshepp::NOP:
                // no update needed
                break;
            }
        }
        // Denormalize and update.
        state[0] = from[0] + state[0] * turningRadius;
        state[1] = from[1] + state[1] * turningRadius;
    }
    
    return direction;
}

/*
 * interpolateRS - interpolate along ReedsShepp curve between states.
 */
inline void interpolateRS(const real64_T* from,
                          const real64_T* towards,
                          const real64_T maxDistance,
                          const uint32_T numSteps,
                          const real64_T turningRadius,
                          const real64_T reverseCost,
                          real64_T* state) {
    // Compute ReedsShepp path.
    autonomous::reedsshepp::ReedsSheppPath path =
        computeReedsSheppPath(from, towards, turningRadius, reverseCost);

    const RSSegmentType* segments = pathToSegment[path.getPathType()];

    // Compute the fraction of path to be traversed based on maximum
    // connection distance (maxDistance).
    //  * If the distance between the states is less than maxDistance, we
    //    interpolate all the through to the destination state (towards).
    //  * If the distance between the states is more than maxDistance, we
    //    only interpolate a fraction of the path between the two states.

    // Find the distance between the states.
    const real64_T pathLength = path.length();
    const real64_T dist = pathLength * turningRadius;

    // Find the fraction of the path to interpolate.
    const real64_T t = std::min(maxDistance / dist, 1.0);

    // Find interpolation step based on number of steps.
    const real64_T step = t / static_cast<real64_T>(numSteps);

    std::vector<real64_T> temp(3, 0);
    real64_T fraction = 0;

    // Interpolate along transition points
    const uint32_T numTransitions = 4;
    const uint32_T arrLength = numSteps + numTransitions;

    const real64_T* segLengths = path.getSegmentLengths();
    
    for (uint32_T n = 0; n < numTransitions; ++n) {
        // Compute fraction along path corresponding to n-th transition
        // point
        fraction += (autonomous::abs(segLengths[n]) / pathLength);

        // Saturate at maxDistance. This has the effect of returning
        // the ending pose for all transition poses that come after the
        // maxDistance has been reached.
        fraction = std::min(fraction, t);

        interpolateAlongInitializedRSPath(pathLength, segments, segLengths, from, towards, fraction,
                                          turningRadius, &temp[0]);

        state[n] = temp[0];
        state[n + arrLength] = temp[1];
        state[n + 2 * arrLength] = temp[2];
    }

    // Interpolate along path at equidistant intervals
    fraction = 0;
    for (uint32_T n = numTransitions; n < arrLength; ++n) {
        fraction += step;
        interpolateAlongInitializedRSPath(pathLength, segments, segLengths, from, towards, fraction,
                                          turningRadius, &temp[0]);

        state[n] = temp[0];
        state[n + arrLength] = temp[1];
        state[n + 2 * arrLength] = temp[2];
    }
}

/*
 * interpolateReedsSheppSegments - interpolate along given states and Reeds-Shepp curve.
 */
inline void interpolateReedsSheppSegments(const real64_T* from,
                                      const real64_T* towards,
                                      const real64_T* samples,
                                      const uint32_T numSamples,
                                      const real64_T turningRadius,
                                      const real64_T* segmentLengths,
                                      const int32_T* segmentDirections,
                                      const uint32_T* segmentTypes,
                                      real64_T* state,
                                      real64_T* directions) {
    RSSegmentType segments[5] = {static_cast<RSSegmentType>(segmentTypes[0]),
                                     static_cast<RSSegmentType>(segmentTypes[1]),
                                     static_cast<RSSegmentType>(segmentTypes[2]),
                                     static_cast<RSSegmentType>(segmentTypes[3]),
                                     static_cast<RSSegmentType>(segmentTypes[4])};
    
    real64_T temp[3] = {0, 0, 0};

    const real64_T pathLength = segmentLengths[0] + segmentLengths[1] + 
            segmentLengths[2] + segmentLengths[3] + segmentLengths[4];
    
    real64_T normalizedSegmentLengths[5] = {segmentLengths[0] / turningRadius,
                                            segmentLengths[1] / turningRadius,
                                            segmentLengths[2] / turningRadius,
                                            segmentLengths[3] / turningRadius,
                                            segmentLengths[4] / turningRadius};
                                              
    const real64_T normalizedPathLength =
        normalizedSegmentLengths[0] + normalizedSegmentLengths[1] + 
            normalizedSegmentLengths[2] + normalizedSegmentLengths[3] +
            normalizedSegmentLengths[4];        
    
    real64_T round_val = pow(10.0, 15);
    
    normalizedSegmentLengths[0] = round(normalizedSegmentLengths[0] * round_val) / round_val;
    normalizedSegmentLengths[1] = round(normalizedSegmentLengths[1] * round_val) / round_val;
    normalizedSegmentLengths[2] = round(normalizedSegmentLengths[2] * round_val) / round_val;
    normalizedSegmentLengths[3] = round(normalizedSegmentLengths[3] * round_val) / round_val;
    normalizedSegmentLengths[4] = round(normalizedSegmentLengths[4] * round_val) / round_val;
    
    // Round up to 15 decimal point to get same optimal path in all system 
    // (maci, linux, windows). (Geck --> g1770360)
    normalizedSegmentLengths[0] = normalizedSegmentLengths[0]*static_cast<real64_T>(segmentDirections[0]);
    normalizedSegmentLengths[1] = normalizedSegmentLengths[1]*static_cast<real64_T>(segmentDirections[1]);
    normalizedSegmentLengths[2] = normalizedSegmentLengths[2]*static_cast<real64_T>(segmentDirections[2]);
    normalizedSegmentLengths[3] = normalizedSegmentLengths[3]*static_cast<real64_T>(segmentDirections[3]);
    normalizedSegmentLengths[4] = normalizedSegmentLengths[4]*static_cast<real64_T>(segmentDirections[4]);    

    for (uint32_T n = 0; n < numSamples; ++n) {
        // Compute fraction along path corresponding to n-th transition
        // point        
        
        // Round up to 15 decimal point to get same optimal path in all 
        // system (maci, linux, windows). (Geck --> g1770360)
        real64_T roundedSample = round(samples[n] * round_val) / round_val;
        roundedSample = roundedSample / pathLength;               
        
        if (directions){
            directions[n] = interpolateAlongInitializedRSPath(normalizedPathLength, segments, normalizedSegmentLengths,
                    from, towards, roundedSample, turningRadius, &temp[0]);
        }else{
            interpolateAlongInitializedRSPath(normalizedPathLength, segments, normalizedSegmentLengths,
                    from, towards, roundedSample, turningRadius, &temp[0]);
        }

        state[n]                    = round(temp[0] * round_val) / round_val;
        state[n + numSamples]       = round(temp[1] * round_val) / round_val;
        state[n + 2 * numSamples]   = round(temp[2] * round_val) / round_val;        
    }
}

} // namespace reedsshepp
} // namespace autonomous

#endif /* AUTONOMOUSCODEGEN_REEDSSHEPP_H_ */
