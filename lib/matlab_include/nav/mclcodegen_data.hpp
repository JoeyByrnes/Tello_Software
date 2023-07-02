/* Copyright 2019-2020 The MathWorks, Inc. */

/**
 * @file
 * Data Storage classes for Adaptive Monte Carlo Localization (AMCL).
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C++98 standard.
 */

#ifndef MCLCODEGEN_DATA_H_
#define MCLCODEGEN_DATA_H_

#include <memory>  // For std::unique_ptr
#include <random>  // For std::minstd_rand0 and std::uniform_real_distribution
#include <utility> // For std::pair and std::make_pair
#include <vector>

#ifdef BUILDING_LIBMWMCLCODEGEN
#include "amcl/map/amcl_map.h"
#include "amcl/pf/amcl_pf.h"
#include "amcl/pf/amcl_pf_vector.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/sensors/amcl_odom.h"
#include "mclcodegen/mclcodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "amcl_laser.h"
#include "amcl_map.h"
#include "amcl_odom.h"
#include "amcl_pf.h"
#include "amcl_pf_vector.h"
#include "mclcodegen_util.hpp"
#endif

namespace nav {
/**
 * @brief Implements Data Class for Pose Hypothesis
 */
class MCLHypothesis {

  public:
    double _weight; ///< Total weight

    pf_vector_t _poseMean; ///< Mean of pose estimate

    pf_matrix_t _poseCovariance; ///< Covariance of pose estimate
};

/**
 * @brief Implements Data Class to be passed to global localization sampling function
 */
class GlobalSamplingData {
  public:
    map_t* _mapPtr; ///< Pointer to map used for sampling

    /* Using separate random number generators for XY and Theta produces very bad results.
       The MCL algorithm converges to wrong locations. Need to figure out why this happens. */
    std::minstd_rand0 _randGenerator; ///< Random number generator.

    /* Using separate distribution -PI to PI for Theta produces inferior results. In addition,
       using common distribution will produce comparable results with ROS implementation */
    std::uniform_real_distribution<double>
        _zeroToOne; ///< Uniform distribution for random number generation.

    std::vector<std::pair<int, int> > _freeSpaceIndices; ///< Free indices in the map

    GlobalSamplingData(unsigned int seed = 0)
        : _mapPtr(nullptr)
        , _randGenerator(seed)
        , _zeroToOne(0.0, 1.0) {
    }
};

/**
 * @brief Data storage container for one instance of the Monte Carlo Localization Filter
 */
class MCLData {
  public:
    map_t* _map;                                   ///< Map used in AMCL
    pf_t* _particleFilter;                         ///< Particle filter used in AMCL

	std::unique_ptr<amcl::AMCLLaser> _sensorModel; ///< Sensor model object
    amcl::AMCLOdom _motionModel;                   ///< Motion model object
    pf_vector_t _odometryPose;                     ///< Last stored odometry pose

	boolean_T _isPFInitialized ;				   ///< Has particle filter been initialized? (mclInitialize has been called)
    boolean_T _isPFUpdated;                        ///< Has particle filter been updated? (mclUpdate has been called)
    boolean_T _isResampled;                        ///< Has particle filter been resampled? (mclResample has been called)
    boolean_T _updateLikelihood;

    uint32_T _resampleCount;                       ///< Current number of correction steps that have been called. If _resampleCount %
                                                   ///< _resampleInterval == 0, resampling occurs
    uint32_T _resampleInterval;                    ///< How often particle filter should be resampled. =1 means every correction step.

    real64_T _xThreshold;                          ///< Threshold for detecting non-static motion in x-direction
    real64_T _yThreshold;                          ///< Threshold for detecting non-static motion in y-direction
    real64_T _thetaThreshold;                      ///< Threshold for detecting non-static motion in theta angle

    real64_T _minSensorRange;                      ///< Minimum range of sensor to use for localization
    real64_T _maxSensorRange;                      ///< Maximum range of sensor to use for localization

    nav::GlobalSamplingData _globalLocalizationData; ///< Map data for global localization

    boolean_T _testHookForMemory;                  ///< Test hook for memory testing

	MCLData(unsigned int seed)
        : _map(nullptr)
        , _particleFilter(nullptr)
        , _isPFInitialized(false)
        , _isPFUpdated(false)
        , _isResampled(false)
        , _updateLikelihood(false)
        , _resampleCount(0)
        , _resampleInterval(1)
        , _xThreshold(0.2)
        , _yThreshold(0.2)
        , _thetaThreshold(0.2)
        , _minSensorRange(0.0)
        , _maxSensorRange(6.0)
        , _globalLocalizationData(nav::GlobalSamplingData(seed))
        , _testHookForMemory(false) {
    }
};

} // namespace nav

#endif /* MCLCODEGEN_DATA_H_ */
