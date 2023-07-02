/* Copyright 2019-2020 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for Adaptive Monte Carlo Localization (AMCL).
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifndef MCLCODEGEN_API_H_
#define MCLCODEGEN_API_H_

#ifdef BUILDING_LIBMWMCLCODEGEN
#include "mclcodegen/mclcodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "mclcodegen_util.hpp"
#endif

/**
 * @brief Initialize algorithm with random see
 *
 * @param[in] seed Global random seed used for resampling and initialization.
 * @return Initialized Monte Carlo Localization object.
 */
EXTERN_C MCL_CODEGEN_API void* mclInitializeWithSeed_real64(real64_T seed);

/**
 * @brief Free memory and reset to initial state
 */
EXTERN_C MCL_CODEGEN_API void mclCleanup_real64(void* mclObj);

/**
 * @brief Set the motion model.
 *
 * This method creates an odometry differential drive motion model and
 * assigns the values to its error parameters.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] motionParams Four element array of error parameters.
 */
EXTERN_C MCL_CODEGEN_API void mclSetMotionModel_real64(void* mclObj, const real64_T* motionParams);

/**
 * @brief Sets the occupancy grid map.
 *
 * This method assigns the values from a MATLAB occupancy grid to
 * the AMCL implementation of an occupancy grid.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] width Width of the Occupancy Grid in number of cells.
 * @param[in] height Height of the Occupancy Grid in number of cells.
 * @param[in] resolution Resolution of the Occupancy Grid in meters/cell.
 * @param[in] originx X location for origin. The origin is the center of the map.
 * @param[in] originy Y location for origin. The origin is the center of the map.
 * @param[in] gridCells Occupancy grid data in row-major format 1D array.
 */
EXTERN_C MCL_CODEGEN_API void mclSetOccupancyGrid_real64(void* mclObj,
														 real64_T width,
                                                         real64_T height,
                                                         real64_T resolution,
                                                         real64_T originx,
                                                         real64_T originy,
                                                         const real64_T* gridCells);

/**
 * @brief Set the sensor model.
 *
 * This method creates a likelihood field sensor model for range sensors
 * and assigns the sensor parameters.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] max_beams Max beams to be used for likelihood computation
 * @param[in] z_hit Weight for probability of correct measurement
 * @param[in] z_rand Weight for probability of random measurement
 * @param[in] sigma_hit Standard deviation for noise in correct measurement
 * @param[in] laser_likelihood_max_dist Max distance to search for obstacles
 * @param[in] sensorLimits Min and Max measurement limits of the range sensor, [min max]
 * @param[in] sensorPoseParams Sensor location relative to robot base [x y yaw]
 */
EXTERN_C MCL_CODEGEN_API void mclSetSensorModel_real64(void* mclObj,
                                                       real64_T max_beams,
                                                       real64_T z_hit,
                                                       real64_T z_rand,
                                                       real64_T sigma_hit,
                                                       real64_T laser_likelihood_max_dist,
                                                       const real64_T* sensorLimits,
                                                       const real64_T* sensorPoseParams);

/**
 * @brief Initialize particle filter.
 *
 * This method initializes the particle filter and assigns its
 * parameters. All the particles are initialized to zero.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] minParticles Minimum number of particles
 * @param[in] maxParticles Maximum number of particles
 * @param[in] alphaSlow Parameter that affects recovery behavior
 * @param[in] alphaFast Parameter that affects recovery behavior
 * @param[in] kldErr KLD sampling parameter - error bound
 * @param[in] kldZ KLD sampling parameter - probability bound
 */
EXTERN_C MCL_CODEGEN_API void mclInitializePf_real64(void* mclObj,
													 real64_T minParticles,
                                                     real64_T maxParticles,
                                                     real64_T alphaSlow,
                                                     real64_T alphaFast,
                                                     real64_T kldErr,
                                                     real64_T kldZ);

/**
 * @brief Set initial pose and covariance.
 *
 * This method initializes particles based on given initial pose and
 * covariance.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] poseParams Initial pose of the robot 3 element array.
 * @param[in] covParams Initial covariance 9 element array in column major format.
 */
EXTERN_C MCL_CODEGEN_API void mclSetInitialPose_real64(void* mclObj,
                                                       const real64_T* poseParams,
                                                       const real64_T* covParams);

/**
 * @brief Single call update, calls predict, correct and resample.
 *
 * The method calls the predict, correct and resample methods respectively.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] rangeCount Number of ranges in laser scan.
 * @param[in] rangesParams Range values from laser scanner.
 * @param[in] anglesParams Angles corresponding to the ranges.
 * @param[in] poseParams Odometry pose of the robot [x, y, theta].
 */
EXTERN_C MCL_CODEGEN_API void mclUpdate_real64(void* mclObj,
                                               real64_T rangeCount,
                                               const real64_T* rangesParams,
                                               const real64_T* anglesParams,
                                               const real64_T* poseParams);

/**
 * @brief Get mean and covariance estimate.
 *
 * The method returns mean and covariance of the maximum weighted
 * cluster of particles.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[out] maxWeightHyp 12 elements of mean and covariance in row major format
 */
EXTERN_C MCL_CODEGEN_API void mclGetHypothesis_real64(void* mclObj, real64_T* maxWeightHyp);

/**
 * @brief Set particles and weights.
 *
 * This function allows to set particles and their weights for testing
 * purposes.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] particles Particle cloud where rows represent states, [x y theta]
 * @param[in] weights Particle weights as an array (one per particle)
 */
EXTERN_C MCL_CODEGEN_API void mclSetParticles_real64(void* mclObj,
                                                     real64_T* particles,
                                                     real64_T* weights);

/**
 * @brief Get number of particles currently used in particle filter.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @return Number of particles
 */
EXTERN_C MCL_CODEGEN_API uint32_T mclGetNumParticles_real64(void* mclObj);

/**
 * @brief Get states for all particles.
 *
 * This method returns [x y theta] states of all particles in a
 * vector form. The returned vector is a 3*N vector representing
 * [x, y, theta] states for N particles.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[out] particleInfo A vector representing [x, y, theta] states of particles in column major
 * format
 */
EXTERN_C MCL_CODEGEN_API void mclGetParticles_real64(void* mclObj, real64_T* particleInfo);

/**
 * @brief Motion update on particle filter.
 *
 * This is the prediction step of the particle filter. This function updates
 * the particles based on the motion model and the input odometry pose.
 * The particles are not updated for first call or if robot has moved
 * less then update thresholds.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] poseParams Odometry pose of the robot [x y theta].
 */
EXTERN_C MCL_CODEGEN_API void mclPredict_real64(void* mclObj, const real64_T* poseParams);

/**
 * @brief Sensor update on particle filter.
 *
 * This is the correction step of the particle filter. This function
 * updates the particle weights based on sensor data. The weights are
 * not updated if the robot has moved less then update thresholds.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] rangeCount Number of ranges in laser scan.
 * @param[in] rangesParams Range values from laser scanner.
 * @param[in] anglesParams Angles corresponding to the ranges.
 */
EXTERN_C MCL_CODEGEN_API void mclCorrect_real64(void* mclObj,
                                                real64_T rangeCount,
                                                const real64_T* rangesParams,
                                                const real64_T* anglesParams);

/**
 * @brief Resample particles.
 *
 * This is the resampling step of the particle filter. It resamples
 * particles if resampling interval is satisfied.
 *
 * @param[in] mclObj MonteCarloLocalizationInternal object
 */
EXTERN_C MCL_CODEGEN_API void mclResample_real64(void* mclObj);

/**
 * @brief Set resampling interval.
 *
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] interval Resampling interval.
 */
EXTERN_C MCL_CODEGEN_API void mclSetResamplingInterval_real64(void* mclObj, real64_T interval);

/**
 * @brief Get resampling interval.
 *
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @return resampling interval.
 */
EXTERN_C MCL_CODEGEN_API real64_T mclGetResamplingInterval_real64(void* mclObj);

/**
 * @brief Get occupancy value from occupancy grid using world coordinates
 *
 * This function is used for testing purposes.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] x X world location to check for occupancy
 * @param[in] y Y world location to check for occupancy
 * @return Occupancy value -1 for free, 1 for occupied
 */
EXTERN_C MCL_CODEGEN_API real64_T mclGetOccupancyWorld_real64(void* mclObj, real64_T x, real64_T y);

/**
 * @brief Set state change thresholds to trigger update.
 *
 * This method sets thresholds for states [x y theta] change
 * based on which the motion and sensor updates are triggered.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] x Threshold for change in state X
 * @param[in] y Threshold for change in state Y
 * @param[in] theta Threshold for change in state Theta
 */
EXTERN_C MCL_CODEGEN_API void mclSetUpdateThresholds_real64(void* mclObj,
                                                            real64_T x,
                                                            real64_T y,
                                                            real64_T theta);

/**
 * @brief Get update thresholds.
 *
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[out] x Threshold for change in state X
 * @param[out] y Threshold for change in state Y
 * @param[out] theta Threshold for change in state Theta
 */
EXTERN_C MCL_CODEGEN_API void mclGetUpdateThresholds_real64(void* mclObj,
                                                            real64_T* x,
                                                            real64_T* y,
                                                            real64_T* theta);

/**
 * @brief Returns true if particle filter is updated.
 *
 * This function returns true if the particle filter was updated
 * in the last call to "update". This means "predict" and "correct"
 * methods were executed.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @retval TRUE if particle filter was updated
 * @retval FALSE if it was not updated
 */
EXTERN_C MCL_CODEGEN_API boolean_T mclIsUpdated_real64(void* mclObj);

/**
 * @brief Returns true if particle filter is resampled.
 *
 * This function returns true if the mean estimate was updated
 * in the last call to "update" as a result of resampling.
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @retval TRUE if particle filter was resampled
 * @retval FALSE if it was not resampled
 */
EXTERN_C MCL_CODEGEN_API boolean_T mclIsResampled_real64(void* mclObj);

/**
 * @brief Initialize particles uniformly in entire map.
 *
 * This method initializes particles uniformly in the entire map.
 * This is used to start the global localization.
 * 
 * @param[in] mclObj MonteCarloLocalizationInternal object
 */
EXTERN_C MCL_CODEGEN_API void mclGlobalLocalization_real64(void* mclObj);

/**
 * @brief Set the test hook value
 *
 * This function allows setting the testHookForMemory property.
 * The test hook enables negative tests for memory error.
 * 
 * @param[in] mclObj MonteCarloLocalizationInternal object
 * @param[in] value Boolean value for test hook setting
 */
EXTERN_C MCL_CODEGEN_API void mclSetTestHookForMemory_real64(void* mclObj, boolean_T value);

#endif /* MCLCODEGEN_API_H_ */
