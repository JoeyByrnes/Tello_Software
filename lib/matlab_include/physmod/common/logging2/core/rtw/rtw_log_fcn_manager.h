/* Copyright 2021 The MathWorks, Inc. */

#ifndef logging2_core_rtw_log_fcn_manager_h
#define logging2_core_rtw_log_fcn_manager_h


#include "physmod/common/logging2/core/rtw/SscRTWLogFcnManagerFwd.h"

/* These functions should not be part of SscRTWLogFcnManager.h since
 * SscRTWLogFcnManager.h is deployed and following functions need to be
 * published. Including these functions in SscRTWLogFcnManager.h will result in
 * redeclaration of SscRTWLogFcnManager struct in rapid accel code because of
 * the code generation function calls in SinkBlock.
 *
 * All the functions used to generate code from SinkBlock are part of the
 * published code. The functions that are used to generate code when making sink
 * data are part of the deployed code.
 */

/* Wrapper function for mRtwSetupLogFcn in SscRTWLogFcnManager. It sets up the
 * SscRTWLogFcnManagerObject */
extern
void ssc_rtw_setup_log_fcn(SscRTWLogFcnManager* manager);

/* Wrapper function for mRtwLogFcn in SscRTWLogFcnManager. */
extern
void ssc_rtw_log_fcn(SscRTWLogFcnManager* manager,
                     real_T const         timeStep,
                     real_T const*        voidInputs,
                     double*              buffer);

/* Wrapper function for mRtwDestroy in SscRTWLogFcnManager. */
extern
void ssc_rtw_destroy(SscRTWLogFcnManager* manager);

#endif
