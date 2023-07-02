/* Copyright 2020 The MathWorks, Inc. */

/**
 * @file
 * Functions used for UAV Dubins motion primitive calculations.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C++98 standard.
 */



#ifndef UAVDUBINSCODEGEN_DUBINS_FUNCTORS_HPP_
#define UAVDUBINSCODEGEN_DUBINS_FUNCTORS_HPP_

#ifdef BUILDING_LIBMWUAVDUBINSCODEGEN
#include "uavdubinscodegen/uavdubinscodegen_constants.hpp"
#include "uavdubinscodegen/uavdubinscodegen_connection.hpp"
#include "uavdubinscodegen/uavdubinscodegen_pathsegments.hpp"
#include "uavdubinscodegen/uavdubinscodegen_parallel_range.hpp"
#include "uavdubinscodegen/uavdubinscodegen_util.hpp"
#else
// To deal with the fact that PackNGo has no include file hierarchy during test
#include "uavdubinscodegen_constants.hpp"
#include "uavdubinscodegen_connection.hpp"
#include "uavdubinscodegen_pathsegments.hpp"
#include "uavdubinscodegen_parallel_range.hpp"
#include "uavdubinscodegen_util.hpp"
#endif


namespace uav {
namespace dubins {
/**
 * @brief Functor for calculating the distance along UAV Dubins motion primitive segments
 *
 * This function is used to allow both parallel (with TBB) and serial execution.
 */
template <typename T>
class DubinsDistanceFunctor {
  private:
    void* m_connObj;
    T* m_startPose;
    const uint32_T m_maxNumPoses;
    const uint32_T m_numStartPoses;
    const uint32_T m_numGoalPoses;
    T* m_goalPose;
    const T* idx;
    const boolean_T m_flagOptimal;
    const T m_minTurningRadius;
    real64_T* m_sp;
    real64_T* m_gp;
    real64_T* m_fpa;
    real64_T* m_a;
    real64_T* m_mtr;
    real64_T* m_h;
    real64_T* m_mt;
    real64_T* m_ml;
    std::vector<real64_T> motionLengths;
    std::vector<real64_T> motionCosts;
    UavDubinsConnection* obj;
    std::vector<uav::UavDubinsPathsegments> pathSegObjs;
    const uint32_T m_nRows;
    
public:
    DubinsDistanceFunctor(void* connObj,
            T* startPose,
            const uint32_T numStartPoses,
            T* goalPose,
            const uint32_T numGoalPoses,
            const T* idx1,
            const boolean_T flagOptimal,
            const T minTurningRadius,
            real64_T* sp,
            real64_T* gp,
            real64_T* fpa,
            real64_T* a,
            real64_T* mtr,
            real64_T* h,
            real64_T* mt,
            real64_T* ml)
        : m_connObj(connObj)
        , m_startPose(startPose)
        , m_maxNumPoses(numStartPoses > numGoalPoses ? numStartPoses : numGoalPoses)
        , m_numStartPoses(numStartPoses)
        , m_numGoalPoses(numGoalPoses)
        , m_goalPose(goalPose) 
        , idx(idx1)
        , m_flagOptimal(flagOptimal) 
        , m_minTurningRadius(minTurningRadius)
        ,m_sp(sp)
        ,m_gp(gp)
        ,m_fpa(fpa)
        ,m_a(a)
        ,m_mtr(mtr)
        ,m_h(h)
        ,m_mt(mt)
        ,m_ml(ml)
        ,m_nRows(flagOptimal!=1 ? 28 : 1) {
            
        obj = static_cast<UavDubinsConnection *>(m_connObj);
        pathSegObjs.resize(m_nRows*m_maxNumPoses);
        std::vector<real64_T> tempVar{0, uav::nan ,0, 0};
        std::vector<real64_T> s{50 ,50,50,50};
        
        if(m_flagOptimal)
        {
            for(uint32_T j= 0 ; j < m_maxNumPoses ; j++)
            {                
                pathSegObjs[j].setMethod(s, s , 0 , obj->getAirSpeed() , m_minTurningRadius , 0 , static_cast<int>(idx[0]) , tempVar);
            }
        }
        else
        {
            for(uint32_T i = 0 ; i < m_nRows ; i++)
            {
                for(uint32_T j= 0 ; j < m_maxNumPoses ; j++)
                {                   
                    pathSegObjs[i*m_maxNumPoses + j].setMethod(s, s , 0 , obj->getAirSpeed(),
                            m_minTurningRadius , 0 , static_cast<int>(idx[i]) , tempVar);
                }
            }
        }

        boolean_T Dubins2DPathTypes[6] = {true, true, true, true, true, true};
        std::vector<real64_T> startPoseDubins2D;
        std::vector<real64_T> goalPoseDubins2D;
        
        startPoseDubins2D.resize(m_numStartPoses * 3);
        goalPoseDubins2D.resize(m_numGoalPoses * 3);
        
        for(uint32_T i = 0 ; i < m_numStartPoses ; i++)
        {
            startPoseDubins2D[i] = m_startPose[i];
            startPoseDubins2D[i+m_numStartPoses] = -m_startPose[i + 1 * m_numStartPoses];
            startPoseDubins2D[i + 2*m_numStartPoses] = -m_startPose[i + 3 * m_numStartPoses];
        }
        
        for(uint32_T i = 0 ; i < m_numGoalPoses ; i++)
        {
            goalPoseDubins2D[i] = m_goalPose[i];
            goalPoseDubins2D[i + m_numGoalPoses] = -m_goalPose[i + m_numGoalPoses];
            goalPoseDubins2D[i + 2* m_numGoalPoses] = -m_goalPose[i + 3 * m_numGoalPoses];
        }
        
        motionLengths.resize(3*6*m_maxNumPoses);
        motionCosts.resize(6*m_maxNumPoses);
        #ifdef IS_NOT_MATLAB_HOST
            autonomousDubinsSegmentsCodegen_real64(&startPoseDubins2D[0],
                    m_numStartPoses, // number of start pose
                    &goalPoseDubins2D[0],
                    m_numGoalPoses, // number of goal pose
                    m_minTurningRadius,
                    Dubins2DPathTypes,
                    false,       // return optimal path (so isOptimal should be true)
                    2,          // calculate 2 outputs
                    &motionCosts[0], &motionLengths[0], nullptr);
        #else
            autonomousDubinsSegmentsCodegen_tbb_real64(&startPoseDubins2D[0],
                    m_numStartPoses, // number of start pose
                    &goalPoseDubins2D[0],
                    m_numGoalPoses, // number of goal pose
                    m_minTurningRadius,
                    Dubins2DPathTypes,
                    false,       // return optimal path (so isOptimal should be true)
                    2,          // calculate 2 outputs
                    &motionCosts[0], &motionLengths[0], nullptr);
        #endif
    }

    /**
     * Functor to calculate the distance along Dubins segments for a
     * given range of start and goal points.
     *
     * This functor can be executed as-is (for serial execution) or
     * be part of parallel execution, e.g. through parallel_for
     *
     * @param range The range information for how many start and
     * goal locations should be processed.
     */
    void operator()(const uav::ParallelRange<uint32_T>& range) const {
        // The conditional statements are constant, so should be
        // compiler-optimized.

        if(m_numStartPoses >= m_numGoalPoses && m_numGoalPoses == 1){

            T a[4],b[4];
            b[0] = m_goalPose[0];
            b[1] = -m_goalPose[1];
            b[2] = -m_goalPose[2];
            b[3] = -m_goalPose[3];

            for(uint32_T i = range.begin(); i < range.end(); ++i){
                
                a[0] = m_startPose[i];
                a[1] = -m_startPose[i + m_maxNumPoses];
                a[2] = -m_startPose[i + 2 * m_maxNumPoses];
                a[3] = -m_startPose[i + 3 * m_maxNumPoses];
                
                std::vector<uav::UavDubinsPathsegments> tempPathSegObjs;
                tempPathSegObjs.resize(m_nRows);
                
                real64_T tempMotionCosts[6];
                real64_T tempMotionLengths[3][6];
                for(uint32_T j = 0 ; j < m_nRows ; j++)
                {
                    tempPathSegObjs[j] = pathSegObjs[j * m_maxNumPoses + i];
                }
                for(uint32_T j = 0 ; j < 6; j++)
                {
                    tempMotionCosts[j] = motionCosts[j + 6* i];
                    for(uint32_T k = 0 ; k < 3; k++)
                    {
                        tempMotionLengths[k][j] = motionLengths[k + 3*j + 3*6*i];
                    }
                }
                obj->connectAirplaneXYZ(&a[0] ,&b[0] , m_flagOptimal, idx, tempMotionLengths, tempMotionCosts, &tempPathSegObjs[0]);
                for(uint32_T j = 0 ; j < static_cast<uint32_T>(tempPathSegObjs.size()) ; ++j){
                    m_sp[4*(j * m_maxNumPoses + i)+0] = a[0];
                    m_sp[4*(j * m_maxNumPoses + i)+1] = -a[1];
                    m_sp[4*(j * m_maxNumPoses + i)+2] = -a[2];
                    m_sp[4*(j * m_maxNumPoses + i)+3] = -a[3];
                    m_gp[4*(j * m_maxNumPoses + i)+0] = b[0];
                    m_gp[4*(j * m_maxNumPoses + i)+1] = -b[1];
                    m_gp[4*(j * m_maxNumPoses + i)+2] = -b[2];
                    m_gp[4*(j * m_maxNumPoses + i)+3] = -b[3];
                   
                    m_fpa[j * m_maxNumPoses + i] = tempPathSegObjs[j].FlightPathAngle;
                    m_a[j * m_maxNumPoses + i] = tempPathSegObjs[j].AirSpeed;
                    m_mtr[j * m_maxNumPoses + i] = tempPathSegObjs[j].MinTurningRadius;
                    m_h[j * m_maxNumPoses + i] = tempPathSegObjs[j].HelixRadius;
                    m_mt[j * m_maxNumPoses + i] = tempPathSegObjs[j].MotionTypes;
                    m_ml[4*(j * m_maxNumPoses + i)+0] = tempPathSegObjs[j].MotionLengths[0];
                    m_ml[4*(j * m_maxNumPoses + i)+1] = tempPathSegObjs[j].MotionLengths[1];
                    m_ml[4*(j * m_maxNumPoses + i)+2] = tempPathSegObjs[j].MotionLengths[2];
                    m_ml[4*(j * m_maxNumPoses + i)+3] = tempPathSegObjs[j].MotionLengths[3];
                }
            }
        }
        else if (m_numStartPoses < m_numGoalPoses && m_numStartPoses == 1) {
            // Single startPose, multiple goalPoses
            T a[4],b[4];
            a[0] = m_startPose[0];
            a[1] = -m_startPose[1];
            a[2] = -m_startPose[2];
            a[3] = -m_startPose[3];
            for (uint32_T i = range.begin(); i < range.end(); ++i) {
                b[0] = m_goalPose[i];
                b[1] = -m_goalPose[i + m_maxNumPoses];
                b[2] = -m_goalPose[i + 2 * m_maxNumPoses];
                b[3] = -m_goalPose[i + 3 * m_maxNumPoses];
                
                std::vector<uav::UavDubinsPathsegments> tempPathSegObjs;
                tempPathSegObjs.resize(m_nRows);
                
                real64_T tempMotionCosts[6];
                real64_T tempMotionLengths[3][6];
                
                for(uint32_T j = 0 ; j < m_nRows ; j++)
                {
                    tempPathSegObjs[j] = pathSegObjs[j * m_maxNumPoses + i];
                }
                for(uint32_T j = 0 ; j < 6; j++)
                {
                    tempMotionCosts[j] = motionCosts[j + 6* i];
                    for(uint32_T k = 0 ; k < 3; k++)
                    {
                        tempMotionLengths[k][j] = motionLengths[k + 3*j + 3*6*i];
                    }
                }

                obj->connectAirplaneXYZ(&a[0] , &b[0] , m_flagOptimal, idx, tempMotionLengths, tempMotionCosts, &tempPathSegObjs[0]);
                for(uint32_T j = 0 ; j < static_cast<uint32_T>(tempPathSegObjs.size()) ; ++j){
                    m_sp[4*(j * m_maxNumPoses + i)+0] = a[0];
                    m_sp[4*(j * m_maxNumPoses + i)+1] = -a[1];
                    m_sp[4*(j * m_maxNumPoses + i)+2] = -a[2];
                    m_sp[4*(j * m_maxNumPoses + i)+3] = -a[3];
                    
                    m_gp[4*(j * m_maxNumPoses + i)+0] = b[0];
                    m_gp[4*(j * m_maxNumPoses + i)+1] = -b[1];
                    m_gp[4*(j * m_maxNumPoses + i)+2] = -b[2];
                    m_gp[4*(j * m_maxNumPoses + i)+3] = -b[3];
                    
                    m_fpa[j * m_maxNumPoses + i] = tempPathSegObjs[j].FlightPathAngle;
                    m_a[j * m_maxNumPoses + i] = tempPathSegObjs[j].AirSpeed;
                    m_mtr[j * m_maxNumPoses + i] = tempPathSegObjs[j].MinTurningRadius;
                    m_h[j * m_maxNumPoses + i] = tempPathSegObjs[j].HelixRadius;
                    m_mt[j * m_maxNumPoses + i] = tempPathSegObjs[j].MotionTypes;
                    
                    m_ml[4*(j * m_maxNumPoses + i)+0] = tempPathSegObjs[j].MotionLengths[0];
                    m_ml[4*(j * m_maxNumPoses + i)+1] = tempPathSegObjs[j].MotionLengths[1];
                    m_ml[4*(j * m_maxNumPoses + i)+2] = tempPathSegObjs[j].MotionLengths[2];
                    m_ml[4*(j * m_maxNumPoses + i)+3] = tempPathSegObjs[j].MotionLengths[3];
                }
            }

        } else {
            // Multiple startPoses, multiple goalPoses
            
            T a[4];
            T b[4];
            for (uint32_T i = range.begin(); i < range.end(); ++i) {
                a[0] = m_startPose[i];
                a[1] = -m_startPose[i + m_maxNumPoses];
                a[2] = -m_startPose[i + 2 * m_maxNumPoses];
                a[3] = -m_startPose[i + 3 * m_maxNumPoses];                
                b[0] = m_goalPose[i];
                b[1] = -m_goalPose[i + m_maxNumPoses];
                b[2] = -m_goalPose[i + 2 * m_maxNumPoses];
                b[3] = -m_goalPose[i + 3 * m_maxNumPoses];
                
                std::vector<uav::UavDubinsPathsegments> tempPathSegObjs;
                tempPathSegObjs.resize(m_nRows);
                real64_T tempMotionCosts[6];
                real64_T tempMotionLengths[3][6];
                
                for(uint32_T j = 0 ; j < m_nRows ; j++)
                {
                    tempPathSegObjs[j] = pathSegObjs[j * m_maxNumPoses + i];
                }
                
                for(uint32_T j = 0 ; j < 6; j++)
                {
                    tempMotionCosts[j] = motionCosts[j + 6* i];
                    for(uint32_T k = 0 ; k < 3; k++)
                    {
                        tempMotionLengths[k][j] = motionLengths[k + 3*j + 3*6*i];
                    }
                }

                obj->connectAirplaneXYZ(&a[0] , &b[0] , m_flagOptimal, idx, tempMotionLengths, tempMotionCosts, &tempPathSegObjs[0]);
                
                for(uint32_T j = 0 ; j < static_cast<uint32_T>(tempPathSegObjs.size()) ; ++j){
                    m_sp[4*(j * m_maxNumPoses + i)+0] = a[0];
                    m_sp[4*(j * m_maxNumPoses + i)+1] = -a[1];
                    m_sp[4*(j * m_maxNumPoses + i)+2] = -a[2];
                    m_sp[4*(j * m_maxNumPoses + i)+3] = -a[3];
                    
                    m_gp[4*(j * m_maxNumPoses + i)+0] = b[0];
                    m_gp[4*(j * m_maxNumPoses + i)+1] = -b[1];
                    m_gp[4*(j * m_maxNumPoses + i)+2] = -b[2];
                    m_gp[4*(j * m_maxNumPoses + i)+3] = -b[3];
                    
                    m_fpa[j * m_maxNumPoses + i] = tempPathSegObjs[j].FlightPathAngle;
                    m_a[j * m_maxNumPoses + i] = tempPathSegObjs[j].AirSpeed;
                    m_mtr[j * m_maxNumPoses + i] = tempPathSegObjs[j].MinTurningRadius;
                    m_h[j * m_maxNumPoses + i] = tempPathSegObjs[j].HelixRadius;
                    m_mt[j * m_maxNumPoses + i] = tempPathSegObjs[j].MotionTypes;
                    
                    m_ml[4*(j * m_maxNumPoses + i)+0] = tempPathSegObjs[j].MotionLengths[0];
                    m_ml[4*(j * m_maxNumPoses + i)+1] = tempPathSegObjs[j].MotionLengths[1];
                    m_ml[4*(j * m_maxNumPoses + i)+2] = tempPathSegObjs[j].MotionLengths[2];
                    m_ml[4*(j * m_maxNumPoses + i)+3] = tempPathSegObjs[j].MotionLengths[3];
                }
            }
        }
    }
};
} // namespace dubins
} // namespace uav

#endif /* UAVDUBINSCODEGEN_DUBINS_FUNCTORS_HPP_ */
