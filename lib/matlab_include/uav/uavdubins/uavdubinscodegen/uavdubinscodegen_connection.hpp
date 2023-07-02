/* Copyright 2020 The MathWorks, Inc. */
#ifndef UAVDUBINSCODEGEN_CONNECTION_HPP_
#define UAVDUBINSCODEGEN_CONNECTION_HPP_

#include <limits>
#include <string>
#include <vector>
#include <cmath> //tan
#include <algorithm>

#ifdef BUILDING_LIBMWUAVDUBINSCODEGEN
#include "uavdubinscodegen/uavdubinscodegen_util.hpp"
#include "uavdubinscodegen/uavdubinscodegen_constants.hpp"
#include "uavdubinscodegen/uavdubinscodegen_pathsegments.hpp"
#else
#include "uavdubinscodegen_util.hpp"
#include "uavdubinscodegen_constants.hpp"
#include "uavdubinscodegen_pathsegments.hpp"
#endif

#ifdef IS_NOT_MATLAB_HOST
#include "autonomouscodegen_dubins_api.hpp"
#include "uavdubinscodegen_connection_api.hpp"
#else
#include "autonomouscodegen_dubins_tbb_api.hpp"
#include "uavdubinscodegen_connection_tbb_api.hpp"
#endif

namespace uav {
   class UavDubinsConnection {
    public:
        UavDubinsConnection(double airSpeed, double maxRollAngle,
                double flightPathAngleLimits[2], std::vector<double> disabledPathTypes);
       
        UavDubinsConnection(double airSpeed, double maxRollAngle, 
                double flightPathAngleLimits[2], double disabledPathTypes[], uint32_T mNumDisabledPathTypes);
        
        //Setter functions
        void setAirSpeed(double airSpeed);
        void setMaxRollAngle(double maxRollAngle);
        void setFlightPathAngleLimits(double flightPathAngleLimits[2]);
        void setDisabledPathTypes(std::vector<double> disabledPathTypes);
        //Getter functions
        double getAirSpeed();
        double getMaxRollAngle();
        double* getFlightPathAngleLimits();
        std::vector<double> getDisabledPathTypes();
        
        void connectAirplaneXYZ(const double startPose[4], const double goalPose[4],
                const bool flagOptimal, const double* idx, double motionLengths[3][6],
                double motionCosts[6], UavDubinsPathsegments* psegObjs);
private:
    // private properties
    double mAirSpeed;
    double mMaxRollAngle;
    double mFlightPathAngleLimits[2];
    std::vector<double> mDisabledPathTypes;
    const std::vector<std::string> AllPathTypes{"LSLN","LSRN","RSLN","RSRN","RLRN","LRLN",
                "HlLSL","HlLSR","HrRSL","HrRSR","HrRLR","HlLRL",
                "LSLHl","LSRHr", "RSLHl", "RSRHr", "RLRHr", "LRLHl",
                "LRSL", "LRSR", "LRLR", "RLSR", "RLRL", "RLSL",
                "LSRL", "RSRL", "LSLR", "RSLR"}; 
    const double Gravity = 9.8;
    double ErrorTolerance = 1.0/100000;
    double mMinTurningRadius;
    // All paths are enabled
    boolean_T Dubins2DPathTypes[6] = {true, true, true, true, true, true};
    // private methods
    double updateMinTurningRadius() const;
    void optimizeMidAltitude(double startPoseDubins2D[3], double goalPoseDubins2D[3], double theta,
            int optStartCurve, double maxFlightPathAngle, double absAltitude, double motionLength[2][3],
            double motionCost[2], char motionType[2][3] , double lm[2], int* qualityFlag );
    
    void addArc(std::string tempMotionType, double tempMotionCost, double* tempMotionLength,
            double maxFlightPathAngle, double lm, char strMotion, double& lowestMotionCost,
            UavDubinsPathsegments* psegObjs, int optStartCurve, bool flagOptimal, int signFlightPathAngle);
    };
}

#endif
