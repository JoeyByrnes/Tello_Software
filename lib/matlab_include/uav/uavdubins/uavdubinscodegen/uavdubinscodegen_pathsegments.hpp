/* Copyright 2020 The MathWorks, Inc. */
#ifndef UAVDUBINSCODGEN_PATHSEGMENTS_HPP_
#define UAVDUBINSCODGEN_PATHSEGMENTS_HPP_

#include <vector>

#include "uavdubinscodegen_util.hpp"

namespace uav{
    class UavDubinsPathsegments{
    public:
        UavDubinsPathsegments()
        {
            FlightPathAngle = 0;
            AirSpeed = 10;
            MinTurningRadius = 18.6784;
            HelixRadius = 0;
            MotionTypes = 0;
            MotionLengths = {0 , 0 , 0 , 0};
            Length = 0;
        }
        void setMethod(std::vector<double> startPose, std::vector<double> goalPose,
                double flightPathAngle, double airSpeed, double minTurningRadius,
                double helixRadius, double motionTypes, std::vector<double> motionLengths)
        {
            StartPose = startPose;
            GoalPose  = goalPose;
            FlightPathAngle = flightPathAngle;
            AirSpeed = airSpeed;
            MinTurningRadius = minTurningRadius;
            HelixRadius = helixRadius;
            MotionTypes = static_cast<int>(motionTypes);
            MotionLengths[0] = motionLengths[0];
            MotionLengths[1] = motionLengths[1];
            MotionLengths[2] = motionLengths[2];
            MotionLengths[3] = motionLengths[3];
            Length = 0;
            for(size_t i = 0 ; i < 4; i++)
            {
                Length += motionLengths[i];
            }
        }
        
        //Properties
        std::vector<double> StartPose;
        std::vector<double> GoalPose;
        double FlightPathAngle;
        double AirSpeed;
        double MinTurningRadius;
        double HelixRadius;
        double Length;
        int MotionTypes;
        std::vector<double> MotionLengths{0,0,0,0};       
    };
}

#endif
