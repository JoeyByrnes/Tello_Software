/* Copyright 2022 The MathWorks, Inc. */
#ifndef MINTRAVERSAL_HPP
#define MINTRAVERSAL_HPP
#ifdef BUILDING_LIBMWCOVERAGECODEGEN
#include "coveragecodegen/coveragePathUtils.hpp"
#include "coveragecodegen/exportUtils.hpp"

#else
#include "coveragePathUtils.hpp"
#include "exportUtils.hpp"
#endif

#include <memory>

/*Yan Li, Hai Chen, Meng Joo Er, Xinmin Wang,
Coverage path planning for UAVs based on enhanced exact cellular decomposition method,
Mechatronics*/

//DistanceUtils Class hosts methods that need to be mocked for unit tests.
namespace uav {
    namespace coverage {
        class LIBMWCOVERAGE_API DistanceUtils
        {
        public:
            virtual double  findMinimumDistanceSweepPattern(size_t& candidateSweepindex, std::vector<CoveragePolygon>& polygons, size_t currentSweepIndex, size_t startIndex, size_t visitIndex);
            virtual double computeLandingDistance(Point3D Landing, CoveragePolygon& cp, size_t sweepIndex);
            virtual double computeTakeoffDistance(Point3D Takeoff, CoveragePolygon& cp, size_t& sweepIndex);
            virtual ~DistanceUtils() {}
        };


        //Debug utilities to log and display all traversed paths in an organised user-friendly format.
        void  pushToCandidateOptions(std::vector<std::vector<size_t>>& candidateStack, std::vector<std::vector<size_t>>& candidateSweepStack, std::vector<double>& candidateCostStack, std::vector<size_t> candidatePath, std::vector<size_t> candidateSweepSequence, double Cost);

        void  printRecursionResults(std::vector<std::vector<size_t>>& candidateStack, std::vector<double>& candidateCostStack, size_t index, size_t startIndex, std::vector<size_t>& Neighbours,bool debugFlag);

        void designTraversalPath(std::vector<size_t>& Path, std::vector<size_t>& SweepSequence, std::vector<size_t>& candidatePath, std::vector<size_t>& candidateSweepSequence, size_t startIndex, size_t sweepIndex);

        //Optimizer class for MinTraversal recursion.
        class LIBMWCOVERAGE_API MinTraversalPathOptimizer
        {
        public:
            MinTraversalPathOptimizer(std::vector<size_t>AdjacencyInfo, std::vector<size_t>PolygonIndices, std::vector<double>startPoses, std::vector<double>endPoses, double TakeoffArr[], double LandingArr[], std::shared_ptr<DistanceUtils>);
            void planMinTraversalPath(std::vector<size_t>& Path, std::vector<size_t>& SweepSequence, double& minCost, size_t startIndex, std::vector<size_t>candidateSet);
            void subRegionConnection(std::vector<size_t>& Path, std::vector<size_t>& SweepSequence, double& minCost, size_t startIndex, size_t sweepIndex, std::vector<size_t>candidateSet);
            void trivialPathConnection(std::vector<size_t>&SpecifiedSequence, std::vector<size_t>&SweepSequence, double &minCost);

            //Distance Utility.
            //
            std::shared_ptr<DistanceUtils> DistUtils;

            //DistanceUtils *DistUtils;
            //Set to true to print traversal stack.
            bool DebugFlag = false;
        private:
            PolygonGraph PolyGraph;

        };
        //Log Jump from an index.
        void logJump(size_t startIndex,bool debugFlag);


    }
}
#endif

