/* Copyright 2022 The MathWorks, Inc. */
#ifndef EXHAUSTIVE_HPP
#define EXHAUSTIVE_HPP
#ifdef BUILDING_LIBMWCOVERAGECODEGEN
#include "coveragecodegen/coveragePathUtils.hpp"
#include "coveragecodegen/exportUtils.hpp"
#else
#include "coveragePathUtils.hpp"
#include "exportUtils.hpp"
#endif
//For std::shared ptr
#include<memory> 
/*Marina Torres, David A. Pelta, Jose L. Verdegay, Juan C. Torres,
    Coverage path planning with unmanned aerial vehicles for 3D terrain reconstruction,
    Expert Systems with Applications,*/

//Optimizer class for MinTraversal recursion.
namespace uav
{
    class LIBMWCOVERAGE_API CtrlCHandler
    {
    public:
       virtual void ctrlC_Check() const
        {

       }

        virtual void injectCtrlC() const
        {

        }
      virtual ~CtrlCHandler() = default;
    };
}
namespace uav {
    namespace coverage {
        void computeItoaBase4(size_t arr[], size_t number, size_t size);
        bool ExhaustivePathTraversal(std::vector<CoveragePolygon>& CoveragePolygons, std::vector<size_t>& areaPermutation, size_t& SweepPermutationNumber, double& minimumCost, Point3D& Takeoff, Point3D& Landing);
        //LIBMWCOVERAGE_API descriptor signals that dll must expose class interface (and is used by builtins)
        class LIBMWCOVERAGE_API ExhaustivePathOptimizer
        {
        public:
            PolygonGraph PolyGraph;
            ExhaustivePathOptimizer(const std::vector<size_t>& AdjacencyInfo, const std::vector<size_t>& PolygonIndices, const std::vector<double>& startPoses, const std::vector<double>& endPoses, const double TakeoffArr[], const double LandingArr[]);
            void planExhaustivePath(std::vector<size_t>& Path, std::vector<size_t>& SweepSequence, double& minimumCost, const std::vector<size_t>& visitingSequence, const bool visitOrderSpecified, const int minAdjacencyCount,CtrlCHandler const &);
            bool validateAreaPermutation(std::vector<size_t>& areaPermutation, const int minAdjacencyCount, const bool DistinctLanding);

        };

    }
}
#endif
