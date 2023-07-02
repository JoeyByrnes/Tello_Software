/* Copyright 2022 The MathWorks, Inc. */

#ifndef CUSTOMCOVUTILS_HPP
#define CUSTOMCOVUTILS_HPP
#include <algorithm>
#include <numeric>
#include <cmath>
#include <climits>
#include <iostream>
#include<vector>
#include<cfloat>
namespace uav {
    namespace coverage {
        class Point3D
        {
        public:
            double X;
            double Y;
            double Z;
            bool IsValid;
            //constructor
            Point3D(double a, double b, double c);
            Point3D();
            bool operator == (const Point3D& point) const;

        };

        class CoverageAlternative
        {
        public:
            Point3D Start;
            Point3D Exit;
            CoverageAlternative();
            CoverageAlternative(Point3D a, Point3D b);
        };

        //Class that holds the coverage alternatives of a polygon.
        class CoveragePolygon
        {
        public:
            /*Marina Torres, David A. Pelta, Jose L. Verdegay, Juan C. Torres,
            Coverage path planning with unmanned aerial vehicles for 3D terrain reconstruction,
            Expert Systems with Applications,*/
            //Alternatives for coverage of a single polygon.
            //index 0 Standard clockwise forward sweep
            //index 1 Counter clockwise forward sweep
            //index 2 Clockwise and opposite line sweep
            //index 3 Counter clockwise opposite.
            CoverageAlternative CoverageAlt[4];
        };

        std::vector<CoveragePolygon> constructCoverageOptions(const std::vector<double>& startPoses, const std::vector<double>& endPoses);

        double computeDistance(const Point3D& A, const Point3D& B);
        double computeDistance2D(const Point3D& A, const Point3D& B);


        class PolygonGraph
        {//Utility functions that are common to both MinTraversal and Exhaustive Path Optimizer.
        public:
            //AdjacencyMatrix representing the connections. 
            std::vector<std::vector<int>> AdjacencyMatrix;
            //Represent the neighbours
            std::vector<std::vector<size_t>> AdjacencyGraph;
            PolygonGraph(const std::vector<size_t>& AdjacencyInfo, const std::vector<size_t>& PolygonIndices, const std::vector<double>& startPoses, const std::vector<double>& endPoses, const double TakeoffArr[], const double LandingArr[]);
            std::vector<CoveragePolygon> CoveragePolygons;
            Point3D Takeoff;
            Point3D Landing;
            int adjacencyCount(const std::vector<size_t>& areaPermutation);

        };
    }
}
#endif

