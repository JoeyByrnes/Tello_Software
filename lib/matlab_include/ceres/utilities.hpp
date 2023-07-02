// Copyright 2021 The MathWorks, Inc.
#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>

namespace mw_ceres {

    namespace g2o {
        struct VertexSE3 {
            VertexSE3() {
                m_Data = std::vector<double>(7, 0);
                m_ID = 0;
            }
            std::vector<double> m_Data;
            int m_ID;
            static std::string type() { return "VERTEX_SE3:QUAT"; }
        };

        std::istream& operator>>(std::istream& input, VertexSE3& v);

        struct VertexPointXYZ {
            VertexPointXYZ() {
                m_Data = std::vector<double>(3, 0);
                m_ID = 0;
            }
            std::vector<double> m_Data;
            int m_ID;
            static std::string type() {
                return "VERTEX_TRACKXYZ";
            }
        };

        std::istream& operator>>(std::istream& input, VertexPointXYZ& v);

        struct EdgeSE3 {
            EdgeSE3() {
                m_Data = std::vector<double>(7, 0);
                m_Info = std::vector<double>(36,0);
                m_IDs = std::vector<int>(2, 0);
            }
            std::vector<double> m_Data;
            std::vector<double> m_Info;
            std::vector<int> m_IDs;
            static std::string type() { return "EDGE_SE3:QUAT"; }
        };

        std::istream& operator >> (std::istream& input, EdgeSE3& e);

        struct EdgeSE3PointXYZ {
            EdgeSE3PointXYZ() {
                m_Data = std::vector<double>(3, 0);
                m_IDs = std::vector<int>(2, 0);
                m_Info = std::vector<double>(9, 0);
            }
            std::vector<double> m_Data;
            std::vector<double> m_Info;
            std::vector<int> m_IDs;
            static std::string type() {
                return "EDGE_SE3_TRACKXYZ";
            }
        };

        std::istream& operator>>(std::istream& input, EdgeSE3PointXYZ& e);

        struct VertexSE2 {
            VertexSE2() {
                m_Data = std::vector<double>(3, 0);
                m_ID = 0;
            }
            std::vector<double> m_Data;
            int m_ID;
            static std::string type() {
                return "VERTEX_SE2";
            }
        };

        std::istream& operator>>(std::istream& input, VertexSE2& v);

        struct VertexPointXY {
            VertexPointXY() {
                m_Data = std::vector<double>(2, 0);
                m_ID = 0;
            }
            std::vector<double> m_Data;
            int m_ID;
            static std::string type() {
                return "VERTEX_PointXY";
            }
        };

        std::istream& operator>>(std::istream& input, VertexPointXY& v);

        struct EdgeSE2 {
            EdgeSE2() {
                m_Data = std::vector<double>(3, 0);
                m_Info = std::vector<double>(9, 0);
                m_IDs = std::vector<int>(2, 0);
            }
            std::vector<double> m_Data;
            std::vector<double> m_Info;
            std::vector<int> m_IDs;
            static std::string type() {
                return "EDGE_SE2";
            }
        };

        std::istream& operator>>(std::istream& input, EdgeSE2& e);

        struct EdgeSE2PointXY {
            EdgeSE2PointXY() {
                m_Data = std::vector<double>(3, 0);
                m_Info = std::vector<double>(4, 0);
                m_IDs = std::vector<int>(2, 0);
            }
            std::vector<double> m_Data;
            std::vector<double> m_Info;
            std::vector<int> m_IDs;
            static std::string type() {
                return "EDGE_SE2_TRACKXY";
            }
        };

        std::istream& operator>>(std::istream& input, EdgeSE2PointXY& e);

        template <typename VertexPose, typename ConstraintPose, typename VertexPoint, typename ConstraintPosePoint>
        bool read_g2o(const std::string& fileName, 
            std::vector<VertexPose>& vertices1,
            std::vector<VertexPoint>& vertices2,
            std::vector<ConstraintPose>& constraints1,
            std::vector<ConstraintPosePoint>& constraints2) {

            std::ifstream inputFile(fileName.c_str(), std::ifstream::in);
            if (!inputFile)
                return false;

            std::string dataType;

            while (inputFile.good()) {
                dataType = "";
                inputFile >> dataType;
                if (dataType == VertexPose::type()) {
                    VertexPose v;
                    inputFile >> v;
                    vertices1.push_back(v);
                }
                else if (dataType == VertexPoint::type()) {
                    VertexPoint v;
                    inputFile >> v;
                    vertices2.push_back(v);
                }
                else if (dataType == ConstraintPose::type()) {
                    ConstraintPose constr;
                    inputFile >> constr;
                    constraints1.push_back(constr);
                }
                else if (dataType == ConstraintPosePoint::type()) {
                    ConstraintPosePoint constr;
                    inputFile >> constr;
                    constraints2.push_back(constr);
                }
                //else {
                //    return false;
                //}
            }
            //inputFile.close();
            return true;
        }
    }
}

#endif // UTILITY_HPP
