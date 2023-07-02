// Copyright 2021-2022 The MathWorks, Inc.

#include <vector>
#include <unordered_map>
#include "ceres/ceres.h"

#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/cerescodegen_spec.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy */
    #include "cerescodegen_spec.hpp"
#endif

#ifndef FACTOR_HPP
#define FACTOR_HPP
namespace mw_ceres {
    // variable type enumerations
    enum VariableType {
        Pose_SE3,
        Pose_SE2,
        Point_XY,
        Point_XYZ,
        Vel_3,
        IMU_Bias,
        Pseudo_Pose_SE3,
        Orientation_SO3,
        Orientation_SO3_AD,
        Eigen_Quaternion,
        Angle,
        Pose_SE3_Scale
    };

    // user-facing node type to node enum
    static std::unordered_map<std::string, int> NodeTypeInt{
        {"POSE_SE3", 0}, {"POSE_SE2", 1},
        {"POINT_XY", 2}, {"POINT_XYZ", 3},
        {"VEL3", 4}, {"IMU_BIAS", 5},
        {"Eigen_Quaternion", 6}, {"Pose_SE3_Scale", 7}};

    // user-facing factor type to internal factor type
    static std::unordered_map<std::string, int> FactorTypeInt{
        {"factorTwoPoseSE2", 0},
        {"factorTwoPoseSE3", 1},
        {"factorPoseSE2AndPointXY", 2},
        {"factorPoseSE3AndPointXYZ", 3},
        {"factorIMU", 4},
        {"factorGPS", 5},
        {"factorPoseSE2Prior", 6},
        {"factorPoseSE3Prior", 7},
        {"factorIMUBiasPrior", 8},
        {"factorVelocity3Prior", 9},
        {"factorCameraSE3AndPointXYZ", 10}
    };

    // user-facing variable type string
    static std::unordered_map<VariableType, std::string> VariableTypeString{
        {VariableType::Pose_SE3, "POSE_SE3"}, {VariableType::Pose_SE2, "POSE_SE2"},
        {VariableType::Point_XY, "POINT_XY"}, {VariableType::Point_XYZ, "POINT_XYZ"},
        {VariableType::Vel_3, "VEL3"},       {VariableType::IMU_Bias, "IMU_BIAS"},
        {VariableType::Pose_SE3_Scale, "POSE_SE3_SCALE"}, 
        {VariableType::Eigen_Quaternion, "EIGEN_QUATERNION"}};


    class CERESCODEGEN_API Factor {

    protected:
        /// Variable IDs involved in this factor
        std::vector<int> m_VariableIDs;

        /// Map from variable ID to variable state dimension
        std::unordered_map<int, int> m_MapVariableDims;
        
        /// Map from variable ID to variable local parameterization type
        std::unordered_map<int, int> m_MapVariableLPTypes;

        // only accessible to derived classes
        Factor(std::vector<int> ids, std::vector<int> dims, std::vector<int> varTypes)
            : m_VariableIDs(ids) {
            assert(ids.size() == dims.size());
            assert(ids.size() == varTypes.size());
            for (size_t i = 0; i < ids.size(); ++i) {
                m_MapVariableDims[ids[i]] = dims[i];
                m_MapVariableLPTypes[ids[i]] = varTypes[i];
            }
        }


        Factor() { }

    public:
        virtual ceres::CostFunction* createFactorCostFcn() const = 0;
        virtual ceres::LocalParameterization* getVariableLocalParameterization(int variableID) = 0;
        virtual std::vector<double> getDefaultState(int variableID) const = 0;
        virtual std::string getVariableTypeString(int ) const {
            return "N/A";
        }

        /// This method is optional, use it only when the factor's internal data needs to be updated based on the current variable states
        virtual void preOptimizationUpdate(std::vector<double*>& ) {}

        virtual ~Factor() {}
        std::vector<int> getVariableIDs() const {
            return m_VariableIDs;
        }
        int getVariableDim(int variableID) const {
            auto it = m_MapVariableDims.find(variableID);
            if (it == m_MapVariableDims.end())
                return -1;
            return it->second;
        }
        int getVariableType(int variableID) const {
            auto it = m_MapVariableLPTypes.find(variableID);
            if (it == m_MapVariableLPTypes.end())
                return -1;
            return it->second;
        }
    };

    /// <summary>
    ///  Base class for non-sensor-specific factors whose measurements follow a zero-mean Gaussian noise model.
    ///  The factor can involve one ore multiple variable. 
    /// </summary>
    class CERESCODEGEN_API FactorGaussianNoiseModel : public Factor {
    public:
        FactorGaussianNoiseModel(std::vector<int> ids, std::vector<int> dims, std::vector<int> types)
            : Factor(ids, dims, types) {
            m_MeasurementLength = 0;
            m_InfoMatLength = 0;
        }

        bool setMeasurement(const double* meas) {
            m_Measurement = std::vector<double>(meas, meas+m_MeasurementLength);
            return true;
        }

        bool setInformation(const double* infoMat) {
            m_InformationMatrix = std::vector<double>(infoMat, infoMat+m_InfoMatLength);
            return true;
        }

        std::vector<double> getMeasurement() {
            return m_Measurement;
        }

        std::vector<double> getInformation() {
            return m_InformationMatrix;
        }

        const size_t getInformationLength() {
            return m_InfoMatLength;
        }

    protected:
        size_t m_MeasurementLength;
        size_t m_InfoMatLength;
        std::vector<double> m_Measurement;
        std::vector<double> m_InformationMatrix;
    };
}
#endif // FACTOR_HPP
