// Copyright 2022 The MathWorks, Inc.

// This file contains implementations for:
// - factor between an SE(3) Camera and an R3 point
//
// All implementations represent the poses as a single variable and the Jacobians
// are computed through ceres auto-diff.

#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <unsupported/Eigen/MatrixFunctions>

#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/cerescodegen_spec.hpp"
    #include "cerescodegen/group_utilities.hpp"
    #include "cerescodegen/custom_local_parameterization.hpp"
    #include "cerescodegen/factor.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy */
    #include "cerescodegen_spec.hpp"
    #include "group_utilities.hpp"
    #include "custom_local_parameterization.hpp"
    #include "factor.hpp"
#endif

#ifndef CAMERA_PROJECTION_FACTOR_HPP
#define CAMERA_PROJECTION_FACTOR_HPP
namespace mw_ceres {

     // Functor to compute the projection cost between SE3 Camera and R(3) point
    // The Jacobians are computed through Ceres auto-diff.
    class PinholeCameraSE3Point3ReprojectionCost {
     public:
        PinholeCameraSE3Point3ReprojectionCost(const std::vector<double>& observedPosition,
                          const std::vector<double>& information, const std::vector<double>& sensorTform ) {
            // m_Observed is the observed position of the projection point in camera frame
            m_ObservedXY << observedPosition[0], observedPosition[1];
            m_SqrtInformation = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>(information.data()).llt().matrixL();
            Eigen::Map<const Eigen::Matrix<double, 4, 4>> tform(sensorTform.data());
            m_SensorTransform_q =  tform.block<3,3>(0,0).transpose();
            m_SensorTransform_t = m_SensorTransform_q*tform.block<3,1>(0,3);
            focalL[0] = observedPosition[2];
            focalL[1] = observedPosition[3];
        }

        template <typename T>
        bool operator()(const T* const camera, const T* const pos, T* residuals) const {
            // here camera is the vehicle/robot's camera pose in reference frame,
            //      pos is the point position in reference frame

            // world position of camera
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(camera);
            // world orientation of camera
            Eigen::Map<const Eigen::Quaternion<T>> qi(camera + 3);
            // world position of landmark
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> xi(pos);

            // rotate and translate the point to bring it to  
            // camera frame i from world 
            auto qs = m_SensorTransform_q.cast<T>();
            Eigen::Matrix<T, 3, 1> p = qs*qi.inverse()* xi  - qs*qi.inverse()*pi + m_SensorTransform_t.cast<T>();
        
            // compute projected image point
            T predicted_x = p[0] / p[2] * focalL[0] ;
            T predicted_y = p[1] / p[2] * focalL[1];
        
            // The error is the difference between the predicted and observed position.
            T r_x = predicted_x - T(m_ObservedXY[0]);
            T r_y = predicted_y - T(m_ObservedXY[1]);

            residuals[0] = r_x*m_SqrtInformation(0,0) + r_y*m_SqrtInformation(1,0);
            residuals[1] = r_y*m_SqrtInformation(0,1) + r_y*m_SqrtInformation(1,1);

            return true;   
        }

    protected:
        Eigen::Vector2d m_ObservedXY;
        Eigen::Matrix<double, 2, 2> m_SqrtInformation;
        Eigen::Quaterniond m_SensorTransform_q;
        Eigen::Matrix<double, 3, 1> m_SensorTransform_t;

        double focalL[2];
    };

    // This class represents a factor that relates between an camera and an R(3) point
    // using PinholeCameraSE3Point3ReprojectionCost as residual cost function
    class CERESCODEGEN_API FactorCameraSE3AndPointXYZ : public FactorGaussianNoiseModel {
        public:
        FactorCameraSE3AndPointXYZ(std::vector<int> ids) : FactorGaussianNoiseModel(ids, {7, 3}, {VariableType::Pose_SE3, VariableType::Point_XYZ}) {
            m_MeasurementLength = 4;
            m_InfoMatLength = 4;
            m_Measurement = {0.0, 0.0, 0.0, 0.0};
            m_InformationMatrix = std::vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 2, 2>::Identity();
            m_SensorTransform = std::vector<double>(16, 0.0);
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> matS(m_SensorTransform.data());
            matS = Eigen::Matrix<double, 4, 4>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PinholeCameraSE3Point3ReprojectionCost, 2, 7, 3>(
                new PinholeCameraSE3Point3ReprojectionCost(m_Measurement, m_InformationMatrix, m_SensorTransform));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Point_XYZ)
                return new ceres::IdentityParameterization(3);
            else
                return new ceres::ProductParameterization( new ceres::IdentityParameterization(3),
                    new ceres::EigenQuaternionParameterization());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Point_XYZ)
                return {0.0, 0.0, 0.0};
            else
                return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(static_cast<VariableType>(getVariableType(variableID)));
        }

        bool setSensorTransform(const double* sensorTform) {
            m_SensorTransform = std::vector<double>(sensorTform, sensorTform+16);
            return true;
        }

        std::vector<double> m_SensorTransform;

    };

}

#endif // CAMERA_PROJECTION_FACTOR_HPP
