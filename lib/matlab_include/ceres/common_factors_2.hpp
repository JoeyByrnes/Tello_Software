// Copyright 2021-2022 The MathWorks, Inc.

// This file contains implementations for:
// - factor between two SE(3) poses
// - factor between an SE(3) pose and an R3 point
// - factor between two SE(2) poses
// - factor between an SE(2) pose and an R2 point
// - full-state prior factor on an SE(3) pose
// - simple GPS factor (a prior factor on SE(3) pose with position-only measurement)
// - general prior factor of dimension N (assuming identity local parameterization)
//
// All implementations represent the poses as a single variable and the Jacobians
// are computed through ceres auto-diff.

#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"
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

#ifndef COMMON_FACTORS_2_HPP
#define COMMON_FACTORS_2_HPP
namespace mw_ceres {

    // Functor to compute SE(2) prior cost
    class SE2PriorCostAD {
    public:
        SE2PriorCostAD(const std::vector<double>& measurement, const std::vector<double>& information) {
            // prior measurement: [x, y, theta] in reference frame 
            m_MeasuredPose = Eigen::Matrix<double, 3, 1>(measurement.data());
            m_SqrtInformation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose, T* residuals) const {
            Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> pos(pose);
            T theta = pose[2];
            res.template head<2>() = (pos - m_MeasuredPose.template head<2>().template cast<T>());
            T dTheta = wrapToPi<T>(theta - static_cast<T>(m_MeasuredPose(2)));
            res(2) = dTheta;

            res = m_SqrtInformation.template cast<T>()* res;

            return true;
        }

    protected:
        Eigen::Matrix<double, 3, 1> m_MeasuredPose;
        Eigen::Matrix<double, 3, 3> m_SqrtInformation;
    };

    // Represents a full-state prior factor for an SE(2) pose variable
    // using SE2PriorCostAD as residual cost
    class CERESCODEGEN_API FactorPoseSE2Prior : public FactorGaussianNoiseModel {
    public:
        FactorPoseSE2Prior(std::vector<int> ids) : FactorGaussianNoiseModel(ids, { 3 }, { VariableType::Pose_SE2 }) {
            m_MeasurementLength = 3;
            m_InfoMatLength = 9;
            m_Measurement = { 0.0, 0.0, 0.0 };
            m_InformationMatrix = vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 3, 3>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<SE2PriorCostAD, 3, 3>(
                new SE2PriorCostAD(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::ProductParameterization(new ceres::IdentityParameterization(2),
                                                      new ceres::AutoDiffLocalParameterization<mw_ceres::AngleOPlus, 1, 1>());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return {0.0, 0.0, 0.0};
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(VariableType::Pose_SE2);
        }
    };




    // Functor to compute SE(3) prior cost
    class SE3PriorCostAD {
      public:
        SE3PriorCostAD(const std::vector<double>& measurement, const std::vector<double>& information) {
            // measurement: [x, y, z,  qx, qy, qz, qw]
            m_MeasuredPose = Eigen::Matrix<double, 7, 1>(measurement.data());
            m_SqrtInformation = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose_i, T* residuals) const {

            Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos(pose_i);
            Eigen::Map<const Eigen::Quaternion<T>> q(pose_i + 3);
            Eigen::Matrix<T, 6, 1> r;
            r.template head<3>() = (pos - m_MeasuredPose.template head<3>().template cast<T>());
            Eigen::Quaternion<T> qMeasusred(static_cast<T>(m_MeasuredPose[6]),
                                            static_cast<T>(m_MeasuredPose[3]),
                                            static_cast<T>(m_MeasuredPose[4]),
                                            static_cast<T>(m_MeasuredPose[5]));
            r.template tail<3>() = static_cast<T>(2.0) * (qMeasusred.conjugate() * q).vec();
            res = m_SqrtInformation.template cast<T>() * r;

            return true;
        }

      protected:
        Eigen::Matrix<double, 7, 1> m_MeasuredPose;
        Eigen::Matrix<double, 6, 6> m_SqrtInformation;
    };

    // Represents a prior factor on the full state of an SE(3) pose variable
    // using SE3PriorCostAD as residual cost
    class CERESCODEGEN_API FactorPoseSE3Prior : public FactorGaussianNoiseModel {
    public:
        FactorPoseSE3Prior(std::vector<int> ids) : FactorGaussianNoiseModel(ids, { 7 }, { VariableType::Pose_SE3 }) {
            m_MeasurementLength = 7;
            m_InfoMatLength = 36;
            m_Measurement = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
            m_InformationMatrix = vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 6, 6>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction <SE3PriorCostAD, 6, 7>(new SE3PriorCostAD(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::ProductParameterization(new ceres::IdentityParameterization(3),
                                                      new ceres::EigenQuaternionParameterization());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(VariableType::Pose_SE3);
        }
    };


    // Functor to compute GPS-like SE(3) prior cost
    class GPSCostAD {
    public:
        GPSCostAD(const std::vector<double>& measurement, const std::vector<double>& information) {
            // measurement: [x, y, z]
            m_MeasuredPosition = Eigen::Matrix<double, 3, 1>(measurement.data());
            m_SqrtInformation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(information.data()).llt().matrixL(); 
        }

        template <typename T>
        bool operator()(const T* const pose_i, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos(pose_i);
            Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
            res = m_SqrtInformation.template cast<T>() * (pos - m_MeasuredPosition.template cast<T>());
            return true;
        }

    protected:
        Eigen::Matrix<double, 3, 1> m_MeasuredPosition;
        Eigen::Matrix<double, 3, 3> m_SqrtInformation;
    };

    // Represents (absolute, GPS-like) position-only prior on an SE3 pose variable
    // The measured position is assumed to have already been converted to local frame
    class CERESCODEGEN_API FactorSimpleGPS : public FactorGaussianNoiseModel {
      public:
        FactorSimpleGPS(std::vector<int> ids) : FactorGaussianNoiseModel(ids, {7}, { VariableType::Pose_SE3}) {
            m_MeasurementLength = 3;
            m_InfoMatLength = 9;
            m_Measurement = std::vector<double>(3, 0.0);
            m_InformationMatrix = vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat( m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 3, 3>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<GPSCostAD, 3, 7>(
                new GPSCostAD(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::ProductParameterization(new ceres::IdentityParameterization(3),
                                                      new ceres::EigenQuaternionParameterization());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            // GPS factor expects the variable to be POSE_SE3 ([x,y,z,qx,qy,qz,qw])
            return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        }
    };



    // The non-type template parameter N can be 1 - 6
    // Functor to compute general prior cost (assuming identity local parameterization)
    template <int N>
    class PriorCostAD {
    public:
        PriorCostAD(const std::vector<double>& measurement, const std::vector<double>& information) {
            m_MeasuredPos = Eigen::Matrix<double, N, 1>(measurement.data());
            m_SqrtInformation = Eigen::Matrix<double, N, N, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose_i, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, N, 1>> pos(pose_i);
            Eigen::Map<Eigen::Matrix<T, N, 1>> res(residuals);
            res = m_SqrtInformation.template cast<T>() * (pos - m_MeasuredPos.template cast<T>());
            return true;
        }
    protected:
        Eigen::Matrix<double, N, 1> m_MeasuredPos;
        Eigen::Matrix<double, N, N> m_SqrtInformation;
    };

    /// A factor that represents the 3D velocity prior
    class CERESCODEGEN_API FactorVel3Prior : public FactorGaussianNoiseModel {

    public:
        FactorVel3Prior(std::vector<int> ids) : FactorGaussianNoiseModel(ids, { 3 }, { VariableType::Vel_3 }) {
            m_MeasurementLength = 3;
            m_InfoMatLength = 9;
            m_Measurement = vector<double>(m_MeasurementLength, 0.0);
            m_InformationMatrix = vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 3, 3>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PriorCostAD<3>, 3, 3>(new PriorCostAD<3>(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::IdentityParameterization(3);
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return vector<double>(3, 0.0);
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(VariableType::Pose_SE3);
        }
    };

    /// A factor that represents the IMU Bias prior
    class CERESCODEGEN_API FactorIMUBiasPrior : public FactorGaussianNoiseModel {

    public:
        FactorIMUBiasPrior(std::vector<int> ids) : FactorGaussianNoiseModel(ids, { 6 }, { VariableType::IMU_Bias }) {
            m_MeasurementLength = 6;
            m_InfoMatLength = 36;
            m_Measurement = vector<double>(m_MeasurementLength, 0.0);
            m_InformationMatrix = vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 6, 6>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PriorCostAD<6>, 6, 6>(new PriorCostAD<6>(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::IdentityParameterization(6);
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return vector<double>(6, 0.0);
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(VariableType::IMU_Bias);
        }
    };



    // Functor to compute the residual cost between two SE(2) poses (each SE(2) pose is expressed as a single variable).
    // The Jacobians are computed through Ceres auto-diff.
    // This class is conceptually equivalent to "PoseSE2CostCompositeAD"
    class PoseSE2Cost {
    public:
        PoseSE2Cost(const std::vector<double>& measuredRelPose, const std::vector<double>& information) {
            // relative pose: [x, y, theta] (theta is the yaw angle in radians)
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1];
            m_MeasuredTheta = measuredRelPose[2];
            m_SqrtInformation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> pi(pose_i);
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> pj(pose_j);

            T thi = pose_i[2];
            T thj = pose_j[2];

            Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);

            // we need R(thi).transpose() here.
            Eigen::Matrix<T, 2, 2> Ri;
            Ri << ceres::cos(thi), -ceres::sin(thi), ceres::sin(thi), ceres::cos(thi); // must use ceres::cos and ceres::sin
            res.template head<2>() = Ri.transpose()* (pj - pi) - m_MeasuredPos.template cast<T>();
            T dTheta = wrapToPi<T>(thj - thi - static_cast<T>(m_MeasuredTheta) );
            res(2) = dTheta;

            res = m_SqrtInformation.template cast<T>() * res;
            return true;
        }
    protected:
        Eigen::Vector2d m_MeasuredPos;
        double m_MeasuredTheta;
        Eigen::Matrix<double, 3, 3> m_SqrtInformation;
    };

    // This class represents a factor that relates between two SE(2) poses (each pose is expressed
    // as a single variable) using PoseSE2Cost (i.e. PoseSE2CostCompositeAD) as residual cost function
    // This class is conceptually equivalent to "FactorTwoPosesSE2CompositeAD"
    class CERESCODEGEN_API FactorTwoPosesSE2 : public FactorGaussianNoiseModel {
    public:
        FactorTwoPosesSE2(std::vector<int> ids) : FactorGaussianNoiseModel(ids, {3, 3}, {VariableType::Pose_SE2, VariableType::Pose_SE2}) {
            m_MeasurementLength = 3;
            m_InfoMatLength = 9;
            m_Measurement = {0.0, 0.0, 0.0};
            m_InformationMatrix = std::vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat( m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 3, 3>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE2Cost, 3, 3, 3>(
                new PoseSE2Cost(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::ProductParameterization( new ceres::IdentityParameterization(2),
                new ceres::AutoDiffLocalParameterization<mw_ceres::AngleOPlus, 1, 1>());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return {0.0, 0.0, 0.0};
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(VariableType::Pose_SE2);
        }
    };



    // Functor to compute the residual cost between an SE(2) pose and a R(2) point 
    // The Jacobians are computed through Ceres auto-diff.
    class PoseSE2Point2Cost {
    public:
        PoseSE2Point2Cost(const std::vector<double>& measuredRelPosition, const std::vector<double>& information) {
            m_MeasuredPos << measuredRelPosition[0], measuredRelPosition[1];
            m_SqrtInformation = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose, const T* const pos, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> p1(pose);
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> p2(pos);
            T theta1 = pose[2];
            T sinTheta1 = ceres::sin(theta1);
            T cosTheta1 = ceres::cos(theta1);
            Eigen::Matrix<T, 2, 2> RT;
            RT << cosTheta1, sinTheta1, -sinTheta1, cosTheta1;

            Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residuals);
            res = m_SqrtInformation.template cast<T>() * (RT * (p2 - p1) - m_MeasuredPos.template cast<T>());

            return true;
        }

    protected:
        Eigen::Vector2d m_MeasuredPos;
        Eigen::Matrix<double, 2, 2> m_SqrtInformation;
    };

    // This class represents a factor that relates between an SE(2) pose and an R(2) point
    // using PoseSE2Point2Cost as residual cost function
    class CERESCODEGEN_API FactorPoseSE2AndPoint2 : public FactorGaussianNoiseModel {
    public:
        FactorPoseSE2AndPoint2(std::vector<int> ids) : FactorGaussianNoiseModel(ids, {3, 2}, {VariableType::Pose_SE2, VariableType::Point_XY}) {
            m_MeasurementLength = 2;
            m_InfoMatLength = 4;
            m_Measurement = {0.0, 0.0};
            m_InformationMatrix = std::vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> mat( m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 2, 2>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE2Point2Cost, 2, 3, 2>(
                new PoseSE2Point2Cost(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Point_XY)
                return new ceres::IdentityParameterization(2);
            else
                return new ceres::ProductParameterization( new ceres::IdentityParameterization(2),
                    new ceres::AutoDiffLocalParameterization<mw_ceres::AngleOPlus, 1, 1>());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Point_XY)
                return {0.0, 0.0};
            else
                return {0.0, 0.0, 0.0};
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(static_cast<VariableType>(getVariableType(variableID)));
        }

    };



    // Functor to compute the residual cost between two SE(3) poses (each SE(3) pose is represented by a single variable).
    // The Jacobians are computed through Ceres auto-diff.
    // This class is conceptually equivalent to "PoseSE3CostCompositeAD"
    class PoseSE3Cost {
    public:
        PoseSE3Cost(const vector<double>& measuredRelPose, const vector<double>& information) {
            // relative pose: [x, y, z,  qx, qy, qz, qw]
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1], measuredRelPose[2];
            m_MeasuredQuat = Eigen::Quaterniond(measuredRelPose.data() + 3).normalized();
            // assuming information matrix is stored row-major
            m_SqrtInformation = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(pose_i);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pj(pose_j);

            Eigen::Map<const Eigen::Quaternion<T>> qi(pose_i + 3);
            Eigen::Map<const Eigen::Quaternion<T>> qj(pose_j + 3);

            Eigen::Quaternion<T> q = qi.conjugate() * qj;
            Eigen::Quaternion<T> dq = m_MeasuredQuat.template cast<T>() * q.conjugate();

            Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
            // Eigen quaternion has already overloaded the * operator when the input it's a 3d
            // vector. q*v is equivalent to qvq'
            res.template block<3, 1>(0, 0) = qi.conjugate() * (pj - pi) - m_MeasuredPos.template cast<T>();
            res.template block<3, 1>(3, 0) = T(2.0) * dq.vec();
            res.applyOnTheLeft(m_SqrtInformation.template cast<T>());

            return true;
        }

    protected:
        Eigen::Vector3d m_MeasuredPos;
        Eigen::Quaterniond m_MeasuredQuat;
        Eigen::Matrix<double, 6, 6> m_SqrtInformation;
    };

    // This class represents a factor that relates between two SE(3) poses (each pose is represented by a single variable)
    // using PoseSE3Cost (i.e. PoseSE3CostCompositeAD) as residual cost function.
    // This class is conceptually equivalent to "FactorTwoPosesSE3CompositeAD"
    class CERESCODEGEN_API FactorTwoPosesSE3 : public FactorGaussianNoiseModel {
    public:
        FactorTwoPosesSE3(std::vector<int> ids) : FactorGaussianNoiseModel(ids,  {7, 7}, {VariableType::Pose_SE3, VariableType::Pose_SE3}) {
            m_MeasurementLength = 7;
            m_InfoMatLength = 36;
            m_Measurement = {0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0};
            m_InformationMatrix = std::vector<double>(36, 0.0);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 6, 6>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE3Cost, 6, 7, 7>(
                new PoseSE3Cost(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new ceres::ProductParameterization(new ceres::IdentityParameterization(3),
                                                      new ceres::EigenQuaternionParameterization());
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        }

        std::string getVariableTypeString(int variableID) const override {
            return VariableTypeString.at(VariableType::Pose_SE3);
        }
    };



    // Functor to compute the residual cost between an SE(3) pose and a R(3) point
    // The Jacobians are computed through Ceres auto-diff.
    class PoseSE3Point3Cost {
     public:
        PoseSE3Point3Cost(const std::vector<double>& measuredRelPosition,
                          const std::vector<double>& information) {
            // m_MeasuredPos is the observed relative position of the landmark point (relative to the vehicle/robot frame)
            m_MeasuredPos << measuredRelPosition[0], measuredRelPosition[1], measuredRelPosition[2];
            m_SqrtInformation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pose, const T* const pos, T* residuals) const {
            // here pose is the vehicle/robot's pose in reference frame,
            //      pos is the point/landmark position in reference frame

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(pose);
            Eigen::Map<const Eigen::Quaternion<T>> q1(pose + 3);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p2(pos);

            Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
            // Eigen quaternion has already overloaded the * operator when the input it's a 3d
            // vector. q*v is equivalent to qvq'
            res = q1.conjugate() * (p2 - p1) - m_MeasuredPos.template cast<T>();

            res.applyOnTheLeft(m_SqrtInformation.template cast<T>());

            return true;
        }

    protected:
        Eigen::Vector3d m_MeasuredPos;
        Eigen::Matrix<double, 3, 3> m_SqrtInformation;
    };

    // This class represents a factor that relates between an SE(3) pose and an R(3) point
    // using PoseSE2Point3Cost as residual cost function
    class CERESCODEGEN_API FactorPoseSE3AndPoint3 : public FactorGaussianNoiseModel {
        public:
        FactorPoseSE3AndPoint3(std::vector<int> ids) : FactorGaussianNoiseModel(ids, {7, 3}, {VariableType::Pose_SE3, VariableType::Point_XYZ}) {
            m_MeasurementLength = 3;
            m_InfoMatLength = 9;
            m_Measurement = {0.0, 0.0, 0.0};
            m_InformationMatrix = std::vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat( m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 3, 3>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE3Point3Cost, 3, 7, 3>(
                new PoseSE3Point3Cost(m_Measurement, m_InformationMatrix));
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

    };



}

#endif // COMMON_FACTORS_2_HPP
