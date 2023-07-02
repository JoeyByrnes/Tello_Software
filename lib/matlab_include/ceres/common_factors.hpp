// Copyright 2021-2022 The MathWorks, Inc.

// alternative implementations for factors between two SE(3) poses and 
// factors between two SE(2) poses

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

#ifndef COMMON_FACTORS_HPP
#define COMMON_FACTORS_HPP
namespace mw_ceres {

    /// A class that provides function to compute the residual error between two SE(3) poses,
    /// expressed as a single composite vector, respectively (i.e. [position, orientation]).
    /// The Jacobians are computed analytically using first-order approximation
    class PoseSE3CostComposite: public ceres::SizedCostFunction<6, 7, 7> {// 6 - residual dimension, 7 - param1 dimension, 7, param2 dimension
    public:
        PoseSE3CostComposite(const std::vector<double>& measuredRelPose, const std::vector<double>& information) {
            // relative pose is given as [x, y, z,  qx, qy, qz, qw]
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1], measuredRelPose[2];
            m_MeasuredRot = Eigen::Quaterniond(measuredRelPose.data() + 3).normalized().toRotationMatrix();
            // assuming information (vector) is stored in row-major order
            m_SqrtInformation = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        bool Evaluate(double const* const* parameters,
            double* residuals,
            double** jacobians) const override {
            Eigen::Vector3d pos_i(parameters[0]);
            Eigen::Matrix3d rot_i = Eigen::Quaterniond(parameters[0] + 3).normalized().toRotationMatrix();
            Eigen::Vector3d pos_j(parameters[1]);
            Eigen::Matrix3d rot_j = Eigen::Quaterniond(parameters[1] + 3).normalized().toRotationMatrix();

            Eigen::Map<Eigen::Matrix<double, 6, 1>> residualsVec(residuals);
            Eigen::Matrix3d rot = m_MeasuredRot.transpose() * rot_i.transpose() * rot_j;
            residualsVec.head(3) = rot_i.transpose() * (pos_j - pos_i) - m_MeasuredPos;
            residualsVec.tail(3) = mw_ceres::SO3::logm(rot);
            residualsVec.applyOnTheLeft(m_SqrtInformation);

            if (jacobians) {
                if (jacobians[0]) {
                    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jac_i(jacobians[0]); // Jacobian w.r.t. to pose i
                    // Jac_i:
                    //     [ ddeltaP/dpos_i, ddeltaP/dphi_i]
                    //     [ ddeltaR/dpos_i, ddeltaR/dphi_i]
                    jac_i.block<3, 3>(3, 3) = -mw_ceres::SO3::rightJacobianLogm(residualsVec.tail(3)) * rot_j.transpose() * rot_i; // ddeltaR/dphi_i
                    jac_i.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(); // ddeltaR/dpos_i
                    Eigen::Vector3d tmp = rot_i.transpose() * (pos_j - pos_i);
                    jac_i.block<3, 3>(0, 3) = mw_ceres::SO3::hat<double>(tmp); // ddeltaP / dphi_i
                    jac_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();    // ddeltaP / dpos_i
                    jac_i.col(6).setZero();
                    jac_i.applyOnTheLeft(m_SqrtInformation);
                }

                if (jacobians[1]) {
                    // Jac_j:
                    //     [ ddeltaP/dpos_j, ddeltaP/dphi_j]
                    //     [ ddeltaR/dpos_j, ddeltaR/dphi_j]
                    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jac_j(jacobians[1]); // Jacobian w.r.t. to pose j
                    jac_j.block<3, 3>(3, 3) =
                        mw_ceres::SO3::rightJacobianLogm(residualsVec.tail(3)); // ddeltaR/dphi_j
                    jac_j.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();          // ddeltaR/dpos_j
                    jac_j.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();          // ddeltaP/dphi_j
                    jac_j.block<3, 3>(0, 0) = rot_i.transpose() * rot_j;        // ddeltaP/dpos_j
                    jac_j.col(6).setZero();
                    jac_j.applyOnTheLeft(m_SqrtInformation);
                }
            }
            return true;
        }

    protected:
        Eigen::Vector3d m_MeasuredPos;
        Eigen::Matrix3d m_MeasuredRot;
        Eigen::Matrix<double, 6, 6> m_SqrtInformation;
    };

    /// A factor that relates between two SE(3) poses (each pose is expressed as a single composite vector)
    /// using PoseSE3CostComposite as residual cost function
    class FactorTwoPoseSE3Composite : public FactorGaussianNoiseModel {
    public:
        FactorTwoPoseSE3Composite(std::vector<int> ids) : FactorGaussianNoiseModel(ids, { 7,7 }, {VariableType::Pseudo_Pose_SE3, VariableType::Pseudo_Pose_SE3}) {
            m_MeasurementLength = 7;
            m_InfoMatLength = 36;
            m_Measurement = { 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0 };
            m_InformationMatrix = std::vector<double>(36, 0.0);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 6, 6>::Identity();
        };

        ceres::CostFunction* createFactorCostFcn() const override {
            return new PoseSE3CostComposite(m_Measurement, m_InformationMatrix);
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            return new mw_ceres::PseudoSE3LocalParameterization();
        }

        std::vector<double> getDefaultState(int variableID) const override {
            return { 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0 };
        }
    };



    /// Functor to compute the residual cost between two SE(3) poses (expressed as split variables)
    /// based on quaternion formulation. The Jacobians are computed through auto-diff.
    class PoseSE3CostSplitAD {
      public:
        PoseSE3CostSplitAD(const vector<double>& measuredRelPose, const vector<double>& information) {
            // relative pose: [x, y, z,  qx, qy, qz, qw]
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1], measuredRelPose[2];
            m_MeasuredQuat = Eigen::Quaterniond(measuredRelPose.data() + 3).normalized();
            m_MeasuredRot = m_MeasuredQuat.toRotationMatrix();
            // assuming information matrix is stored row-major
            m_SqrtInformation = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pos_i,
                        const T* const quat_i,
                        const T* const pos_j,
                        const T* const quat_j,
                        T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(pos_i);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pj(pos_j);

            Eigen::Map<const Eigen::Quaternion<T>> qi(quat_i);
            Eigen::Map<const Eigen::Quaternion<T>> qj(quat_j);

            // compute the difference between the estimated orientation change and the measured
            // orientation change
            Eigen::Quaternion<T> q = qi.conjugate() * qj;
            Eigen::Quaternion<T> dq = m_MeasuredQuat.template cast<T>() * q.conjugate();
            // also possible
            // Eigen::Quaternion<T> dq = m_MeasuredQuat.template cast<T>().conjugate() * q;

            Eigen::Matrix<T, 3, 3> Ri = qi.normalized().toRotationMatrix();
            Eigen::Matrix<T, 3, 3> Rj = qj.normalized().toRotationMatrix();

            Eigen::Matrix<T, 3, 3> R = (m_MeasuredRot.transpose().template cast<T>()) * Ri.transpose() * Rj;

            Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);
            // Eigen quaternion has already overloaded the * operator when the input it's a 3d
            // vector. q*v is equivalent to qvq'
            res.template block<3, 1>(0, 0) = qi.conjugate() * (pj - pi) - m_MeasuredPos.template cast<T>();

            res.template block<3, 1>(3, 0) = T(2.0) * dq.vec();
            res.applyOnTheLeft(m_SqrtInformation.template cast<T>());
            return true;
        };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      protected:
        Eigen::Vector3d m_MeasuredPos;
        Eigen::Matrix3d m_MeasuredRot;
        Eigen::Quaterniond m_MeasuredQuat;
        Eigen::Matrix<double, 6, 6> m_SqrtInformation;
    };

    /// A factor that relates between two SE(3) poses (expressed as split variables)
    /// using PoseSE3CostSplitAD as residual cost function
    class FactorTwoPosesSE3SplitAD : public FactorGaussianNoiseModel {

      public:
        FactorTwoPosesSE3SplitAD(std::vector<int> ids) : FactorGaussianNoiseModel(ids,  {3, 4, 3, 4},
                  {VariableType::Point_XYZ, VariableType::Eigen_Quaternion,
                   VariableType::Point_XYZ, VariableType::Eigen_Quaternion}) {
            m_MeasurementLength = 7;
            m_InfoMatLength = 36;
            m_Measurement = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
            m_InformationMatrix = std::vector<double>(36, 0.0);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 6, 6>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE3CostSplitAD, 6, 3, 4, 3, 4>(
                new PoseSE3CostSplitAD(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Eigen_Quaternion)
                return new ceres::EigenQuaternionParameterization();
            else
                return new ceres::IdentityParameterization(3);
        }

        std::vector<double> getDefaultState(int variableID) const override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Eigen_Quaternion)
                return {0.0, 0.0, 0.0, 1.0};
            else
                return {0.0, 0.0, 0.0};
        }
    };



    /// Alternative functor to compute the residual cost between two SE(3) poses (expressed as split
    /// variables) based on matrix exponentials. The Jacobians are computed through auto-diff.
    class PoseSE3CostSplitAD_Alt {
      public:
        PoseSE3CostSplitAD_Alt(const vector<double>& measuredRelPose, const vector<double>& information) {
            // relative pose comes in as [x, y, z,  qx, qy, qz, qw]
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1], measuredRelPose[2];
            m_MeasuredRot = Eigen::Quaterniond(measuredRelPose.data() + 3).normalized().toRotationMatrix();
            // assuming information matrix is stored row-major
            m_SqrtInformation = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(information.data()).llt().matrixL();
        }

        template <typename T>
        bool operator()(const T* const pos_i,
                        const T* const quat_i,
                        const T* const pos_j,
                        const T* const quat_j,
                        T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(pos_i);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pj(pos_j);

            Eigen::Map<const Eigen::Quaternion<T>> qi(quat_i);
            Eigen::Map<const Eigen::Quaternion<T>> qj(quat_j);

            Eigen::Matrix<T, 3, 3> Ri = qi.normalized().toRotationMatrix();
            Eigen::Matrix<T, 3, 3> Rj = qj.normalized().toRotationMatrix();

            Eigen::Matrix<T, 3, 3> R = (m_MeasuredRot.transpose().template cast<T>()) * Ri.transpose() * Rj;

            Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);

            res.template block<3, 1>(0, 0) = Ri.transpose() * (pj - pi) - m_MeasuredPos.template cast<T>();
            res.template block<3, 1>(3, 0) = mw_ceres::SO3::logm<T>(R);
            res.applyOnTheLeft(m_SqrtInformation.template cast<T>());
            return true;
        };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      protected:
        Eigen::Vector3d m_MeasuredPos;
        Eigen::Matrix3d m_MeasuredRot;
        Eigen::Matrix<double, 6, 6> m_SqrtInformation;
    };

    /// A factor that relates between two SE(3) poses (expressed as split variables)
    /// using PoseSE3CostSplitAD_Alt as residual cost function
    class FactorTwoPosesSE3SplitAD_Alt : public FactorTwoPosesSE3SplitAD {
    public:
        FactorTwoPosesSE3SplitAD_Alt(std::vector<int> ids) : FactorTwoPosesSE3SplitAD(ids) {
            m_MapVariableLPTypes = {{ids[0], VariableType::Point_XYZ},
                                          {ids[1], VariableType::Orientation_SO3_AD},
                                          {ids[2], VariableType::Point_XYZ},
                                          {ids[3], VariableType::Orientation_SO3_AD}};
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE3CostSplitAD_Alt, 6, 3, 4, 3, 4>(
                new PoseSE3CostSplitAD_Alt(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Orientation_SO3_AD)
                return new ceres::AutoDiffLocalParameterization<SO3_OPlus, 4, 3>();
            else
                return new ceres::IdentityParameterization(3);
        }
    };



    /// A class that provides function to compute the residual error between two SE(3) poses.
    /// Each SE(3) pose is expressed as two split variables (i.e. one for position one for orientation quaternion).
    /// The Jacobians are computed analytically using first-order approximation
    class PoseSE3CostSplit : public ceres::SizedCostFunction<6, 3, 4, 3, 4> {
    public:
        PoseSE3CostSplit(const std::vector<double>& measuredRelPose, const std::vector<double>& information) {
            // relative pose: [x, y, z, qx, qy, qz, qw]
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1], measuredRelPose[2];
            m_MeasuredRot = Eigen::Quaterniond(measuredRelPose.data() + 3).normalized().toRotationMatrix();
            // assuming information matrix is stored row-major
            m_SqrtInformation = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(information.data()).sqrt();
        }

        bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
            Eigen::Vector3d pos_i(parameters[0]);
            Eigen::Matrix3d rot_i = Eigen::Quaterniond(parameters[1]).normalized().toRotationMatrix();
            Eigen::Vector3d pos_j(parameters[2]);
            Eigen::Matrix3d rot_j = Eigen::Quaterniond(parameters[3]).normalized().toRotationMatrix();
            
            Eigen::Map<Eigen::Matrix<double, 6, 1>> residualsVec(residuals);
            Eigen::Matrix3d rot = m_MeasuredRot.transpose() * rot_i.transpose() * rot_j;
            residualsVec.head(3) = rot_i.transpose() * (pos_j - pos_i) - m_MeasuredPos;
            residualsVec.tail(3) = mw_ceres::SO3::logm(rot);
            residualsVec.applyOnTheLeft(m_SqrtInformation);

            if (jacobians) {
                if (jacobians[0]) { // Jacobian w.r.t. to pos_i
                    Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_pos_i(jacobians[0]);
                    // jac_pos_i:
                    //     [ ddeltaP/dpos_i]
                    //     [ ddeltaR/dpos_i]
                    
                    jac_pos_i.block<3, 3>(0, 0) = -rot_i.transpose();              // ddeltaP / dpos_i
                    jac_pos_i.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();         // ddeltaR / dpos_i
                    
                    jac_pos_i.applyOnTheLeft(m_SqrtInformation);
                }

                if (jacobians[1]) { // Jacobian w.r.t. to quat_i
                    Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_quat_i(jacobians[1]);
                    // Jac_quat_i:
                    //     [ ddeltaP/dphi_i]
                    //     [ ddeltaR/dphi_i]
                    Eigen::Vector3d tmp = rot_i.transpose() * (pos_j - pos_i);
                    jac_quat_i.block<3, 3>(0, 0) = mw_ceres::SO3::hat<double>(tmp); // ddeltaP / dphi_i
                    jac_quat_i.block<3, 3>(3, 0) = -mw_ceres::SO3::rightJacobianLogm(residualsVec.tail(3))
                                             * rot_j.transpose() * rot_i;           // ddeltaR / dphi_i
                    jac_quat_i.col(3).setZero();
                    jac_quat_i.applyOnTheLeft(m_SqrtInformation);
                }

                if (jacobians[2]) { // Jacobian w.r.t. to pos_j
                    Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_pos_j(jacobians[2]);
                    // Jac_pos_j:
                    //     [ ddeltaP/dpos_j]
                    //     [ ddeltaR/dpos_j]
                    jac_pos_j.block<3, 3>(0, 0) = rot_i.transpose();                // ddeltaP / dpos_j
                    jac_pos_j.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();          // ddeltaR / dpos_j

                    jac_pos_j.applyOnTheLeft(m_SqrtInformation);
                }
                if (jacobians[3]) { // Jacobian w.r.t. to quat_j
                    Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_quat_j(jacobians[3]);
                    jac_quat_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();         // ddeltaP / dphi_j
                    jac_quat_j.block<3, 3>(3, 0) = mw_ceres::SO3::rightJacobianLogm(residualsVec.tail(3)); // ddeltaR/dphi_j
                    jac_quat_j.col(3).setZero();
                    jac_quat_j.applyOnTheLeft(m_SqrtInformation);
                }
            }
            return true;
        }

    protected:
        Eigen::Vector3d m_MeasuredPos;
        Eigen::Matrix3d m_MeasuredRot;
        Eigen::Matrix<double, 6, 6> m_SqrtInformation;
    };

    /// A factor that relates between two SE(3) poses (each pose is expressed as two split variables)
    /// using PoseSE3CostSplit as residual cost function
    class FactorTwoPosesSE3Split : public FactorGaussianNoiseModel {
    public:
        FactorTwoPosesSE3Split(std::vector<int> ids) : FactorGaussianNoiseModel(ids, { 3, 4, 3, 4 },
                {VariableType::Point_XYZ, VariableType::Orientation_SO3, VariableType::Point_XYZ, VariableType::Orientation_SO3}) {
            m_MeasurementLength = 7;
            m_InfoMatLength = 36;
            m_Measurement = { 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0 };
            m_InformationMatrix = std::vector<double>(36, 0.0);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> mat(m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 6, 6>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new PoseSE3CostSplit(m_Measurement, m_InformationMatrix);
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Orientation_SO3 )
                return new mw_ceres::SO3LocalParameterization();
            else
                return new ceres::IdentityParameterization(3);
        }

        std::vector<double> getDefaultState(int variableID) const override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Orientation_SO3)
                return { 0.0, 0.0, 0.0, 1.0};
            else
                return { 0.0, 0.0, 0.0 };
        }
    };



    /// Functor to compute the residual cost between two SE(2) poses (each SE(2) pose is expressed as two split variables).
    /// The Jacobians are computed through auto-diff.
    class PoseSE2CostSplitAD {
    public:
        PoseSE2CostSplitAD(const std::vector<double>& measuredRelPose, const std::vector<double>& information) {
            // relative pose: [x, y, theta] (theta is the yaw angle in radians)
            m_MeasuredPos << measuredRelPose[0], measuredRelPose[1];
            m_MeasuredTheta = measuredRelPose[2];
            m_SqrtInformation = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(information.data()).llt().matrixL(); //sqrt();
        }

        template <typename T>
        bool operator()(const T* const pos_i, const T* const theta_i,
                        const T* const pos_j, const T* const theta_j, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> pi(pos_i);
            Eigen::Map<const Eigen::Matrix<T, 2, 1>> pj(pos_j);

            T thi = theta_i[0];
            T thj = theta_j[0];

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

    /// This class represents a factor that relates between two SE(2) poses (each pose is expressed as two split variables)
    /// using PoseSE2CostSplitAD as residual cost function
    class FactorTwoPosesSE2SplitAD : public FactorGaussianNoiseModel {
    public:
        FactorTwoPosesSE2SplitAD(std::vector<int> ids) : FactorGaussianNoiseModel(ids, {2, 1, 2, 1},
                {VariableType::Point_XY, VariableType::Angle, VariableType::Point_XY, VariableType::Angle}) {
            m_MeasurementLength = 3;
            m_InfoMatLength = 9;
            m_Measurement = {0.0, 0.0, 0.0};
            m_InformationMatrix = std::vector<double>(m_InfoMatLength, 0.0);
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat( m_InformationMatrix.data());
            mat = Eigen::Matrix<double, 3, 3>::Identity();
        }

        ceres::CostFunction* createFactorCostFcn() const override {
            return new ceres::AutoDiffCostFunction<PoseSE2CostSplitAD, 3, 2, 1, 2, 1>(
                new PoseSE2CostSplitAD(m_Measurement, m_InformationMatrix));
        }

        ceres::LocalParameterization* getVariableLocalParameterization(int variableID) override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Point_XY)
                return new ceres::IdentityParameterization(2);
            else
                return new ceres::AutoDiffLocalParameterization<mw_ceres::AngleOPlus, 1, 1>();
        }


        std::vector<double> getDefaultState(int variableID) const override {
            if (m_MapVariableLPTypes.find(variableID)->second == VariableType::Point_XY)
                return {0.0, 0.0};
            else
                return {0.0};
        }

    };


}

#endif // COMMON_FACTORS_HPP
